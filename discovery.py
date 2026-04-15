#!/usr/bin/env python3
"""
metaboard_bridge.py
────────────────────────────────────────────────────────────────────────────────
BLE bridge, real-time visualiser & WAV recorder for MetaBoard.

Install:
    pip install bleak
    pip install python-osc   (optional – enables OSC forwarding to 127.0.0.1:8888/8889)

Packet layout (114 bytes, Nordic UART Service):
  [  0 :  45]  45 bytes  IMA-ADPCM audio  →  90 int16 PCM samples @ 16 kHz
  [ 45 : 109]  64 bytes  16 × float32 IMU (little-endian):
                           [0-3]   Quaternion      I  J  K  R
                           [4-6]   Linear accel    X  Y  Z  m/s²  (gravity removed)
                           [7-9]   Gyroscope       X  Y  Z  rad/s
                           [10-12] Magnetometer    X  Y  Z  µT
                           [13-15] Raw accel       X  Y  Z  m/s²
  [109]         1 byte   IMU present flag  (0 = absent, 1 = present)
  [110 : 114]   4 bytes  Battery SoC  float32 LE  (%)

BLE:
  Service  6e400001-b5a3-f393-e0a9-e50e24dcca9e
  Notify   6e400002-b5a3-f393-e0a9-e50e24dcca9e  (device → host)
  Write    6e400003-b5a3-f393-e0a9-e50e24dcca9e  (host → device)
"""

import asyncio
import os
import queue
import struct
import sys
import threading
import time
import wave
from collections import deque
from datetime import datetime

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ── optional deps ─────────────────────────────────────────────────────────────
try:
    from bleak import BleakClient, BleakError, BleakScanner
    BLEAK_OK = True
except ImportError:
    BLEAK_OK = False

try:
    from pythonosc import udp_client
    from pythonosc.osc_message_builder import OscMessageBuilder
    OSC_OK = True
except ImportError:
    OSC_OK = False

# ── BLE UUIDs (Nordic UART Service) ──────────────────────────────────────────
UART_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # write   host→device  (device RX)
UART_TX  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # notify  device→host  (device TX) ← subscribe here

# ── packet constants ──────────────────────────────────────────────────────────
PKT_SIZE     = 114
ADPCM_END    = 45
IMU_OFFSET   = 45
IMU_FLAG     = 109
BATT_OFFSET  = 110
SAMPLE_RATE  = 16_000

# ── IMA ADPCM tables (identical to firmware adpcm_codec.c) ───────────────────
_INDEX = [-1, -1, -1, -1,  2,  4,  6,  8,
          -1, -1, -1, -1,  2,  4,  6,  8]

_STEP  = [
       7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
      19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
      50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
     130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
     337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
     876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
    2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
    5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
   15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
]


# ══════════════════════════════════════════════════════════════════════════════
#  ADPCM DECODER
# ══════════════════════════════════════════════════════════════════════════════

class ADPCMDecoder:
    """
    IMA-ADPCM streaming decoder.
    State (predicted_sample, step_index) persists across packets —
    matches the firmware's continuous encoding model.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._pred = 0
        self._si   = 0

    def decode(self, data: bytes) -> list:
        """Decode ADPCM bytes → list of int16 PCM samples (2× len(data))."""
        pred, si = self._pred, self._si
        out = []
        for byte in data:
            # high nibble first — matches firmware adpcm_encode order
            for nibble in ((byte >> 4) & 0xF, byte & 0xF):
                step = _STEP[si]
                diff = step >> 3
                if nibble & 4: diff += step
                if nibble & 2: diff += step >> 1
                if nibble & 1: diff += step >> 2
                pred = pred - diff if (nibble & 8) else pred + diff
                pred = max(-32768, min(32767, pred))
                si   = max(0, min(88, si + _INDEX[nibble]))
                out.append(pred)
        self._pred, self._si = pred, si
        return out


# ══════════════════════════════════════════════════════════════════════════════
#  PACKET PARSER
# ══════════════════════════════════════════════════════════════════════════════

def parse_packet(raw: bytes, decoder: ADPCMDecoder):
    """
    Parse one 114-byte MetaBoard packet.

    Returns
    -------
    pcm   : list[int16] | None   — 90 decoded audio samples
    imu   : dict | None          — sensor readings
    batt  : float | None         — battery SoC in %
    """
    if len(raw) < PKT_SIZE:
        return None, None, None

    pcm = decoder.decode(raw[:ADPCM_END])

    imu = None
    if raw[IMU_FLAG]:
        f = struct.unpack_from('<16f', raw, IMU_OFFSET)
        imu = {
            'quat'      : f[0:4],    # I  J  K  R
            'lin_accel' : f[4:7],    # m/s²  (gravity removed)
            'gyro'      : f[7:10],   # rad/s
            'mag'       : f[10:13],  # µT
            'raw_accel' : f[13:16],  # m/s²
        }

    batt = None
    if len(raw) >= BATT_OFFSET + 4:
        (batt,) = struct.unpack_from('<f', raw, BATT_OFFSET)

    return pcm, imu, batt


# ══════════════════════════════════════════════════════════════════════════════
#  COLOUR PALETTE  (dark theme)
# ══════════════════════════════════════════════════════════════════════════════
BG      = "#0f172a"   # page background
PANEL   = "#1e293b"   # card / listbox background
BORDER  = "#334155"   # subtle border
ACCENT  = "#3b82f6"   # blue
GREEN   = "#22c55e"   # good / audio trace
YELLOW  = "#f59e0b"   # warning / battery
RED     = "#ef4444"   # error / record
FG      = "#f1f5f9"   # primary text
FG2     = "#94a3b8"   # secondary text
MONO    = ("Courier", 10)
MONO_SM = ("Courier", 9)


# ══════════════════════════════════════════════════════════════════════════════
#  GUI APPLICATION
# ══════════════════════════════════════════════════════════════════════════════

class MetaBoardBridge(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("MetaBoard Bridge")
        self.configure(bg=BG)
        self.minsize(720, 640)
        self.resizable(True, True)

        # ── internal state ────────────────────────────────────────────────────
        self._devices   : dict = {}       # display-string → BLEDevice
        self._client    = None
        self._connected = False
        self._decoder   = ADPCMDecoder()
        self._data_q    : queue.Queue = queue.Queue(maxsize=300)

        # packet stats
        self._total_pkts  = 0
        self._rate_count  = 0
        self._rate_t0     = time.time()
        self._pkt_rate    = 0.0

        # recording
        self._recording   = False
        self._rec_pcm     : list = []     # flat list of int16

        # oscilloscope ring buffer (~200 ms of audio)
        self._wave_buf = deque(maxlen=SAMPLE_RATE // 5)

        # asyncio event loop (BLE lives here)
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        # OSC clients (optional)
        self._osc = []
        if OSC_OK:
            for port in (8888, 8889):
                try:
                    self._osc.append(udp_client.SimpleUDPClient("127.0.0.1", port))
                except Exception:
                    pass

        self._build_ui()
        self.after(40, self._poll_queue)          # 25 Hz UI refresh
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ══════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        # ── header bar ────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=ACCENT, padx=14, pady=9)
        hdr.grid(row=0, column=0, sticky="ew")
        tk.Label(hdr, text="MetaBoard  Bridge",
                 bg=ACCENT, fg="white", font=("Helvetica", 13, "bold")).pack(side="left")
        self._status_var = tk.StringVar(value="●  idle")
        tk.Label(hdr, textvariable=self._status_var,
                 bg=ACCENT, fg="#bfdbfe", font=("Helvetica", 10)).pack(side="right")

        # ── device panel ──────────────────────────────────────────────────────
        dev = self._card(self, "Devices")
        dev.grid(row=1, column=0, sticky="ew", padx=10, pady=(8, 4))
        dev.columnconfigure(0, weight=1)

        self._dev_lb = tk.Listbox(
            dev, height=4, bg=PANEL, fg=FG, font=MONO,
            selectbackground=ACCENT, selectforeground="white",
            relief="flat", bd=0, activestyle="none",
            highlightthickness=1, highlightbackground=BORDER,
        )
        self._dev_lb.grid(row=0, column=0, sticky="ew", rowspan=3, padx=(0, 8))

        bf = tk.Frame(dev, bg=BG)
        bf.grid(row=0, column=1, sticky="n")
        self._scan_btn = self._mkbtn(bf, "⟳  Scan",       self._do_scan,       ACCENT)
        self._conn_btn = self._mkbtn(bf, "⚡  Connect",    self._do_connect,    GREEN)
        self._disc_btn = self._mkbtn(bf, "✕  Disconnect",  self._do_disconnect, BORDER)
        for b in (self._scan_btn, self._conn_btn, self._disc_btn):
            b.pack(fill="x", pady=2)
        self._disc_btn.config(state="disabled")

        # ── main data area ────────────────────────────────────────────────────
        mid = tk.Frame(self, bg=BG)
        mid.grid(row=2, column=0, sticky="nsew", padx=10, pady=4)
        mid.columnconfigure(0, weight=3)
        mid.columnconfigure(1, weight=1)
        mid.rowconfigure(0, weight=1)

        # ── IMU panel ─────────────────────────────────────────────────────────
        imu_card = self._card(mid, "IMU  (BNO08x)")
        imu_card.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        imu_card.columnconfigure(tuple(range(1, 5)), weight=1)

        self._imu_sv : dict = {}
        rows = [
            ("Quaternion",    "quat",       ["I", "J", "K", "R"]),
            ("Lin Accel m/s²","lin_accel",  ["X", "Y", "Z"]),
            ("Gyro  rad/s",   "gyro",       ["X", "Y", "Z"]),
            ("Mag  µT",       "mag",        ["X", "Y", "Z"]),
            ("Raw Accel m/s²","raw_accel",  ["X", "Y", "Z"]),
        ]
        for r, (label, key, axes) in enumerate(rows):
            tk.Label(imu_card, text=label, bg=BG, fg=FG2,
                     font=("Helvetica", 9), anchor="w", width=15).grid(
                row=r, column=0, sticky="w", pady=3)
            for c, ax in enumerate(axes):
                sv = tk.StringVar(value="  –.––––")
                self._imu_sv[f"{key}_{c}"] = sv
                frm = tk.Frame(imu_card, bg=PANEL, relief="flat")
                frm.grid(row=r, column=c + 1, padx=2, pady=2, sticky="ew")
                tk.Label(frm, text=ax, bg=PANEL, fg=FG2,
                         font=("Helvetica", 8), padx=3).pack(side="left")
                tk.Label(frm, textvariable=sv, bg=PANEL, fg=GREEN,
                         font=MONO, anchor="e", width=9, padx=3).pack(side="right")

        # ── right column: battery + stats ─────────────────────────────────────
        right = tk.Frame(mid, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")

        batt_card = self._card(right, "Battery")
        batt_card.pack(fill="x")
        self._batt_sv = tk.StringVar(value="–.–%")
        tk.Label(batt_card, textvariable=self._batt_sv, bg=BG, fg=YELLOW,
                 font=("Helvetica", 22, "bold")).pack(pady=(4, 2))
        self._batt_cv = tk.Canvas(batt_card, height=14, bg=PANEL,
                                  relief="flat", bd=0, highlightthickness=0)
        self._batt_cv.pack(fill="x", pady=(0, 4))

        stats_card = self._card(right, "Stats")
        stats_card.pack(fill="x", pady=(6, 0))
        self._rate_sv  = tk.StringVar(value="–  pkt/s")
        self._total_sv = tk.StringVar(value="0  pkts")
        self._drop_sv  = tk.StringVar(value="0  drops")
        for label_text, sv in [
            ("Rate",   self._rate_sv),
            ("Total",  self._total_sv),
            ("Drops",  self._drop_sv),
        ]:
            row_f = tk.Frame(stats_card, bg=BG)
            row_f.pack(fill="x", pady=1)
            tk.Label(row_f, text=f"{label_text}:", bg=BG, fg=FG2,
                     font=("Helvetica", 8), width=6, anchor="w").pack(side="left")
            tk.Label(row_f, textvariable=sv, bg=BG, fg=FG,
                     font=MONO_SM).pack(side="left")

        # ── OSC status ────────────────────────────────────────────────────────
        osc_card = self._card(right, "OSC Out")
        osc_card.pack(fill="x", pady=(6, 0))
        osc_msg = "127.0.0.1:8888/8889" if (OSC_OK and self._osc) else "python-osc not installed"
        osc_col = FG2 if (OSC_OK and self._osc) else RED
        tk.Label(osc_card, text=osc_msg, bg=BG, fg=osc_col,
                 font=("Helvetica", 8), wraplength=130).pack(anchor="w")

        # ── audio panel ───────────────────────────────────────────────────────
        audio_card = self._card(self, "Audio  ·  IMA-ADPCM → 16 kHz PCM  ·  mono  ·  16-bit")
        audio_card.grid(row=3, column=0, sticky="ew", padx=10, pady=(4, 10))
        audio_card.columnconfigure(0, weight=1)

        self._wave_cv = tk.Canvas(
            audio_card, height=90, bg=PANEL, relief="flat", bd=0,
            highlightthickness=1, highlightbackground=BORDER,
        )
        self._wave_cv.grid(row=0, column=0, sticky="ew", pady=(4, 6))
        self._wave_cv.bind("<Configure>", lambda _e: self._draw_waveform())

        ctrl = tk.Frame(audio_card, bg=BG)
        ctrl.grid(row=1, column=0, sticky="ew")
        ctrl.columnconfigure(4, weight=1)

        self._rec_btn  = self._mkbtn(ctrl, "⏺  Record",   self._do_record, RED)
        self._stop_btn = self._mkbtn(ctrl, "⏹  Stop",     self._do_stop,   BORDER)
        self._save_btn = self._mkbtn(ctrl, "💾  Save WAV", self._do_save,   ACCENT)
        self._rec_btn.grid( row=0, column=0, padx=(0, 4))
        self._stop_btn.grid(row=0, column=1, padx=4)
        self._save_btn.grid(row=0, column=2, padx=4)
        self._stop_btn.config(state="disabled")
        self._save_btn.config(state="disabled")
        self._rec_btn.config( state="disabled")

        self._level_sv = tk.StringVar(value="")
        tk.Label(ctrl, textvariable=self._level_sv, bg=BG, fg=FG2,
                 font=MONO_SM).grid(row=0, column=4, sticky="e")

        # ── log bar ───────────────────────────────────────────────────────────
        tk.Frame(self, bg=BORDER, height=1).grid(row=4, column=0, sticky="ew")
        self._log_sv = tk.StringVar(value="Ready.  Install deps: pip install bleak")
        tk.Label(self, textvariable=self._log_sv, bg=BG, fg=FG2,
                 font=("Helvetica", 9), anchor="w", padx=10, pady=4
                 ).grid(row=5, column=0, sticky="ew")

    # ── widget helpers ────────────────────────────────────────────────────────

    def _card(self, parent, title):
        return tk.LabelFrame(
            parent, text=f"  {title}  ", bg=BG, fg=FG2,
            font=("Helvetica", 8), bd=1, relief="groove", padx=8, pady=6,
        )

    def _mkbtn(self, parent, text, cmd, color):
        return tk.Button(
            parent, text=text, command=cmd,
            bg=color, fg="white", font=("Helvetica", 9, "bold"),
            relief="flat", padx=10, pady=5,
            activebackground=color, activeforeground="white",
            cursor="hand2", disabledforeground="#6b7280",
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  BLE  –  SCAN
    # ══════════════════════════════════════════════════════════════════════════

    def _do_scan(self):
        self._set_status("●  scanning…", YELLOW)
        self._log("Scanning for BLE devices…")
        self._scan_btn.config(state="disabled")
        asyncio.run_coroutine_threadsafe(self._scan(), self._loop)

    async def _scan(self):
        try:
            devs = await BleakScanner.discover(timeout=5.0)
            self.after(0, self._on_scan_done, devs)
        except Exception as exc:
            self.after(0, self._log, f"Scan error: {exc}")
            self.after(0, lambda: self._scan_btn.config(state="normal"))
            self.after(0, self._set_status, "●  idle", FG2)

    def _on_scan_done(self, devs):
        self._devices.clear()
        self._dev_lb.delete(0, "end")
        def _rssi(d):
            return getattr(d, "rssi", None) or -999
        metabow = [d for d in devs if (d.name or "").lower().startswith("metabow")]
        for d in sorted(metabow, key=_rssi, reverse=True):
            name  = d.name or "Unknown"
            rssi_val = getattr(d, "rssi", None)
            rssi  = f"{rssi_val} dBm" if rssi_val else "?"
            entry = f"{name}  [{d.address}]  {rssi}"
            self._devices[entry] = d
            self._dev_lb.insert("end", entry)
        found = len(metabow)
        self._log(f"Found {found} MetaBoard device(s).{' Select one and click Connect.' if found else ' None nearby — try again.'}")
        self._scan_btn.config(state="normal")
        self._set_status("●  idle", FG2)

    # ══════════════════════════════════════════════════════════════════════════
    #  BLE  –  CONNECT / DISCONNECT
    # ══════════════════════════════════════════════════════════════════════════

    def _do_connect(self):
        sel = self._dev_lb.curselection()
        if not sel:
            messagebox.showwarning("MetaBoard Bridge", "Select a device first.")
            return
        entry  = self._dev_lb.get(sel[0])
        device = self._devices.get(entry)
        if device is None:
            return
        self._set_status("●  connecting…", YELLOW)
        self._log(f"Connecting to {device.name or device.address}…")
        self._conn_btn.config(state="disabled")
        asyncio.run_coroutine_threadsafe(self._connect(device), self._loop)

    async def _connect(self, device):
        try:
            self._client = BleakClient(
                device,
                disconnected_callback=lambda _c: self.after(0, self._on_disconnected),
            )
            await self._client.connect()

            # Subscribe to TX characteristic (device→host, notify)
            # Nordic UART naming: TX = device transmits = 6e400003
            tx_char = None
            for svc in self._client.services:
                for ch in svc.characteristics:
                    if ch.uuid.lower() == UART_TX.lower():
                        tx_char = ch
                        break

            notify_uuid = tx_char.uuid if tx_char else UART_TX
            await self._client.start_notify(notify_uuid, self._on_ble_data)
            self.after(0, self._on_connected, device.name or device.address)

        except Exception as exc:
            self.after(0, self._log,        f"Connection failed: {exc}")
            self.after(0, self._set_status, "●  error", RED)
            self.after(0, lambda: self._conn_btn.config(state="normal"))

    def _on_connected(self, name: str):
        self._connected = True
        self._decoder.reset()
        self._rate_count = 0
        self._rate_t0    = time.time()
        self._set_status(f"●  {name}", GREEN)
        self._log(f"Connected to {name}.  Receiving data…")
        self._conn_btn.config(state="disabled")
        self._disc_btn.config(state="normal")
        self._rec_btn.config(state="normal")

    def _do_disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)

    async def _disconnect(self):
        try:
            if self._client and self._client.is_connected:
                await self._client.disconnect()
        except Exception:
            pass
        self.after(0, self._on_disconnected)

    def _on_disconnected(self):
        if not self._connected:
            return
        self._connected = False
        self._set_status("●  idle", FG2)
        self._log("Disconnected.")
        self._conn_btn.config(state="normal")
        self._disc_btn.config(state="disabled")
        self._rec_btn.config(state="disabled")
        if self._recording:
            self._do_stop()

    # ══════════════════════════════════════════════════════════════════════════
    #  BLE DATA CALLBACK
    # ══════════════════════════════════════════════════════════════════════════

    def _on_ble_data(self, _sender, data: bytearray):
        try:
            self._data_q.put_nowait(bytes(data))
        except queue.Full:
            pass   # drop silently; counted as drop below

    # ══════════════════════════════════════════════════════════════════════════
    #  MAIN LOOP  –  QUEUE POLL (runs on tkinter thread via after())
    # ══════════════════════════════════════════════════════════════════════════

    _drops = 0

    def _poll_queue(self):
        limit = 12   # max packets per frame to avoid starving tkinter
        processed = 0
        while processed < limit:
            try:
                raw = self._data_q.get_nowait()
            except queue.Empty:
                break
            pcm, imu, batt = parse_packet(raw, self._decoder)
            if pcm is not None:
                self._total_pkts += 1
                self._rate_count += 1
                self._update_display(pcm, imu, batt)
            else:
                self._drops += 1
            processed += 1

        # update rate every second
        now = time.time()
        if now - self._rate_t0 >= 1.0:
            self._pkt_rate   = self._rate_count / (now - self._rate_t0)
            self._rate_count = 0
            self._rate_t0    = now
            self._rate_sv.set(f"{self._pkt_rate:.1f}  pkt/s")
            self._total_sv.set(f"{self._total_pkts}  pkts")
            self._drop_sv.set(f"{self._drops}  drops")

        self.after(40, self._poll_queue)

    # ══════════════════════════════════════════════════════════════════════════
    #  DISPLAY UPDATE
    # ══════════════════════════════════════════════════════════════════════════

    def _update_display(self, pcm, imu, batt):
        # ── waveform ──────────────────────────────────────────────────────────
        self._wave_buf.extend(pcm)
        if self._recording:
            self._rec_pcm.extend(pcm)
        self._draw_waveform()

        # ── level meter ───────────────────────────────────────────────────────
        peak  = max(abs(s) for s in pcm) if pcm else 0
        ratio = peak / 32768.0
        bars  = int(ratio * 24)
        self._level_sv.set(f"{'█' * bars}{'░' * (24 - bars)}  {int(ratio * 100):3d}%")

        # ── IMU ───────────────────────────────────────────────────────────────
        if imu:
            for key, vals in imu.items():
                for i, v in enumerate(vals):
                    sv_key = f"{key}_{i}"
                    if sv_key in self._imu_sv:
                        self._imu_sv[sv_key].set(f"{v:+.4f}")

        # ── battery ───────────────────────────────────────────────────────────
        if batt is not None:
            self._batt_sv.set(f"{batt:.1f}%")
            self._draw_battery(batt)

        # ── OSC forwarding ────────────────────────────────────────────────────
        if self._osc:
            self._send_osc(imu, batt, pcm)

    # ══════════════════════════════════════════════════════════════════════════
    #  DRAWING
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_waveform(self):
        cv = self._wave_cv
        W  = cv.winfo_width()
        H  = cv.winfo_height()
        if W < 4 or H < 4:
            return
        cv.delete("all")
        cv.create_rectangle(0, 0, W, H, fill=PANEL, outline="")
        # centre line
        cy = H // 2
        cv.create_line(0, cy, W, cy, fill=BORDER, dash=(4, 4))
        # ±50% guides
        g = int(cy * 0.5)
        cv.create_line(0, cy - g, W, cy - g, fill=BORDER, dash=(2, 6))
        cv.create_line(0, cy + g, W, cy + g, fill=BORDER, dash=(2, 6))

        samples = list(self._wave_buf)
        if len(samples) < 2:
            return

        step = max(1, len(samples) // W)
        pts  = []
        for px in range(W):
            idx = min(px * step, len(samples) - 1)
            y   = cy - int((samples[idx] / 32768.0) * (cy - 3))
            pts.extend((px, y))

        if len(pts) >= 4:
            cv.create_line(*pts, fill=GREEN, width=1, smooth=False)

        # REC badge
        if self._recording:
            cv.create_oval(W - 20, 6, W - 8, 18, fill=RED, outline="")
            cv.create_text(W - 14, 12, text="●", fill="white", font=("Helvetica", 6))

    def _draw_battery(self, pct: float):
        cv = self._batt_cv
        W  = cv.winfo_width()
        H  = cv.winfo_height()
        if W < 4:
            return
        cv.delete("all")
        cv.create_rectangle(1, 1, W - 1, H - 1, fill=BORDER, outline=BORDER)
        fill_w = max(0, int((pct / 100.0) * (W - 2)))
        color  = GREEN if pct > 30 else (YELLOW if pct > 15 else RED)
        if fill_w > 0:
            cv.create_rectangle(1, 1, 1 + fill_w, H - 1, fill=color, outline="")

    # ══════════════════════════════════════════════════════════════════════════
    #  OSC
    # ══════════════════════════════════════════════════════════════════════════

    def _send_osc(self, imu, batt, pcm):
        if not self._osc:
            return
        try:
            if imu:
                msg = OscMessageBuilder(address="/metaboard/motion")
                for key in ("quat", "lin_accel", "gyro", "mag", "raw_accel"):
                    for v in imu[key]:
                        msg.add_arg(float(v))
                built = msg.build()
                for client in self._osc:
                    client.send(built)

            if batt is not None:
                msg = OscMessageBuilder(address="/metaboard/battery/percentage")
                msg.add_arg(float(batt))
                built = msg.build()
                for client in self._osc:
                    client.send(built)
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    #  RECORDING
    # ══════════════════════════════════════════════════════════════════════════

    def _do_record(self):
        if not self._connected:
            messagebox.showwarning("MetaBoard Bridge", "Not connected to a device.")
            return
        self._rec_pcm.clear()
        self._recording = True
        self._rec_btn.config( state="disabled")
        self._stop_btn.config(state="normal")
        self._save_btn.config(state="disabled")
        self._log("● Recording…")
        self._set_status("●  REC", RED)

    def _do_stop(self):
        self._recording = False
        self._stop_btn.config(state="disabled")
        n_samples = len(self._rec_pcm)
        if n_samples > 0:
            dur = n_samples / SAMPLE_RATE
            self._log(f"Recording stopped — {n_samples} samples  ({dur:.2f} s).")
            self._save_btn.config(state="normal")
        else:
            self._log("Recording stopped — no samples captured.")
        self._rec_btn.config(state="normal" if self._connected else "disabled")
        if self._connected:
            self._set_status(f"●  connected", GREEN)

    def _do_save(self):
        if not self._rec_pcm:
            messagebox.showwarning("MetaBoard Bridge", "No recorded audio.")
            return
        default = f"metaboard_{datetime.now():%Y%m%d_%H%M%S}.wav"
        path = filedialog.asksaveasfilename(
            defaultextension=".wav",
            filetypes=[("WAV audio", "*.wav"), ("All files", "*.*")],
            initialfile=default,
        )
        if not path:
            return
        try:
            n      = len(self._rec_pcm)
            packed = struct.pack(f"<{n}h", *self._rec_pcm)
            with wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)         # 16-bit signed
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(packed)
            size_kb = os.path.getsize(path) // 1024
            dur     = n / SAMPLE_RATE
            self._log(f"Saved {n} samples  ({dur:.2f} s,  {size_kb} KB)  →  {os.path.basename(path)}")
            messagebox.showinfo("MetaBoard Bridge", f"WAV saved:\n{path}")
        except Exception as exc:
            messagebox.showerror("MetaBoard Bridge", f"Save failed:\n{exc}")

    # ══════════════════════════════════════════════════════════════════════════
    #  HELPERS
    # ══════════════════════════════════════════════════════════════════════════

    def _set_status(self, text: str, color: str = FG2):
        self._status_var.set(text)
        # find the status label and recolour it
        for widget in self.winfo_children():
            if isinstance(widget, tk.Frame) and widget.cget("bg") == ACCENT:
                for child in widget.winfo_children():
                    if isinstance(child, tk.Label) and child.cget("textvariable"):
                        if str(child.cget("textvariable")) == str(self._status_var):
                            child.config(fg=color)

    def _log(self, msg: str):
        self._log_sv.set(msg)
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}]  {msg}")

    def _on_close(self):
        if self._connected:
            asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
            time.sleep(0.25)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self.destroy()


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main():
    if not BLEAK_OK:
        print("━" * 60)
        print("  ERROR: bleak is not installed.")
        print()
        print("  Install it with:")
        print("      pip install bleak")
        print()
        print("  Optional (OSC forwarding):")
        print("      pip install python-osc")
        print("━" * 60)
        sys.exit(1)

    print("MetaBoard Bridge  —  starting")
    print(f"  OSC forwarding: {'enabled  (127.0.0.1:8888 + 8889)' if OSC_OK else 'disabled (pip install python-osc)'}")
    app = MetaBoardBridge()
    app.mainloop()


if __name__ == "__main__":
    main()
