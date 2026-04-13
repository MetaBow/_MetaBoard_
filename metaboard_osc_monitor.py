#!/usr/bin/env python3
"""
MetaBoard OSC Monitor - Diagnostic Tool
========================================
Simplified tool for monitoring BLE-to-OSC data flow:
- Connects to MetaBoard device via BLE
- Identifies incoming OSC routes dynamically
- Tracks incoming BLE packet sample rate (detects gaps/variations)
- Tracks outgoing OSC message sample rate to port 8888
- Logs all data to CSV/JSON for debugging
- Real-time UI with connection controls and sample rate visualization
"""

import asyncio
import concurrent.futures
import os
import struct
import sys
import time
import json
import csv
import threading
import uuid
from datetime import datetime
from collections import deque, defaultdict
from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional
from pathlib import Path

# Project root (for `metabow` + `audio_feature_extractor`)
_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from metabow.audio import AudioSubsystem, AudioSubsystemConfig

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

from bleak import BleakScanner, BleakClient
from pythonosc import udp_client

# ============================================================================
# Configuration
# ============================================================================

NUS_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Nordic UART Service RX
OSC_PORT = 8888
OSC_HOST = "127.0.0.1"

# Packet structure (from firmware)
ADPCM_HEADER_SIZE = 3  # NEW: int16 predicted_sample + uint8 step_index
ADPCM_BLOCK_SIZE = 45
IMU_DATA_SIZE = 64  # 16 floats
IMU_FLAG_SIZE = 1
BATTERY_DATA_SIZE = 4
PACKET_SIZE_OLD = ADPCM_BLOCK_SIZE + IMU_DATA_SIZE + IMU_FLAG_SIZE + BATTERY_DATA_SIZE  # 114 bytes (old)
PACKET_SIZE_NEW = ADPCM_HEADER_SIZE + ADPCM_BLOCK_SIZE + IMU_DATA_SIZE + IMU_FLAG_SIZE + BATTERY_DATA_SIZE  # 117 bytes (new)

# Sample rate tracking
SAMPLE_RATE_WINDOW = 100  # Number of samples for rate calculation
HISTORY_SIZE = 1000  # Keep last 1000 measurements

# Gap and rate drop detection thresholds
PACKET_GAP_WARNING_MS = 100.0  # Warn if gap > 100ms
PACKET_GAP_CRITICAL_MS = 500.0  # Critical if gap > 500ms
OSC_GAP_WARNING_MS = 200.0  # Warn if OSC gap > 200ms
OSC_GAP_CRITICAL_MS = 1000.0  # Critical if OSC gap > 1000ms
RATE_DROP_THRESHOLD = 0.2  # Warn if rate drops > 20% from expected
RATE_DROP_CRITICAL = 0.5  # Critical if rate drops > 50%
EXPECTED_PACKET_RATE = 177.0  # Expected packets/sec from firmware


def _is_portaudio_error(exc: BaseException) -> bool:
    """True if ``exc`` is from sounddevice/PortAudio (VB-Cable write path)."""
    try:
        import sounddevice as sd

        pa_err = getattr(sd, "PortAudioError", None)
        if pa_err is not None and isinstance(exc, pa_err):
            return True
    except ImportError:
        pass
    return type(exc).__name__ == "PortAudioError"


# ============================================================================
# ADPCM Decoder
# ============================================================================

class ADPCMDecoder:
    """IMA ADPCM decoder - matches firmware implementation exactly"""
    
    INDEX_TABLE = [
        -1, -1, -1, -1, 2, 4, 6, 8,
        -1, -1, -1, -1, 2, 4, 6, 8
    ]
    
    STEP_TABLE = [
        7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
        19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
        50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
        130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
        337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
        876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
        2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
        5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
        15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
    ]
    
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.reset()
    
    def reset(self):
        self.predicted_sample = 0
        self.step_index = 0
    
    def set_state(self, predicted_sample, step_index):
        """
        Set decoder state from firmware header (allows re-sync after packet drops).
        
        Args:
            predicted_sample: int16 predicted sample value from firmware
            step_index: uint8 step index from firmware (0-88)
        """
        self.predicted_sample = int(predicted_sample)
        self.step_index = int(step_index) & 0xFF
        # Clamp step_index to valid range
        self.step_index = self._clamp(self.step_index, 0, 88)
    
    def _clamp(self, v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)
    
    def decode_sample(self, adpcm_nibble):
        step = self.STEP_TABLE[self.step_index]
        
        diff = 0
        if adpcm_nibble & 4:
            diff += step
        if adpcm_nibble & 2:
            diff += step >> 1
        if adpcm_nibble & 1:
            diff += step >> 2
        diff += step >> 3
        
        if adpcm_nibble & 8:
            self.predicted_sample -= diff
        else:
            self.predicted_sample += diff
        
        self.predicted_sample = self._clamp(self.predicted_sample, -32768, 32767)
        self.step_index += self.INDEX_TABLE[adpcm_nibble & 0x0F]
        self.step_index = self._clamp(self.step_index, 0, 88)
        
        return self.predicted_sample
    
    def decode(self, adpcm_data):
        """Decode ADPCM to list of int16 samples - state persists across calls"""
        out = []
        for byte_val in adpcm_data:
            high_nibble = (byte_val >> 4) & 0x0F
            out.append(self.decode_sample(high_nibble))
            
            low_nibble = byte_val & 0x0F
            out.append(self.decode_sample(low_nibble))
        
        return out

# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class PacketStats:
    """Statistics for a single packet"""
    timestamp: float
    packet_size: int
    has_imu: bool
    battery_soc: float
    gap_from_previous: float  # Time since last packet (seconds)
    
@dataclass
class OSCMessageStats:
    """Statistics for an OSC message"""
    timestamp: float
    path: str
    value_count: int
    gap_from_previous: float  # Time since last message to same path
    
@dataclass
class RouteInfo:
    """Information about a discovered OSC route"""
    path: str
    first_seen: float
    last_seen: float
    message_count: int
    data_type: str
    sample_rate: float  # Messages per second
    last_value: Any
    expected_rate: float = 0.0  # Expected rate for this route
    gap_warnings: int = 0  # Count of gap warnings
    rate_drop_warnings: int = 0  # Count of rate drop warnings

@dataclass
class GapAlert:
    """Alert for detected gap"""
    timestamp: float
    type: str  # 'packet' or 'osc'
    path: str
    gap_seconds: float
    severity: str  # 'warning' or 'critical'
    
@dataclass
class RateDropAlert:
    """Alert for detected rate drop"""
    timestamp: float
    type: str  # 'packet' or 'osc'
    path: str
    current_rate: float
    expected_rate: float
    drop_percent: float

class SampleRateTracker:
    """Tracks sample rate over a sliding window"""
    def __init__(self, window_size: int = SAMPLE_RATE_WINDOW):
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)
        self.rates = deque(maxlen=HISTORY_SIZE)
        
    def add_sample(self, timestamp: float) -> float:
        """Add a timestamp and return current sample rate"""
        self.timestamps.append(timestamp)
        
        if len(self.timestamps) < 2:
            return 0.0
        
        # Calculate rate from time span
        time_span = self.timestamps[-1] - self.timestamps[0]
        if time_span > 0:
            rate = (len(self.timestamps) - 1) / time_span
        else:
            rate = 0.0
        
        self.rates.append(rate)
        return rate
    
    def get_current_rate(self) -> float:
        """Get most recent sample rate"""
        return self.rates[-1] if self.rates else 0.0
    
    def get_average_rate(self) -> float:
        """Get average rate over history"""
        if not self.rates:
            return 0.0
        return sum(self.rates) / len(self.rates)
    
    def get_rate_history(self) -> List[float]:
        """Get full rate history"""
        return list(self.rates)

# ============================================================================
# Main Monitor Class
# ============================================================================

class MetaBoardOSCMonitor:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MetaBoard OSC Monitor")
        self.root.geometry("1200x800")
        
        # BLE connection
        self.client: Optional[BleakClient] = None
        self.connected = False
        self.device_address: Optional[str] = None
        
        # OSC client
        self.osc_client = udp_client.SimpleUDPClient(OSC_HOST, OSC_PORT)
        self._osc_lock = threading.Lock()

        # Session alignment (shared with metabow.audio alignment log + CSV)
        self.session_id: Optional[str] = None
        self.session_mono_t0: Optional[float] = None
        
        # ADPCM decoder for audio
        self.adpcm_decoder = ADPCMDecoder(sample_rate=16000)
        
        # Data tracking
        self.packet_tracker = SampleRateTracker()
        self.osc_trackers: Dict[str, SampleRateTracker] = {}  # Per-route trackers
        self.routes: Dict[str, RouteInfo] = {}
        self.packet_history: deque = deque(maxlen=HISTORY_SIZE)
        self.osc_history: deque = deque(maxlen=HISTORY_SIZE)
        # Full-session BLE count (packet_history is capped — use this for delivery %)
        self.session_packet_count: int = 0
        self.session_wall_start: Optional[float] = None
        
        # Alert tracking for real-time detection
        self.gap_alerts: deque = deque(maxlen=100)  # Recent gap alerts
        self.rate_drop_alerts: deque = deque(maxlen=100)  # Recent rate drop alerts
        self.last_packet_time: Optional[float] = None
        self.last_osc_times: Dict[str, float] = {}  # Last message time per route
        
        # Logging
        self.logging_enabled = False
        self.log_file: Optional[Path] = None
        self.csv_writer = None
        self.csv_file = None

        # Modular audio pipeline (metabow.audio) — optional; toggles in UI
        self.audio_subsystem: Optional[AudioSubsystem] = None
        # Cleared at disconnect start so BLE callbacks do not push after teardown begins
        self._audio_pipeline_accepting: bool = False
        self._shutting_down: bool = False
        self._window_close_handled: bool = False
        # BLE notify callbacks run on the asyncio loop thread; heavy work must yield the loop
        # so disconnect/scan/connect coroutines can run (see bleak CoreBluetooth notify path).
        self._packet_state_lock = threading.Lock()
        self._ble_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="ble_oscmon"
        )

        # UI update
        self.update_interval = 100  # ms
        self.last_update = time.time()
        
        # Create UI
        self.create_ui()
        
        # Async event loop for BLE operations
        self.loop = None
        self.loop_thread = None
        self.loop_ready = threading.Event()
        
        # Start async event loop in background thread
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop_ready.set()
            self.loop.run_forever()
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
        # Wait for loop to be ready (with timeout to avoid blocking UI)
        if not self.loop_ready.wait(timeout=1.0):
            print("Warning: Event loop initialization timeout")
        
    def create_ui(self):
        """Create the user interface"""
        # Top frame: Connection controls
        conn_frame = ttk.Frame(self.root, padding="10")
        conn_frame.pack(fill=tk.X)
        
        ttk.Label(conn_frame, text="Device:").pack(side=tk.LEFT, padx=5)
        self.device_var = tk.StringVar(value="Not connected")
        ttk.Label(conn_frame, textvariable=self.device_var, width=30).pack(side=tk.LEFT, padx=5)
        
        self.scan_btn = ttk.Button(conn_frame, text="Scan", command=self.start_scan)
        self.scan_btn.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.start_connect, state=tk.DISABLED)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.start_disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Status frame
        status_frame = ttk.Frame(self.root, padding="10")
        status_frame.pack(fill=tk.X)
        
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT, padx=5)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(status_frame, textvariable=self.status_var, foreground="red").pack(side=tk.LEFT, padx=5)
        
        # Logging controls
        ttk.Label(status_frame, text="Logging:").pack(side=tk.LEFT, padx=10)
        self.logging_var = tk.BooleanVar()
        ttk.Checkbutton(status_frame, text="Enable", variable=self.logging_var, 
                       command=self.toggle_logging).pack(side=tk.LEFT, padx=5)
        ttk.Button(status_frame, text="Choose Log File", command=self.choose_log_file).pack(side=tk.LEFT, padx=5)

        # Modular audio (features / WAV+ADPCM / VB-Cable) — active while connected
        # ML feature extraction (librosa) is always enabled on connect; no separate toggle.
        audio_frame = ttk.LabelFrame(self.root, text="Audio options (enable before Connect)", padding="8")
        audio_frame.pack(fill=tk.X, padx=10, pady=(0, 4))
        self.audio_var_record = tk.BooleanVar(value=False)
        self.audio_var_vb = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            audio_frame,
            text="Record WAV + ADPCM",
            variable=self.audio_var_record,
        ).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Checkbutton(
            audio_frame,
            text="Stream → VB-Cable (44.1 kHz)",
            variable=self.audio_var_vb,
        ).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(
            audio_frame,
            text="ML features (librosa) extracted automatically. WAV → ~/Documents/MetaBow_Data",
            foreground="gray",
        ).pack(side=tk.LEFT, padx=(8, 0))
        
        # Main content: Split into left (stats) and right (graphs)
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel: Statistics and routes
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5)
        left_panel.config(width=400)
        
        # Statistics
        stats_frame = ttk.LabelFrame(left_panel, text="Statistics", padding="10")
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.stats_text = scrolledtext.ScrolledText(stats_frame, height=12, width=45)
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        
        # Alerts display
        alerts_frame = ttk.LabelFrame(left_panel, text="Real-time Alerts", padding="10")
        alerts_frame.pack(fill=tk.X, pady=5)
        
        self.alerts_text = scrolledtext.ScrolledText(alerts_frame, height=5, width=45, 
                                                     foreground="red", font=("Courier", 9))
        self.alerts_text.pack(fill=tk.BOTH, expand=True)
        
        # Routes list
        routes_frame = ttk.LabelFrame(left_panel, text="Discovered Routes", padding="10")
        routes_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.routes_listbox = tk.Listbox(routes_frame, height=10)
        self.routes_listbox.pack(fill=tk.BOTH, expand=True)
        
        # Right panel: Graphs
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        
        self.canvas = FigureCanvasTkAgg(self.fig, right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Start UI update loop
        self.update_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def choose_log_file(self):
        """Choose log file location"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.log_file = Path(filename)
            self.logging_var.set(True)
            self.toggle_logging()
    
    def toggle_logging(self):
        """Enable/disable logging"""
        if self.logging_var.get():
            if not self.log_file:
                messagebox.showwarning("No File", "Please choose a log file first")
                self.logging_var.set(False)
                return
            
            self.start_logging()
        else:
            self.stop_logging()
    
    def start_logging(self):
        """Start logging to file"""
        if not self.log_file:
            return
        
        self.logging_enabled = True
        
        if self.log_file.suffix.lower() == '.csv':
            # Drop stale JSON buffer so a prior session cannot confuse stop_logging
            if hasattr(self, 'log_entries'):
                del self.log_entries
            self.csv_file = open(self.log_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header
            self.csv_writer.writerow([
                'timestamp', 'type', 'path', 'packet_size', 'has_imu', 'battery_soc',
                'gap_seconds', 'sample_rate', 'value_count', 'values',
                'session_id', 'host_mono_sec', 'cumulative_pcm_samples', 'ble_packet_seq',
            ])
        else:
            # JSON array on disk — any non-.csv path (e.g. .json, .txt, no extension, "All files")
            self.csv_file = None
            self.csv_writer = None
            self.log_entries = []
    
    def stop_logging(self):
        """Stop logging and save file"""
        self.logging_enabled = False
        
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        if (
            self.log_file
            and self.log_file.suffix.lower() != '.csv'
            and getattr(self, 'log_entries', None) is not None
        ):
            with open(self.log_file, 'w') as f:
                json.dump(self.log_entries, f, indent=2)
            self.log_entries = []
    
    def _alignment_log_fields(self) -> Dict[str, Any]:
        fields: Dict[str, Any] = {
            'session_id': self.session_id,
            'host_mono_sec': (
                time.perf_counter() - self.session_mono_t0
                if self.session_mono_t0 is not None
                else None
            ),
            'cumulative_pcm_samples': (
                self.audio_subsystem.cumulative_pcm_samples
                if self.audio_subsystem is not None
                else None
            ),
            'ble_packet_seq': self.session_packet_count,
        }
        return fields

    def log_data(self, entry: Dict[str, Any]):
        """Log data entry to file"""
        if not self.logging_enabled:
            return

        row = {**self._alignment_log_fields(), **entry}

        if self.csv_writer:
            # CSV format
            values_str = json.dumps(row.get('values', []))
            self.csv_writer.writerow([
                row.get('timestamp', ''),
                row.get('type', ''),
                row.get('path', ''),
                row.get('packet_size', 0),
                row.get('has_imu', False),
                row.get('battery_soc', 0.0),
                row.get('gap_seconds', 0.0),
                row.get('sample_rate', 0.0),
                row.get('value_count', 0),
                values_str,
                row.get('session_id', ''),
                row.get('host_mono_sec', ''),
                row.get('cumulative_pcm_samples', ''),
                row.get('ble_packet_seq', ''),
            ])
        elif hasattr(self, 'log_entries'):
            # JSON format
            self.log_entries.append(row)

    def _tk_dispatch(self, fn) -> None:
        """Run ``fn`` on the Tk main thread. Required for all widget updates from the BLE asyncio thread."""
        try:
            self.root.after(0, fn)
        except (tk.TclError, RuntimeError):
            pass

    def _apply_disconnected_ui(self) -> None:
        self.status_var.set("Disconnected")
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.scan_btn.config(state=tk.NORMAL)

    async def _disconnect_ble_transport(self) -> None:
        """Stop audio ingress, tear down BLE notify/session (runs on asyncio thread)."""
        self._audio_pipeline_accepting = False
        self._stop_audio_subsystem()
        cl = self.client
        if cl is not None:
            try:
                if cl.is_connected:
                    await cl.stop_notify(NUS_UUID)
            except Exception:
                pass
            try:
                if cl.is_connected:
                    await cl.disconnect()
            except Exception:
                pass
        self.connected = False
        self.client = None
        self.session_wall_start = None
        self.session_packet_count = 0
        self.session_id = None
        self.session_mono_t0 = None

    def on_closing(self) -> None:
        """Clean BLE + asyncio before destroying Tk (avoids CoreBluetooth callbacks after teardown)."""
        if self._window_close_handled:
            return
        self._window_close_handled = True
        self._shutting_down = True
        if self.loop and self.loop_thread and self.loop_thread.is_alive():
            fut = asyncio.run_coroutine_threadsafe(self._disconnect_ble_transport(), self.loop)
            try:
                fut.result(timeout=12.0)
            except Exception as exc:
                print(f"[Quit] BLE teardown: {exc}")
            try:
                self.loop.call_soon_threadsafe(self.loop.stop)
            except Exception:
                pass
            self.loop_thread.join(timeout=6.0)
        else:
            self._audio_pipeline_accepting = False
            self._stop_audio_subsystem()
            self.connected = False
            self.client = None
        try:
            self._ble_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self._ble_executor.shutdown(wait=False)
        self._apply_disconnected_ui()
        try:
            self.stop_logging()
        except Exception:
            pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def _emergency_shutdown(self) -> None:
        """If mainloop exits without ``on_closing`` (rare), stop BLE thread and files."""
        self._window_close_handled = True
        self._shutting_down = True
        self._audio_pipeline_accepting = False
        self._stop_audio_subsystem()
        if self.loop and self.loop_thread and self.loop_thread.is_alive():
            try:
                fut = asyncio.run_coroutine_threadsafe(self._disconnect_ble_transport(), self.loop)
                fut.result(timeout=5.0)
            except Exception:
                pass
            try:
                self.loop.call_soon_threadsafe(self.loop.stop)
            except Exception:
                pass
            self.loop_thread.join(timeout=4.0)
        try:
            self._ble_executor.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self._ble_executor.shutdown(wait=False)
        try:
            self.stop_logging()
        except Exception:
            pass

    def start_scan(self):
        """Start BLE device scan"""
        self.scan_btn.config(state=tk.DISABLED)
        self.status_var.set("Scanning...")
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.scan_devices(), self.loop)
        else:
            self.status_var.set("Event loop not ready")
            self.scan_btn.config(state=tk.NORMAL)
    
    async def scan_devices(self):
        """Scan for MetaBoard devices"""
        try:
            devices = await BleakScanner.discover(timeout=5.0)
            # Filter devices, handling None names safely
            metaboard_devices = []
            for d in devices:
                if d.name:  # Check if name is not None
                    name_lower = d.name.lower()
                    if "metabow" in name_lower or "metaboard" in name_lower:
                        metaboard_devices.append(d)
            
            if metaboard_devices:
                self.device_address = metaboard_devices[0].address
                device_name = metaboard_devices[0].name or "Unknown"
                addr = self.device_address

                def _found() -> None:
                    self.device_var.set(f"{device_name} ({addr})")
                    self.connect_btn.config(state=tk.NORMAL)
                    self.status_var.set(f"Found: {device_name}")

                self._tk_dispatch(_found)
            else:

                def _none() -> None:
                    self.status_var.set("No MetaBoard devices found")
                    messagebox.showinfo("Scan Complete", "No MetaBoard devices found")

                self._tk_dispatch(_none)
        except Exception as e:
            err = str(e)

            def _err() -> None:
                self.status_var.set(f"Scan error: {err}")
                messagebox.showerror("Scan Error", err)

            self._tk_dispatch(_err)
        finally:
            self._tk_dispatch(lambda: self.scan_btn.config(state=tk.NORMAL))
    
    def start_connect(self):
        """Start BLE connection"""
        if not self.device_address:
            messagebox.showerror("Error", "No device selected")
            return
        
        self.connect_btn.config(state=tk.DISABLED)
        self.status_var.set("Connecting...")
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.connect_device(), self.loop)
        else:
            self.status_var.set("Event loop not ready")
            self.connect_btn.config(state=tk.NORMAL)
    
    async def connect_device(self):
        """Connect to MetaBoard device"""
        try:
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            
            if self.client.is_connected:
                # Reset ADPCM decoder to match firmware encoder state
                self.adpcm_decoder.reset()
                self.session_packet_count = 0
                self.session_id = uuid.uuid4().hex[:12]
                self.session_mono_t0 = time.perf_counter()
                self.session_wall_start = time.time()
                self.connected = True
                self._start_audio_subsystem()
                self._audio_pipeline_accepting = True
                await self.client.start_notify(NUS_UUID, self.handle_notification)

                def _ok() -> None:
                    self.status_var.set("Connected")
                    self.connect_btn.config(state=tk.DISABLED)
                    self.disconnect_btn.config(state=tk.NORMAL)
                    self.scan_btn.config(state=tk.DISABLED)

                self._tk_dispatch(_ok)
            else:
                self._tk_dispatch(lambda: self.status_var.set("Connection failed"))
        except Exception as e:
            self._shutting_down = False
            self._audio_pipeline_accepting = False
            self._stop_audio_subsystem()
            self.connected = False
            cl = self.client
            if cl is not None:
                try:
                    if cl.is_connected:
                        await cl.disconnect()
                except Exception:
                    pass
                self.client = None
            err = str(e)

            def _fail() -> None:
                self.status_var.set(f"Connection error: {err}")
                messagebox.showerror("Connection Error", err)
                self.connect_btn.config(state=tk.NORMAL)

            self._tk_dispatch(_fail)
    
    def start_disconnect(self):
        """Start BLE disconnection"""
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.disconnect_device(), self.loop)
        else:
            self.status_var.set("Event loop not ready")
    
    def _start_audio_subsystem(self) -> None:
        """Spin up metabow.audio on BLE connect.  Feature extraction is always on;
        recording and VB-Cable depend on UI checkboxes."""
        self._stop_audio_subsystem()

        rec_dir = os.path.expanduser("~/Documents/MetaBow_Data")
        cfg = AudioSubsystemConfig(
            enable_feature_extraction=True,
            enable_recording=self.audio_var_record.get(),
            enable_virtual_c_output=self.audio_var_vb.get(),
            session_id=self.session_id,
            session_clock_t0_wall=self.session_wall_start,
            session_clock_t0_mono=self.session_mono_t0,
            recording_directory=rec_dir,
            on_feature_frame=self._on_ml_feature_frame,
        )
        try:
            self.audio_subsystem = AudioSubsystem(cfg)
        except ImportError as exc:
            # librosa not installed -- fall back to recording / VB-Cable only
            self.audio_subsystem = AudioSubsystem(
                AudioSubsystemConfig(
                    enable_feature_extraction=False,
                    enable_recording=self.audio_var_record.get(),
                    enable_virtual_c_output=self.audio_var_vb.get(),
                    session_id=self.session_id,
                    session_clock_t0_wall=self.session_wall_start,
                    session_clock_t0_mono=self.session_mono_t0,
                    recording_directory=rec_dir,
                )
            )
            self.root.after(
                0,
                lambda: messagebox.showwarning(
                    "ML features unavailable",
                    f"librosa / feature module import failed:\n{exc}\n\n"
                    "Continuing with recording and/or VB-Cable only.",
                ),
            )
        if self.audio_var_vb.get() and self.audio_subsystem.virtual_cable is not None:
            if not self.audio_subsystem.virtual_cable.enabled:
                self.root.after(
                    0,
                    lambda: messagebox.showwarning(
                        "VB-Cable",
                        "Stream was requested but the virtual output could not be opened.\n"
                        "Install VB-Audio Cable and ensure sounddevice can see it.",
                    ),
                )

    def _stop_audio_subsystem(self) -> None:
        if self.audio_subsystem is not None:
            try:
                self.audio_subsystem.stop()
            except Exception:
                pass
            self.audio_subsystem = None

    def _on_ml_feature_frame(
        self,
        frame_idx: int,
        vals: Dict[str, Any],
        t_wall: float,
        t_mono: float,
    ) -> None:
        """Send librosa feature vectors over OSC and log them when logging is enabled."""
        if not vals:
            return

        # --- OSC broadcast ---
        with self._osc_lock:
            for name, val in vals.items():
                if val is None:
                    continue
                path = f"/metabow/audio/{name}"
                try:
                    if isinstance(val, (list, tuple)):
                        self.osc_client.send_message(path, [float(x) for x in val])
                    else:
                        self.osc_client.send_message(path, float(val))
                    if name == "rms":
                        self.osc_client.send_message("/metabow/audio/rms_energy", float(val))
                except (TypeError, ValueError) as exc:
                    print(f"[OSC feature] {path}: {exc}")
            try:
                self.osc_client.send_message("/metabow/audio/ml_frame_index", float(frame_idx))
                self.osc_client.send_message("/metabow/audio/ml_host_mono_sec", float(t_mono))
            except Exception:
                pass

        # --- Log to CSV/JSON so ML consultant gets features in the export ---
        if self.logging_enabled:
            for name, val in vals.items():
                if val is None:
                    continue
                if isinstance(val, (list, tuple)):
                    values_list = [float(x) for x in val]
                else:
                    values_list = [float(val)]
                self.log_data({
                    'timestamp': t_wall,
                    'type': 'ml_feature',
                    'path': f'/metabow/audio/{name}',
                    'packet_size': 0,
                    'has_imu': False,
                    'battery_soc': 0.0,
                    'gap_seconds': 0.0,
                    'sample_rate': 0.0,
                    'value_count': len(values_list),
                    'values': values_list,
                })

    async def disconnect_device(self):
        """Disconnect from device"""
        self._shutting_down = True
        try:
            await self._disconnect_ble_transport()
            self._tk_dispatch(self._apply_disconnected_ui)
        except Exception as e:
            err = str(e)
            self._tk_dispatch(lambda: self.status_var.set(f"Disconnect error: {err}"))
        finally:
            self._shutting_down = False

    def handle_notification(self, sender, data: bytearray):
        """Handle incoming BLE notification (must return quickly — real work runs off the asyncio loop)."""
        if self._shutting_down:
            return
        if len(data) not in (PACKET_SIZE_OLD, PACKET_SIZE_NEW):
            return
        if not self.loop:
            return
        try:
            self.loop.create_task(self._ble_packet_task(bytes(data)))
        except RuntimeError:
            pass

    async def _ble_packet_task(self, data: bytes) -> None:
        try:
            await self.loop.run_in_executor(self._ble_executor, self._process_ble_packet_worker, data)
        except Exception as exc:
            print(f"[BLE] packet worker: {exc}")

    def _process_ble_packet_worker(self, data: bytes) -> None:
        """Decode BLE payload, update stats, OSC, and audio (runs on single-worker executor)."""
        with self._packet_state_lock:
            if self._shutting_down or not self.connected:
                return
            data = bytearray(data)

            self.session_packet_count += 1

            has_state_header = len(data) == PACKET_SIZE_NEW
            current_time = time.time()

            gap = 0.0
            if self.last_packet_time is not None:
                gap = current_time - self.last_packet_time

            if gap > 0:
                gap_ms = gap * 1000.0
                if gap_ms > PACKET_GAP_CRITICAL_MS:
                    self.detect_gap("packet", "/metabow/packet", gap, "critical")
                elif gap_ms > PACKET_GAP_WARNING_MS:
                    self.detect_gap("packet", "/metabow/packet", gap, "warning")

            self.last_packet_time = current_time

            packet_rate = self.packet_tracker.add_sample(current_time)

            if packet_rate > 0 and EXPECTED_PACKET_RATE > 0:
                rate_ratio = packet_rate / EXPECTED_PACKET_RATE
                if rate_ratio < (1.0 - RATE_DROP_CRITICAL):
                    self.detect_rate_drop(
                        "packet",
                        "/metabow/packet",
                        packet_rate,
                        EXPECTED_PACKET_RATE,
                        "critical",
                    )
                elif rate_ratio < (1.0 - RATE_DROP_THRESHOLD):
                    self.detect_rate_drop(
                        "packet",
                        "/metabow/packet",
                        packet_rate,
                        EXPECTED_PACKET_RATE,
                        "warning",
                    )

            if has_state_header:
                predicted_sample = struct.unpack("<h", data[0:2])[0]
                step_index = data[2]
                adpcm_data = data[3 : 3 + ADPCM_BLOCK_SIZE]
                imu_start = ADPCM_HEADER_SIZE + ADPCM_BLOCK_SIZE
                imu_data = data[imu_start : imu_start + IMU_DATA_SIZE]
                imu_flag = data[imu_start + IMU_DATA_SIZE]
                battery_start = imu_start + IMU_DATA_SIZE + IMU_FLAG_SIZE
                battery_bytes = data[battery_start : battery_start + BATTERY_DATA_SIZE]
            else:
                adpcm_data = data[0:ADPCM_BLOCK_SIZE]
                imu_data = data[ADPCM_BLOCK_SIZE : ADPCM_BLOCK_SIZE + IMU_DATA_SIZE]
                imu_flag = data[ADPCM_BLOCK_SIZE + IMU_DATA_SIZE]
                battery_bytes = data[
                    ADPCM_BLOCK_SIZE + IMU_DATA_SIZE + IMU_FLAG_SIZE :
                ]

            battery_soc = struct.unpack("<f", battery_bytes)[0]

            packet_stats = PacketStats(
                timestamp=current_time,
                packet_size=len(data),
                has_imu=(imu_flag == 1),
                battery_soc=battery_soc,
                gap_from_previous=gap,
            )
            self.packet_history.append(packet_stats)

            if self.logging_enabled:
                self.log_data(
                    {
                        "timestamp": current_time,
                        "type": "packet",
                        "path": "/metabow/packet",
                        "packet_size": len(data),
                        "has_imu": (imu_flag == 1),
                        "battery_soc": battery_soc,
                        "gap_seconds": gap,
                        "sample_rate": packet_rate,
                        "value_count": 0,
                        "values": [],
                    }
                )

            if imu_flag == 1:
                imu_floats = struct.unpack("<16f", imu_data)
                self.send_osc_message("/metabow/motion/quaternion", imu_floats[0:4], current_time)
                self.send_osc_message("/metabow/motion/acceleration", imu_floats[4:7], current_time)
                self.send_osc_message("/metabow/motion/gyroscope", imu_floats[7:10], current_time)
                self.send_osc_message("/metabow/motion/magnetometer", imu_floats[10:13], current_time)
                self.send_osc_message("/metabow/motion/raw_acceleration", imu_floats[13:16], current_time)
                self.send_osc_message("/metabow/motion", imu_floats, current_time)

            self.send_osc_message("/metabow/battery/percentage", [battery_soc], current_time)

            if adpcm_data and len(adpcm_data) == ADPCM_BLOCK_SIZE:
                pcm_int16: Optional[List[int]] = None
                try:
                    if has_state_header:
                        self.adpcm_decoder.set_state(predicted_sample, step_index)
                    pcm_int16 = self.adpcm_decoder.decode(adpcm_data)
                except Exception as e:
                    print(f"[ERROR] ADPCM decode failed: {e}")
                    pcm_int16 = None

                if pcm_int16 and len(pcm_int16) > 0:
                    audio_samples = [float(sample) / 32768.0 for sample in pcm_int16]
                    try:
                        self.send_osc_message("/metabow/audio", audio_samples, current_time)
                        if self._audio_pipeline_accepting and self.audio_subsystem is not None:
                            self.audio_subsystem.push_pcm_int16(
                                np.asarray(pcm_int16, dtype=np.int16),
                                adpcm_payload=bytes(adpcm_data),
                            )
                    except Exception as e:
                        if _is_portaudio_error(e):
                            print(f"[ERROR] VB-Cable / audio output: {e}")
                        else:
                            print(f"[ERROR] Audio pipeline failed (OSC/subsystem): {e}")

    def send_osc_message(self, path: str, values: List[float], timestamp: float):
        """Send OSC message and track statistics"""
        # Track route discovery
        if path not in self.routes:
            self.routes[path] = RouteInfo(
                path=path,
                first_seen=timestamp,
                last_seen=timestamp,
                message_count=0,
                data_type="float",
                sample_rate=0.0,
                last_value=values
            )
            self.osc_trackers[path] = SampleRateTracker()
        
        # Update route info
        route = self.routes[path]
        route.last_seen = timestamp
        route.message_count += 1
        route.last_value = values
        
        # Calculate gap from previous message to this route
        gap = 0.0
        if path in self.last_osc_times:
            gap = timestamp - self.last_osc_times[path]
        
        # Detect gap issues in real-time
        if gap > 0:
            gap_ms = gap * 1000.0
            if gap_ms > OSC_GAP_CRITICAL_MS:
                self.detect_gap("osc", path, gap, "critical")
            elif gap_ms > OSC_GAP_WARNING_MS:
                self.detect_gap("osc", path, gap, "warning")
        
        self.last_osc_times[path] = timestamp
        
        # Track OSC sample rate for this route
        osc_rate = self.osc_trackers[path].add_sample(timestamp)
        route.sample_rate = osc_rate
        
        # Detect rate drops in real-time
        if route.expected_rate == 0.0:
            # Initialize expected rate from current rate
            route.expected_rate = osc_rate if osc_rate > 0 else EXPECTED_PACKET_RATE
        else:
            # Update expected rate (moving average)
            route.expected_rate = (route.expected_rate * 0.9) + (osc_rate * 0.1)
        
        if osc_rate > 0 and route.expected_rate > 0:
            rate_ratio = osc_rate / route.expected_rate
            if rate_ratio < (1.0 - RATE_DROP_CRITICAL):
                self.detect_rate_drop("osc", path, osc_rate, route.expected_rate, "critical")
                route.rate_drop_warnings += 1
            elif rate_ratio < (1.0 - RATE_DROP_THRESHOLD):
                self.detect_rate_drop("osc", path, osc_rate, route.expected_rate, "warning")
                route.rate_drop_warnings += 1
        
        # Create OSC stats
        osc_stats = OSCMessageStats(
            timestamp=timestamp,
            path=path,
            value_count=len(values),
            gap_from_previous=gap
        )
        self.osc_history.append(osc_stats)
        
        # Send OSC message
        with self._osc_lock:
            if len(values) == 1:
                self.osc_client.send_message(path, values[0])
            else:
                self.osc_client.send_message(path, values)
        
        # Log OSC message
        if self.logging_enabled:
            self.log_data({
                'timestamp': timestamp,
                'type': 'osc',
                'path': path,
                'packet_size': 0,
                'has_imu': False,
                'battery_soc': 0.0,
                'gap_seconds': gap,
                'sample_rate': osc_rate,
                'value_count': len(values),
                'values': values
            })
    
    def detect_gap(self, alert_type: str, path: str, gap_seconds: float, severity: str):
        """Detect and log a gap alert in real-time"""
        alert = GapAlert(
            timestamp=time.time(),
            type=alert_type,
            path=path,
            gap_seconds=gap_seconds,
            severity=severity
        )
        self.gap_alerts.append(alert)
        
        # Update route gap warning count
        if path in self.routes:
            self.routes[path].gap_warnings += 1
        
        # Log to file if enabled
        if self.logging_enabled:
            self.log_data({
                'timestamp': alert.timestamp,
                'type': f'gap_{alert_type}',
                'path': path,
                'packet_size': 0,
                'has_imu': False,
                'battery_soc': 0.0,
                'gap_seconds': gap_seconds,
                'sample_rate': 0.0,
                'value_count': 0,
                'values': [severity]
            })
    
    def detect_rate_drop(self, alert_type: str, path: str, current_rate: float, 
                        expected_rate: float, severity: str):
        """Detect and log a rate drop alert in real-time"""
        drop_percent = ((expected_rate - current_rate) / expected_rate) * 100.0
        
        alert = RateDropAlert(
            timestamp=time.time(),
            type=alert_type,
            path=path,
            current_rate=current_rate,
            expected_rate=expected_rate,
            drop_percent=drop_percent
        )
        self.rate_drop_alerts.append(alert)
        
        # Update route rate drop warning count
        if path in self.routes:
            self.routes[path].rate_drop_warnings += 1
        
        # Log to file if enabled
        if self.logging_enabled:
            self.log_data({
                'timestamp': alert.timestamp,
                'type': f'rate_drop_{alert_type}',
                'path': path,
                'packet_size': 0,
                'has_imu': False,
                'battery_soc': 0.0,
                'gap_seconds': 0.0,
                'sample_rate': current_rate,
                'value_count': 0,
                'values': [expected_rate, drop_percent, severity]
            })
    
    def update_ui(self):
        """Update UI with current statistics"""
        # Update statistics text
        self.stats_text.delete(1.0, tk.END)
        
        with self._packet_state_lock:
            if self.connected:
                # Packet statistics
                packet_rate = self.packet_tracker.get_current_rate()
                avg_packet_rate = self.packet_tracker.get_average_rate()
                
                stats = f"""=== Packet Statistics ===
Current Rate: {packet_rate:.2f} packets/sec
Average Rate: {avg_packet_rate:.2f} packets/sec
Packets (rolling window, max {HISTORY_SIZE}): {len(self.packet_history)}
"""
                if self.session_wall_start is not None:
                    wall_s = time.time() - self.session_wall_start
                    if wall_s > 0.25:
                        expected = wall_s * EXPECTED_PACKET_RATE
                        pct = (
                            100.0 * self.session_packet_count / expected
                            if expected > 0
                            else 0.0
                        )
                        stats += (
                            f"Session total packets: {self.session_packet_count} "
                            f"(~{pct:.0f}% of nominal {EXPECTED_PACKET_RATE:.0f} Hz — "
                            f"aim high for clean recordings)\n"
                        )
                stats += "\n"
                if self.audio_subsystem is not None:
                    ast = self.audio_subsystem.stats
                    stats += f"""=== metabow.audio ===
Ingest pushes: {ast.push_count}
Mean ingest (fan-out): {ast.mean_time_ms:.3f} ms
Peak ingest: {ast.peak_time_ms:.3f} ms
"""
                    fs = self.audio_subsystem.feature_stage
                    if fs is not None:
                        ex = fs.extractor
                        stats += (
                            f"Feature frames: {ex.features_processed}  "
                            f"last librosa block: {ex.processing_time_ms:.2f} ms\n"
                        )
                    rec = self.audio_subsystem.recorder
                    if rec.recording and rec.wav_path:
                        stats += f"Recording WAV: {rec.wav_path}\n"
                        if rec.adpcm_path:
                            stats += f"Recording ADPCM: {rec.adpcm_path}\n"
                    stats += "\n"
                
                # OSC statistics
                if self.osc_trackers:
                    stats += "=== OSC Route Statistics ===\n"
                    for path, tracker in sorted(self.osc_trackers.items()):
                        route = self.routes[path]
                        rate = tracker.get_current_rate()
                        stats += f"{path}:\n"
                        stats += f"  Rate: {rate:.2f} msg/sec\n"
                        stats += f"  Count: {route.message_count}\n"
                        stats += f"  Last gap: {route.last_seen - route.first_seen:.2f}s\n\n"
                
                self.stats_text.insert(1.0, stats)
                
                # Update routes listbox with alert indicators
                self.routes_listbox.delete(0, tk.END)
                for path in sorted(self.routes.keys()):
                    route = self.routes[path]
                    status = f"{path} ({route.sample_rate:.1f} Hz)"
                    if route.gap_warnings > 0 or route.rate_drop_warnings > 0:
                        status += f" ⚠ ({route.gap_warnings}G/{route.rate_drop_warnings}R)"
                    self.routes_listbox.insert(tk.END, status)
                
                # Update alerts display
                self.update_alerts_display()
                
                # Update graphs
                self.update_graphs()
            
        # Schedule next update
        self.root.after(self.update_interval, self.update_ui)
    
    def update_alerts_display(self):
        """Update the alerts display with recent issues"""
        self.alerts_text.delete(1.0, tk.END)
        
        if not self.connected:
            return
        
        # Show recent gap alerts (last 5)
        recent_gaps = list(self.gap_alerts)[-5:]
        if recent_gaps:
            self.alerts_text.insert(tk.END, "=== Recent Gaps ===\n", "header")
            for alert in recent_gaps:
                gap_ms = alert.gap_seconds * 1000.0
                severity_tag = "CRITICAL" if alert.severity == "critical" else "WARNING"
                time_str = datetime.fromtimestamp(alert.timestamp).strftime("%H:%M:%S")
                self.alerts_text.insert(tk.END, 
                    f"[{time_str}] {severity_tag}: {alert.path}\n"
                    f"  Gap: {gap_ms:.1f}ms\n", 
                    "critical" if alert.severity == "critical" else "warning")
        
        # Show recent rate drop alerts (last 5)
        recent_drops = list(self.rate_drop_alerts)[-5:]
        if recent_drops:
            if recent_gaps:
                self.alerts_text.insert(tk.END, "\n")
            self.alerts_text.insert(tk.END, "=== Recent Rate Drops ===\n", "header")
            for alert in recent_drops:
                severity_tag = "CRITICAL" if alert.drop_percent > (RATE_DROP_CRITICAL * 100) else "WARNING"
                time_str = datetime.fromtimestamp(alert.timestamp).strftime("%H:%M:%S")
                self.alerts_text.insert(tk.END,
                    f"[{time_str}] {severity_tag}: {alert.path}\n"
                    f"  Rate: {alert.current_rate:.1f} Hz (expected: {alert.expected_rate:.1f} Hz)\n"
                    f"  Drop: {alert.drop_percent:.1f}%\n",
                    "critical" if alert.drop_percent > (RATE_DROP_CRITICAL * 100) else "warning")
        
        if not recent_gaps and not recent_drops:
            self.alerts_text.insert(tk.END, "No alerts - all systems normal", "normal")
        
        # Configure text tags for colors
        self.alerts_text.tag_config("critical", foreground="red", font=("Courier", 9, "bold"))
        self.alerts_text.tag_config("warning", foreground="orange", font=("Courier", 9))
        self.alerts_text.tag_config("normal", foreground="green", font=("Courier", 9))
        self.alerts_text.tag_config("header", font=("Courier", 9, "bold"))
    
    def update_graphs(self):
        """Update matplotlib graphs"""
        self.ax1.clear()
        self.ax2.clear()
        
        # Graph 1: Incoming packet rate over time
        if self.packet_tracker.rates:
            times = np.arange(len(self.packet_tracker.rates))
            self.ax1.plot(times, list(self.packet_tracker.rates), 'b-', linewidth=1)
            self.ax1.set_title("Incoming BLE Packet Rate")
            self.ax1.set_xlabel("Sample #")
            self.ax1.set_ylabel("Packets/sec")
            self.ax1.grid(True, alpha=0.3)
            self.ax1.set_ylim(0, max(self.packet_tracker.rates) * 1.1 if self.packet_tracker.rates else 200)
        
        # Graph 2: Outgoing OSC message rates (top routes)
        if self.osc_trackers:
            # Get top 5 routes by message count
            top_routes = sorted(self.routes.items(), key=lambda x: x[1].message_count, reverse=True)[:5]
            
            for path, route in top_routes:
                tracker = self.osc_trackers[path]
                if tracker.rates:
                    times = np.arange(len(tracker.rates))
                    self.ax2.plot(times, list(tracker.rates), label=path, linewidth=1)
            
            self.ax2.set_title("Outgoing OSC Message Rates (Top 5 Routes)")
            self.ax2.set_xlabel("Sample #")
            self.ax2.set_ylabel("Messages/sec")
            _handles, labels = self.ax2.get_legend_handles_labels()
            if labels:
                self.ax2.legend(loc='upper right', fontsize=8)
            self.ax2.grid(True, alpha=0.3)
        
        self.fig.tight_layout()
        self.canvas.draw()
    
    def run(self):
        """Start the application"""
        try:
            self.root.mainloop()
        finally:
            if not self._window_close_handled:
                self._emergency_shutdown()

# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    app = MetaBoardOSCMonitor()
    app.run()

