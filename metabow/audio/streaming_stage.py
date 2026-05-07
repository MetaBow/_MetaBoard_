"""Thread 3: host-side streaming to VB-Cable.

Producers (BLE worker via :meth:`push_float32`) only enqueue samples — they never
block on PortAudio or the resampler. A dedicated worker thread drains the queue,
runs a stateful streaming resampler, and writes into a lock-protected ring
buffer. PortAudio runs in callback mode and pulls from that ring buffer; on
underrun the callback emits silence rather than back-pressuring the producer.

This replaces the previous in-line ``stream.write`` path, which serialized
librosa resampling and a blocking PortAudio write on the single BLE executor and
caused BLE delivery to collapse to ~50%.
"""

from __future__ import annotations

import math
import os
import queue
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from metabow.audio.types import (
    METABOW_AUDIO_SAMPLE_RATE_HZ,
    VIRTUAL_CABLE_OUTPUT_HZ,
    AudioSubsystemConfig,
)


# soxr filter quality. "MQ" gives ~3 ms group delay and minimal pre-echo on
# transients, vs "HQ" which is audibly cleaner on broadband material but rings
# slightly ahead of sharp attacks (audible as a faint "tsk" before percussive
# sounds when monitoring live). Override with METABOW_VB_QUALITY=HQ if desired.
_DEFAULT_SOXR_QUALITY = os.environ.get("METABOW_VB_QUALITY", "MQ").upper()
# Linear fade applied at underrun boundaries to avoid hard zero-step clicks.
_UNDERRUN_FADE_LEN = 32  # samples @ output rate (~0.7 ms at 44.1 kHz)
# IMA ADPCM predictor wanders across BLE drops, leaving large DC offsets on
# the streamed signal (~+19% FS in field tests). Mirrors the corner used by
# the recording path's DcBlock — well below musical content.
_DC_BLOCK_CORNER_HZ = 15.0


class _DcBlockerFloat32:
    """Streaming 1st-order DC block: y[n] = x[n] - x[n-1] + R * y[n-1].

    Vectorized through ``scipy.signal.lfilter`` with persistent state; falls
    back to a pure-Python sample loop only if scipy is not installed (numpy
    alone cannot run a stateful IIR in C).
    """

    __slots__ = ("_R", "_b", "_a", "_zi", "_signal", "_xp", "_yp")

    def __init__(self, sample_rate: int, corner_hz: float) -> None:
        self._R = math.exp(-2.0 * math.pi * corner_hz / float(sample_rate))
        self._b = np.array([1.0, -1.0], dtype=np.float64)
        self._a = np.array([1.0, -self._R], dtype=np.float64)
        self._signal: Any = None
        try:
            from scipy import signal  # type: ignore
            self._signal = signal
            self._zi = np.zeros(1, dtype=np.float64)
        except ImportError:
            self._zi = None
            self._xp = 0.0
            self._yp = 0.0

    def process(self, x: np.ndarray) -> np.ndarray:
        if x.size == 0:
            return x
        if self._signal is not None:
            y, self._zi = self._signal.lfilter(
                self._b, self._a, x.astype(np.float64, copy=False), zi=self._zi
            )
            return y.astype(np.float32, copy=False)
        # scipy-less fallback: small Python loop (per-BLE-chunk = ~90 samples)
        out = np.empty_like(x, dtype=np.float32)
        R = self._R
        xp = self._xp
        yp = self._yp
        for i in range(x.size):
            xi = float(x[i])
            yi = xi - xp + R * yp
            out[i] = yi
            xp = xi
            yp = yi
        self._xp = xp
        self._yp = yp
        return out


def _import_sounddevice() -> Any:
    """Import PortAudio binding only when streaming is used (avoids slow init on import)."""
    try:
        import sounddevice as sd

        return sd
    except ImportError:  # pragma: no cover
        return None


# ---------------------------------------------------------------------------
# Resampler backends
# ---------------------------------------------------------------------------

_ResampleFn = Callable[[np.ndarray, bool], np.ndarray]


def _build_soxr_stream(orig_sr: int, target_sr: int) -> Optional[Tuple[_ResampleFn, str]]:
    try:
        import soxr  # type: ignore
    except ImportError:
        return None
    quality = _DEFAULT_SOXR_QUALITY if _DEFAULT_SOXR_QUALITY in {"QQ", "LQ", "MQ", "HQ", "VHQ"} else "MQ"
    rs = soxr.ResampleStream(
        orig_sr,
        target_sr,
        num_channels=1,
        dtype="float32",
        quality=quality,
    )

    def _f(x: np.ndarray, last: bool = False) -> np.ndarray:
        y = rs.resample_chunk(x, last=last)
        return np.asarray(y, dtype=np.float32).reshape(-1)

    return _f, f"soxr/{quality}"


def _build_samplerate_stream(orig_sr: int, target_sr: int) -> Optional[Tuple[_ResampleFn, str]]:
    try:
        import samplerate  # type: ignore
    except ImportError:
        return None
    ratio = float(target_sr) / float(orig_sr)
    rs = samplerate.Resampler("sinc_fastest", channels=1)

    def _f(x: np.ndarray, last: bool = False) -> np.ndarray:
        y = rs.process(x.astype(np.float32, copy=False), ratio, end_of_input=last)
        return np.asarray(y, dtype=np.float32).reshape(-1)

    return _f, "samplerate/sinc_fastest"


def _build_scipy_block(orig_sr: int, target_sr: int) -> Optional[Tuple[_ResampleFn, str]]:
    try:
        from math import gcd
        from scipy import signal  # type: ignore
    except ImportError:
        return None
    g = gcd(int(orig_sr), int(target_sr))
    up = target_sr // g
    down = orig_sr // g

    def _f(x: np.ndarray, last: bool = False) -> np.ndarray:
        if x.size == 0:
            return np.empty(0, dtype=np.float32)
        return signal.resample_poly(x, up, down).astype(np.float32, copy=False)

    return _f, "scipy/resample_poly (stateless)"


def _build_linear(orig_sr: int, target_sr: int) -> Tuple[_ResampleFn, str]:
    def _f(x: np.ndarray, last: bool = False) -> np.ndarray:
        if x.size == 0:
            return np.empty(0, dtype=np.float32)
        n_out = max(1, int(round(x.size * target_sr / orig_sr)))
        t_in = np.linspace(0.0, 1.0, num=x.size, endpoint=False)
        t_out = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
        return np.interp(t_out, t_in, x).astype(np.float32, copy=False)

    return _f, "linear (stateless, last resort)"


def _make_stream_resampler(orig_sr: int, target_sr: int) -> Tuple[_ResampleFn, str, bool]:
    """Pick the best available resampler.

    Returns ``(callable, backend_name, is_stateful)``. Stateful backends preserve
    filter state across calls so feeding tiny BLE-sized chunks is fine; stateless
    backends need accumulation to avoid chunk-boundary artefacts.
    """
    if orig_sr == target_sr:
        def _pass(x: np.ndarray, last: bool = False) -> np.ndarray:
            return np.asarray(x, dtype=np.float32).reshape(-1)
        return _pass, "passthrough", True
    for builder in (_build_soxr_stream, _build_samplerate_stream):
        built = builder(orig_sr, target_sr)
        if built is not None:
            fn, name = built
            return fn, name, True
    built = _build_scipy_block(orig_sr, target_sr)
    if built is not None:
        fn, name = built
        return fn, name, False
    fn, name = _build_linear(orig_sr, target_sr)
    return fn, name, False


# ---------------------------------------------------------------------------
# Lock-protected float32 ring buffer (single producer / single PA-callback consumer)
# ---------------------------------------------------------------------------


class _RingBuffer:
    """Drop-oldest ring buffer. Producer never blocks; PA callback never allocates."""

    def __init__(self, capacity: int):
        self._buf = np.zeros(int(capacity), dtype=np.float32)
        self._cap = int(capacity)
        self._w = 0
        self._r = 0
        self._size = 0
        self._lock = threading.Lock()

    @property
    def capacity(self) -> int:
        return self._cap

    def fill_level(self) -> int:
        with self._lock:
            return self._size

    def write(self, data: np.ndarray) -> int:
        n = int(data.size)
        if n == 0:
            return 0
        if n >= self._cap:
            data = data[-self._cap:]
            n = data.size
        with self._lock:
            free = self._cap - self._size
            if n > free:
                drop = n - free
                self._r = (self._r + drop) % self._cap
                self._size -= drop
            end = self._w + n
            if end <= self._cap:
                self._buf[self._w:end] = data
            else:
                first = self._cap - self._w
                self._buf[self._w:] = data[:first]
                self._buf[: n - first] = data[first:]
            self._w = end % self._cap
            self._size += n
            return n

    def read_into(self, out: np.ndarray, n: int) -> int:
        with self._lock:
            available = min(self._size, n)
            if available == 0:
                return 0
            end = self._r + available
            if end <= self._cap:
                out[:available] = self._buf[self._r:end]
            else:
                first = self._cap - self._r
                out[:first] = self._buf[self._r:]
                out[first:available] = self._buf[: available - first]
            self._r = end % self._cap
            self._size -= available
            return available


# ---------------------------------------------------------------------------
# Device probe (unchanged contract)
# ---------------------------------------------------------------------------


class VirtualCableDeviceProbe:
    """Detect VB-Audio Cable output device (same heuristic as mega-script)."""

    @staticmethod
    def query() -> Dict[str, Any]:
        sd = _import_sounddevice()
        if sd is None:
            return {"success": False, "error": "sounddevice not installed"}
        try:
            devices = sd.query_devices()
            for i, device in enumerate(devices):
                name = device.get("name", "")
                if "VB-Cable" in name:
                    return {
                        "success": True,
                        "device_name": name,
                        "device_index": i,
                        "channels": device.get("max_output_channels", 0),
                        "sample_rate": device.get("default_samplerate"),
                    }
            return {
                "success": False,
                "error": "VB-Cable not found",
                "instructions": "Install from https://vb-audio.com/Cable/",
            }
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "error": str(exc)}


# ---------------------------------------------------------------------------
# Streaming stage
# ---------------------------------------------------------------------------


class VirtualCableStreamStage:
    """16 kHz mono float → stateful resample → 44.1 kHz callback-mode OutputStream.

    The BLE worker thread only calls :meth:`push_float32`, which enqueues and
    returns. A dedicated worker thread does the resampling and ring-buffer fill.
    PortAudio's own RT thread pulls from the ring via the callback.
    """

    # Input queue: bounded so a hung worker can't grow memory unbounded. BLE pushes
    # ~177 chunks/sec (~90 samples each); 128 entries ≈ 720 ms of slack.
    _INPUT_QUEUE_MAX = 128

    # Accumulate this many input samples before resampling (helps the stateless
    # fallbacks; stateful backends are unaffected by chunk size).
    _ACCUM_TARGET_SAMPLES = 512  # 32 ms @ 16 kHz

    def __init__(self, config: AudioSubsystemConfig):
        self._sd = _import_sounddevice()
        if self._sd is None:
            raise ImportError("streaming requires sounddevice; pip install sounddevice")
        self._cfg = config

        self._stream: Any = None
        self._enabled = False

        self._in_q: "queue.Queue[Optional[np.ndarray]]" = queue.Queue(
            maxsize=self._INPUT_QUEUE_MAX
        )
        self._ring: Optional[_RingBuffer] = None
        self._worker: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # Diagnostics (read by callers / monitor UI if desired)
        self.dropped_input_chunks: int = 0
        self.callback_underruns: int = 0
        self.backend_name: str = "(none)"

        # Pre-allocated fade ramp used by the PA callback at underrun boundaries
        # (smoothly steps from the last-played sample down to zero rather than
        # hard-cutting, which prevents the click most of the residual artefacts
        # are coming from).
        self._fade_ramp = np.linspace(
            1.0, 0.0, _UNDERRUN_FADE_LEN, endpoint=False, dtype=np.float32
        )
        self._last_played_sample: float = 0.0

        self._last_log_t = 0.0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def probe(self) -> Dict[str, Any]:
        return VirtualCableDeviceProbe.query()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        info = VirtualCableDeviceProbe.query()
        if not info.get("success"):
            return False

        target_sr = VIRTUAL_CABLE_OUTPUT_HZ
        # ~250 ms at target rate: enough to absorb BLE jitter, short enough to keep
        # DAW monitoring latency in the neighbourhood of 200 ms total.
        ring_capacity = max(int(target_sr * 0.25), 4096)
        self._ring = _RingBuffer(ring_capacity)

        self._stop_evt.clear()
        # Drain any sentinel left over from a previous stop().
        try:
            while True:
                self._in_q.get_nowait()
        except queue.Empty:
            pass

        try:
            self._stream = self._sd.OutputStream(
                device=info["device_index"],
                channels=1,
                samplerate=target_sr,
                dtype=np.float32,
                blocksize=self._cfg.virtual_cable_blocksize,
                latency="low",
                callback=self._pa_callback,
            )
            self._stream.start()
        except Exception as exc:
            print(f"[VB-Cable] OutputStream open failed: {exc}")
            self._stream = None
            self._ring = None
            return False

        self._worker = threading.Thread(
            target=self._resampler_loop,
            name="vbcable_resampler",
            daemon=True,
        )
        self._worker.start()
        self._enabled = True
        return True

    def stop(self) -> None:
        self._enabled = False
        self._stop_evt.set()
        try:
            self._in_q.put_nowait(None)
        except queue.Full:
            pass

        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.join(timeout=1.5)

        stream = self._stream
        self._stream = None
        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass

        self._ring = None
        try:
            while True:
                self._in_q.get_nowait()
        except queue.Empty:
            pass

    # ------------------------------------------------------------------
    # Producer side (called from BLE executor thread)
    # ------------------------------------------------------------------

    def push_float32(self, mono: np.ndarray) -> None:
        if not self._enabled or mono.size == 0:
            return
        # Copy: the BLE worker's mono_f buffer is reused downstream and we'll touch it
        # later from the resampler thread.
        x = np.clip(np.asarray(mono, dtype=np.float32).reshape(-1), -1.0, 1.0).copy()
        try:
            self._in_q.put_nowait(x)
        except queue.Full:
            self.dropped_input_chunks += 1
            self._log_throttled(
                f"[VB-Cable] input queue full; dropped chunk "
                f"(total={self.dropped_input_chunks})"
            )

    # ------------------------------------------------------------------
    # Resampler thread
    # ------------------------------------------------------------------

    def _resampler_loop(self) -> None:
        cfg = self._cfg
        orig_sr = cfg.sample_rate_hz or METABOW_AUDIO_SAMPLE_RATE_HZ
        target_sr = VIRTUAL_CABLE_OUTPUT_HZ
        try:
            resample, backend_name, is_stateful = _make_stream_resampler(orig_sr, target_sr)
        except Exception as exc:
            print(f"[VB-Cable] resampler init failed: {exc}; stream disabled")
            self._enabled = False
            return
        dc_block = _DcBlockerFloat32(orig_sr, _DC_BLOCK_CORNER_HZ)
        self.backend_name = backend_name
        # Best-effort floor for end-to-end latency: PortAudio block + resampler
        # group delay + ring buffer fill. Real DAW latency (Reaper input buffer,
        # VB-Cable internal latency) sits on top of this and dominates.
        block_ms = 1000.0 * self._cfg.virtual_cable_blocksize / target_sr
        try:
            stream_latency_ms = 1000.0 * float(self._stream.latency)
        except Exception:
            stream_latency_ms = float("nan")
        print(
            f"[VB-Cable] streaming via {backend_name} ({orig_sr}→{target_sr} Hz); "
            f"block {block_ms:.1f} ms, PA-reported latency {stream_latency_ms:.1f} ms"
        )

        # Stateless backends benefit from larger blocks; stateful backends can run
        # per-BLE-chunk without quality loss.
        accum_target = self._ACCUM_TARGET_SAMPLES if not is_stateful else 0

        accum: list[np.ndarray] = []
        accum_size = 0

        while not self._stop_evt.is_set():
            try:
                chunk = self._in_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if chunk is None:
                break

            if accum_target == 0:
                self._resample_and_push(resample, dc_block.process(chunk), last=False)
                continue

            accum.append(chunk)
            accum_size += chunk.size
            if accum_size < accum_target:
                continue
            x = np.concatenate(accum) if len(accum) > 1 else accum[0]
            accum.clear()
            accum_size = 0
            self._resample_and_push(resample, dc_block.process(x), last=False)

        # Flush whatever's left so the DAW doesn't lose the trailing audio.
        if accum:
            x = np.concatenate(accum) if len(accum) > 1 else accum[0]
            self._resample_and_push(resample, dc_block.process(x), last=True)
        else:
            try:
                tail = resample(np.empty(0, dtype=np.float32), True)
                if tail.size:
                    ring = self._ring
                    if ring is not None:
                        ring.write(tail)
            except Exception:
                pass

    def _resample_and_push(self, resample: _ResampleFn, x: np.ndarray, last: bool) -> None:
        try:
            y = resample(x, last)
        except Exception as exc:
            self._log_throttled(f"[VB-Cable] resample failed: {exc}")
            return
        if y.size == 0:
            return
        ring = self._ring
        if ring is not None:
            ring.write(y)

    # ------------------------------------------------------------------
    # PortAudio callback (real-time thread; no allocations, no logging)
    # ------------------------------------------------------------------

    def _pa_callback(self, outdata, frames, time_info, status) -> None:  # noqa: D401
        ring = self._ring
        if ring is None:
            outdata.fill(0.0)
            return
        n = ring.read_into(outdata[:, 0], frames)
        if n == frames:
            self._last_played_sample = float(outdata[-1, 0])
            return
        # Underrun: linearly fade from the last good sample down to zero over a
        # short tail, then hold zero. Hard zero-stepping is what produces the
        # audible click at every BLE jitter gap.
        self.callback_underruns += 1
        last = float(outdata[n - 1, 0]) if n > 0 else self._last_played_sample
        fade = min(frames - n, self._fade_ramp.size)
        if fade > 0:
            outdata[n : n + fade, 0] = last * self._fade_ramp[:fade]
        if n + fade < frames:
            outdata[n + fade :, 0] = 0.0
        self._last_played_sample = 0.0

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def _log_throttled(self, msg: str) -> None:
        now = time.perf_counter()
        if now - self._last_log_t >= 0.5:
            print(msg)
            self._last_log_t = now
