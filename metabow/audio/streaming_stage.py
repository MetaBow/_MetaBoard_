"""Thread 3: host-side streaming to VB-Cable (sounddevice), ported from PARALLELIZED ``AudioRecorder``."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

import numpy as np

from metabow.audio.types import (
    METABOW_AUDIO_SAMPLE_RATE_HZ,
    VIRTUAL_CABLE_OUTPUT_HZ,
    AudioSubsystemConfig,
)


def _import_sounddevice() -> Any:
    """Import PortAudio binding only when streaming is used (avoids slow init on import)."""
    try:
        import sounddevice as sd

        return sd
    except ImportError:  # pragma: no cover
        return None


def _linear_resample(mono_float: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    """Small-block resampler; keeps deps light. Optionally upgrade to librosa in-process."""
    x = np.asarray(mono_float, dtype=np.float64).reshape(-1)
    if orig_sr == target_sr or x.size == 0:
        return x.astype(np.float32)
    n_out = max(1, int(round(x.size * target_sr / orig_sr)))
    t_in = np.linspace(0.0, 1.0, num=x.size, endpoint=False)
    t_out = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
    return np.interp(t_out, t_in, x).astype(np.float32)


def _librosa_resample(mono_float: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    try:
        import librosa
    except ImportError:
        raise  # caller falls back to linear
    return librosa.resample(
        mono_float.astype(np.float32),
        orig_sr=orig_sr,
        target_sr=target_sr,
        res_type="kaiser_fast",
    ).astype(np.float32)


def _scipy_resample_poly(mono_float: np.ndarray, orig_sr: int, target_sr: int) -> np.ndarray:
    """Band-limited resampling without librosa (polyphase)."""
    from math import gcd

    from scipy import signal

    x = np.asarray(mono_float, dtype=np.float32).reshape(-1)
    if orig_sr == target_sr or x.size == 0:
        return x
    g = gcd(int(orig_sr), int(target_sr))
    up = target_sr // g
    down = orig_sr // g
    return signal.resample_poly(x, up, down).astype(np.float32)


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


class VirtualCableStreamStage:
    """
    16 kHz mono float → resample → 44.1 kHz ``sounddevice.OutputStream`` (VB-Cable input in OS).

    Enable/disable matches the UX of the research app without coupling to Tk.
    """

    def __init__(self, config: AudioSubsystemConfig):
        self._sd = _import_sounddevice()
        if self._sd is None:
            raise ImportError("streaming requires sounddevice; pip install sounddevice")
        self._cfg = config
        self._stream: Any = None
        self._enabled = False
        self._last_write_error_log = 0.0
        self._recovery_attempted_for_stream = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def probe(self) -> Dict[str, Any]:
        return VirtualCableDeviceProbe.query()

    def start(self) -> bool:
        info = VirtualCableDeviceProbe.query()
        if not info.get("success"):
            return False
        try:
            self._stream = self._sd.OutputStream(
                device=info["device_index"],
                channels=1,
                samplerate=VIRTUAL_CABLE_OUTPUT_HZ,
                dtype=np.float32,
                blocksize=self._cfg.virtual_cable_blocksize,
                latency="low",
            )
            self._stream.start()
            self._enabled = True
            self._recovery_attempted_for_stream = False
            return True
        except Exception:
            self._stream = None
            self._enabled = False
            return False

    def stop(self) -> None:
        if self._stream:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        self._enabled = False

    def _log_write_error_throttled(self, msg: str) -> None:
        now = time.perf_counter()
        if now - self._last_write_error_log >= 0.5:
            print(msg)
            self._last_write_error_log = now

    def _recover_stream_once(self) -> bool:
        if self._recovery_attempted_for_stream:
            return False
        self._recovery_attempted_for_stream = True
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        self._enabled = False
        return self.start()

    def push_float32(self, mono: np.ndarray) -> None:
        if not self._enabled or self._stream is None or mono.size == 0:
            return
        stream = self._stream
        if hasattr(stream, "active") and not stream.active:
            self._log_write_error_throttled("[VB-Cable] OutputStream inactive; skipping write")
            return
        x = np.clip(np.asarray(mono, dtype=np.float32).reshape(-1), -1.0, 1.0)
        orig_sr = self._cfg.sample_rate_hz or METABOW_AUDIO_SAMPLE_RATE_HZ
        try:
            y = _librosa_resample(x, orig_sr, VIRTUAL_CABLE_OUTPUT_HZ)
        except Exception:
            try:
                y = _scipy_resample_poly(x, orig_sr, VIRTUAL_CABLE_OUTPUT_HZ)
            except Exception:
                y = _linear_resample(x, orig_sr, VIRTUAL_CABLE_OUTPUT_HZ)
        if y.size == 0:
            return
        try:
            stream.write(y)
        except Exception as exc:
            self._log_write_error_throttled(f"[VB-Cable] write failed: {exc}")
            if self._recover_stream_once():
                try:
                    if self._stream is not None:
                        self._stream.write(y)
                except Exception as exc2:
                    self._log_write_error_throttled(f"[VB-Cable] write failed after reopen: {exc2}")
                    self.stop()
            else:
                self.stop()
