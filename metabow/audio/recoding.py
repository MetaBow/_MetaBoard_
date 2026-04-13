"""Decoded PCM / ADPCM sidecar recording and DC blocking."""

from __future__ import annotations

import json
import math
import os
import time
import wave
from typing import BinaryIO, Optional

import numpy as np

from metabow.audio.types import AudioSubsystemConfig


class _MonoDcBlock:
    """
    First-order high-pass (DC blocker): y[n] = x[n] - x[n-1] + R * y[n-1],
    R = exp(-2 pi fc / fs). Removes slow wander from ADPCM state drift / packet loss
    without affecting musical content much when fc is about 10-20 Hz at 16 kHz.
    """

    __slots__ = ("_R", "_xp", "_yp")

    def __init__(self, sample_rate: int, corner_hz: float) -> None:
        self._R = math.exp(-2.0 * math.pi * corner_hz / float(sample_rate))
        self._xp = 0.0
        self._yp = 0.0

    def reset(self) -> None:
        self._xp = 0.0
        self._yp = 0.0

    def process(self, pcm: np.ndarray) -> np.ndarray:
        x = pcm.astype(np.float64)
        R = self._R
        xp, yp = self._xp, self._yp
        n = x.size
        y = np.empty(n)
        for i in range(n):
            xi = x[i]
            yi = xi - xp + R * yp
            y[i] = yi
            xp, yp = xi, yi
        self._xp, self._yp = xp, yp
        return np.clip(np.rint(y), -32768, 32767).astype(np.int16)


class DecodedPcmRecorder:
    """
    WAV (decoded PCM) + optional raw ADPCM binary — same contract as PARALLELIZED ``AudioRecorder``
    without PyAudio or asyncio dependencies.

    Do not reset the ADPCM decoder when starting a take; decoder state must follow the stream.

    **DC block:** optional first-order high-pass to remove decoder drift on 114-byte
    (no ADPCM state header) streams.

    **Playback duration:** Each written chunk is only what BLE delivered. The WAV header
    uses the device sample rate (16 kHz). If the OS drops many notifications, ingest is
    slower than 16 kHz and players will run the file faster than wall-clock session time.
    Fixing that needs either a per-packet **sequence number** from firmware (to insert
    silence only for *actually* missing frames) or offline time-stretch in a DAW — not
    inferring drops from inter-arrival time, which matches steady ~77 Hz delivery as if
    every gap were a lost frame and floods the recording with silence.
    """

    def __init__(self, config: AudioSubsystemConfig, channels: int = 1, sample_width: int = 2):
        self._channels = channels
        self._sample_width = sample_width
        self._framerate = config.sample_rate_hz
        self._gain = float(config.record_output_gain)

        hz = float(config.record_dc_block_corner_hz)
        self._dc: Optional[_MonoDcBlock] = None
        if hz > 0.0:
            self._dc = _MonoDcBlock(config.sample_rate_hz, hz)

        self._recording = False
        self._wave: Optional[wave.Wave_write] = None
        self._adpcm: Optional[BinaryIO] = None
        self.wav_path: Optional[str] = None
        self.adpcm_path: Optional[str] = None
        self._session_id: Optional[str] = config.session_id

    @property
    def recording(self) -> bool:
        return self._recording

    def start(self, directory: Optional[str] = None) -> Optional[str]:
        if directory is None:
            directory = os.path.expanduser("~/Documents/MetaBow_Data")
        os.makedirs(directory, exist_ok=True)
        ts = int(time.time())
        prefix = f"{self._session_id}_" if self._session_id else ""
        self.adpcm_path = os.path.join(directory, f"{prefix}adpcm_audio_{ts}.bin")
        self.wav_path = os.path.join(directory, f"{prefix}decoded_audio_{ts}.wav")
        manifest = {
            "session_id": self._session_id,
            "sample_rate_hz": self._framerate,
            "started_wall_unix": time.time(),
            "wav_basename": os.path.basename(self.wav_path) if self.wav_path else None,
            "adpcm_basename": os.path.basename(self.adpcm_path) if self.adpcm_path else None,
        }
        man_path = os.path.join(directory, f"{prefix}session_manifest_{ts}.json")
        try:
            with open(man_path, "w", encoding="utf-8") as mf:
                json.dump(manifest, mf, indent=2)
        except OSError:
            pass
        if self._dc is not None:
            self._dc.reset()
        try:
            self._adpcm = open(self.adpcm_path, "wb", buffering=8192)
        except OSError:
            self._adpcm = None
        try:
            self._wave = wave.open(self.wav_path, "wb")
            self._wave.setnchannels(self._channels)
            self._wave.setsampwidth(self._sample_width)
            self._wave.setframerate(self._framerate)
        except OSError:
            self._wave = None
        self._recording = True
        return self.wav_path

    def write(self, pcm_int16: np.ndarray, adpcm_payload: Optional[bytes] = None) -> None:
        if not self._recording:
            return
        if pcm_int16.size == 0:
            return
        samples = np.asarray(pcm_int16, dtype=np.int16).reshape(-1)
        if self._dc is not None:
            samples = self._dc.process(samples)
        if self._gain != 1.0:
            samples = np.clip(
                np.rint(samples.astype(np.float64) * self._gain),
                -32768,
                32767,
            ).astype(np.int16)
        if self._wave is not None:
            self._wave.writeframes(samples.tobytes())
        if self._adpcm is not None and adpcm_payload:
            self._adpcm.write(adpcm_payload)

    def stop(self) -> Optional[str]:
        if not self._recording:
            return None
        self._recording = False
        if self._adpcm:
            try:
                self._adpcm.flush()
                self._adpcm.close()
            except OSError:
                pass
            self._adpcm = None
        if self._wave:
            try:
                self._wave.close()
            except OSError:
                pass
            self._wave = None
        path = self.wav_path
        self.wav_path = None
        self.adpcm_path = None
        return path
