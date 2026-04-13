"""Shared audio datatypes for the modular bridge."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

# ML pipeline defaults (documented for Gestura handoff)
ML_FRAME_SIZE: int = 2048
ML_HOP_LENGTH: int = 512

# Hardware / bridge convention: decoded ADPCM → mono PCM at 16 kHz (see AUDIT.md).
METABOW_AUDIO_SAMPLE_RATE_HZ: int = 16_000
# VB-Audio virtual cable path expects host-rate PCM (matches PARALLELIZED mega-script).
VIRTUAL_CABLE_OUTPUT_HZ: int = 44_100


@dataclass
class AudioProcessingStats:
    """Lightweight per-push timing for CPU / pipeline diagnostics (bridge or UI thread)."""

    push_count: int = 0
    total_time_ms: float = 0.0
    peak_time_ms: float = 0.0

    def record(self, duration_ms: float) -> None:
        self.push_count += 1
        self.total_time_ms += duration_ms
        if duration_ms > self.peak_time_ms:
            self.peak_time_ms = duration_ms

    @property
    def mean_time_ms(self) -> float:
        if self.push_count == 0:
            return 0.0
        return self.total_time_ms / self.push_count


@dataclass
class AudioSubsystemConfig:
    """Wire-up for the three audio applications: ML features, file/CPU path, virtual cable.

    WAV files use the nominal device rate (16 kHz) but only contain samples that BLE delivered.
    Heavy OS-side packet loss makes playback faster than real time unless you time-stretch in a
    DAW or firmware adds a frame counter so missing packets can be padded accurately.
    """

    sample_rate_hz: int = METABOW_AUDIO_SAMPLE_RATE_HZ
    enable_feature_extraction: bool = True
    enable_recording: bool = False
    enable_virtual_c_output: bool = False
    recording_directory: Optional[str] = None
    track_per_push_timing: bool = True
    virtual_cable_blocksize: int = 512
    # WAV path: 114-byte ADPCM + BLE drops → decoder DC wander → asymmetric clipping. High-pass helps.
    record_dc_block_corner_hz: float = 15.0  # 0 = off
    record_output_gain: float = 1.0  # e.g. 0.92 if peaks still hit full scale after DC block

    # Session alignment (recording, streaming, features, CSV logs share ``session_id``)
    session_id: Optional[str] = None
    session_clock_t0_wall: Optional[float] = None
    session_clock_t0_mono: Optional[float] = None
    alignment_log_path: Optional[str] = None  # default: MetaBow_Data/{session_id}_align.jsonl
    write_alignment_log: bool = True

    # ML: enable a small preset when feature extraction starts (monitor has no Tk config UI)
    apply_basic_ml_preset: bool = True
    # (frame_index, feature_values, wall_unix, host_mono_sec_since_session)
    on_feature_frame: Optional[Callable[[int, Dict[str, Any], float, float], None]] = None
