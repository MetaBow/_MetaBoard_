"""Single ingress for the modular bridge: fans out to features, recoding, and streaming."""

from __future__ import annotations

import os
import time
from dataclasses import replace
from typing import TYPE_CHECKING, Any, Dict, Optional

import numpy as np

from metabow.audio.alignment import AlignmentJsonlWriter, SessionClock
from metabow.audio.recoding import DecodedPcmRecorder
from metabow.audio.types import AudioProcessingStats, AudioSubsystemConfig

if TYPE_CHECKING:
    from metabow.audio.features_stage import FeatureExtractionStage
    from metabow.audio.streaming_stage import VirtualCableStreamStage


class AudioSubsystem:
    """
    Orchestrates the three audio threads of concern:

    1. **Feature extraction** — background librosa thread via ``FeatureExtractionStage``.
    2. **Recoding / CPU** — synchronous WAV + ADPCM sidecar + ingest timing stats.
    3. **Third-party streaming** — VB-Cable via ``sounddevice`` at 44.1 kHz.

    The BLE packet processor should call :meth:`push_pcm_int16` once per decoded frame.
    """

    def __init__(self, config: Optional[AudioSubsystemConfig] = None):
        self.config = config or AudioSubsystemConfig()
        self.stats = AudioProcessingStats()

        self._clock: Optional[SessionClock] = None
        self._align: Optional[AlignmentJsonlWriter] = None
        base_dir = self.config.recording_directory or os.path.expanduser("~/Documents/MetaBow_Data")
        sid = self.config.session_id
        if sid and self.config.write_alignment_log:
            self._clock = SessionClock(
                session_id=sid,
                t0_wall_unix=self.config.session_clock_t0_wall or time.time(),
                t0_mono=self.config.session_clock_t0_mono or time.perf_counter(),
            )
            ap = self.config.alignment_log_path or os.path.join(base_dir, f"{sid}_align.jsonl")
            self._align = AlignmentJsonlWriter(ap, self._clock)

        feat_cfg = self.config
        if self._align is not None:
            orig_cb = self.config.on_feature_frame

            def _wrapped_feat(
                fi: int,
                vals: Dict[str, Any],
                tw: float,
                tm: float,
            ) -> None:
                self._align.write(
                    {
                        "event": "ml_frame",
                        "frame_index": fi,
                        "wall_unix": tw,
                        "host_mono_sec": tm,
                    }
                )
                if orig_cb:
                    orig_cb(fi, vals, tw, tm)

            feat_cfg = replace(self.config, on_feature_frame=_wrapped_feat)

        self._features: Optional[FeatureExtractionStage] = None
        if feat_cfg.enable_feature_extraction:
            from metabow.audio.features_stage import FeatureExtractionStage

            self._features = FeatureExtractionStage(feat_cfg)
            self._features.start()

        self._recorder = DecodedPcmRecorder(self.config)
        if self.config.enable_recording:
            self._recorder.start(
                self.config.recording_directory
                or os.path.expanduser("~/Documents/MetaBow_Data")
            )

        self._cable: Optional[VirtualCableStreamStage] = None
        if self.config.enable_virtual_c_output:
            from metabow.audio.streaming_stage import VirtualCableStreamStage

            self._cable = VirtualCableStreamStage(self.config)
            self._cable.start()

    @property
    def feature_stage(self) -> Optional["FeatureExtractionStage"]:
        return self._features

    @property
    def recorder(self) -> DecodedPcmRecorder:
        return self._recorder

    @property
    def virtual_cable(self) -> Optional[VirtualCableStreamStage]:
        return self._cable

    @property
    def cumulative_pcm_samples(self) -> int:
        if self._clock is None:
            return 0
        return self._clock.cumulative_pcm_samples

    @property
    def session_clock(self) -> Optional[SessionClock]:
        return self._clock

    def push_pcm_int16(
        self,
        pcm_int16: np.ndarray,
        *,
        adpcm_payload: Optional[bytes] = None,
    ) -> None:
        t0 = time.perf_counter()
        samples = np.asarray(pcm_int16, dtype=np.int16).reshape(-1)
        if samples.size == 0:
            return

        if self._clock is not None and self._align is not None:
            row = self._clock.register_pcm_chunk(samples.size)
            self._align.write(row)

        # Parallel float view for ML + streaming (symmetric with firmware scaling).
        mono_f = samples.astype(np.float32) / 32768.0

        if self._features is not None:
            self._features.push_float32(mono_f)

        if self._recorder.recording:
            self._recorder.write(samples, adpcm_payload)

        if self._cable is not None:
            self._cable.push_float32(mono_f)

        if self.config.track_per_push_timing:
            self.stats.record((time.perf_counter() - t0) * 1000.0)

    def stop(self) -> None:
        if self._features is not None:
            self._features.stop()
            self._features = None
        self._recorder.stop()
        if self._cable is not None:
            self._cable.stop()
            self._cable = None
        if self._align is not None:
            self._align.close()
            self._align = None
        self._clock = None
