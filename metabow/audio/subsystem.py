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
        if sid:
            self._clock = SessionClock(
                session_id=sid,
                t0_wall_unix=self.config.session_clock_t0_wall or time.time(),
                t0_mono=self.config.session_clock_t0_mono or time.perf_counter(),
            )

        self._align = None
        if self._clock is not None and self.config.write_alignment_log:
            ap = self.config.alignment_log_path or os.path.join(base_dir, f"{sid}_align.jsonl")
            self._align = AlignmentJsonlWriter(ap, self._clock)

        feat_cfg = self.config
        orig_cb = self.config.on_feature_frame
        if orig_cb is not None:

            def _wrapped_feat(
                fi: int,
                vals: Dict[str, Any],
                tw: float,
                tm: float,
            ) -> None:
                al = self._align
                if al is not None:
                    al.write(
                        {
                            "event": "ml_frame",
                            "frame_index": fi,
                            "wall_unix": tw,
                            "host_mono_sec": tm,
                        }
                    )
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

        # Wall-clock span for this subsystem (BLE session); used to diagnose WAV vs real time.
        self._ingress_mono_t0: float = time.perf_counter()
        self._ingress_pcm_samples: int = 0

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

    def start_recording_take(self, directory: str, basename: str) -> Optional[str]:
        """
        Begin WAV + raw ADPCM capture under ``directory``, using filenames derived from
        ``basename`` (see :meth:`DecodedPcmRecorder.start`). Opens ``alignment.jsonl`` in the
        same directory when a session clock exists (take-scoped alignment).
        """
        os.makedirs(directory, exist_ok=True)
        if self._align is not None:
            try:
                self._align.close()
            except OSError:
                pass
            self._align = None
        if self._clock is not None:
            align_path = os.path.join(directory, "alignment.jsonl")
            self._align = AlignmentJsonlWriter(align_path, self._clock)
        return self._recorder.start(directory, basename=basename)

    def stop_recording_take(self) -> Optional[str]:
        """Stop WAV/ADPCM; append ``recording_take_end`` and close take alignment writer."""
        wav_path = self._recorder.stop()
        if self._align is not None and self._clock is not None:
            row: Dict[str, Any] = {
                "event": "recording_take_end",
                "wall_unix": time.time(),
                "host_mono_sec": self._clock.host_mono_sec(),
                "cumulative_pcm_samples_end": self._clock.cumulative_pcm_samples,
            }
            try:
                self._align.write(row)
            except OSError:
                pass
            try:
                self._align.close()
            except OSError:
                pass
            self._align = None
        return wav_path

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
        self._ingress_pcm_samples += int(samples.size)

        if self._clock is not None:
            row = self._clock.register_pcm_chunk(samples.size)
            if self._align is not None:
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

        elapsed_mono = time.perf_counter() - self._ingress_mono_t0
        n_pcm = (
            self._clock.cumulative_pcm_samples
            if self._clock is not None
            else self._ingress_pcm_samples
        )
        eff_hz = (n_pcm / elapsed_mono) if elapsed_mono > 0 else 0.0
        nominal = float(self.config.sample_rate_hz or 16000)
        speed_vs_wall = (nominal / eff_hz) if eff_hz > 1e-6 else None

        if self._align is not None and self._clock is not None:
            row_end: Dict[str, Any] = {
                "event": "session_end",
                "host_session_sec": elapsed_mono,
                "cumulative_pcm_samples": n_pcm,
                "effective_sample_delivery_hz": eff_hz,
                "nominal_sample_rate_hz": int(nominal),
            }
            if speed_vs_wall is not None:
                row_end["approx_wav_playback_speed_vs_wallclock"] = speed_vs_wall
                # ~90 samples/packet @ 16 kHz → 177.78 pkt/s nominal; compare to diagnose BLE drops.
                row_end["approx_ble_packets_per_sec_if_90_samples"] = eff_hz / 90.0
            self._align.write(row_end)

        if speed_vs_wall is not None and speed_vs_wall > 1.05:
            print(
                f"[audio] WAV is stamped {int(nominal)} Hz but only ~{eff_hz:.0f} samples/s "
                f"were delivered (~{speed_vs_wall:.2f}× faster playback than wall clock). "
                "Causes: BLE/backlog drops, OS scheduling, or link throughput. "
                "See DecodedPcmRecorder docstring / alignment JSONL session_end.",
                flush=True,
            )

        self._recorder.stop()
        if self._cable is not None:
            self._cable.stop()
            self._cable = None
        if self._align is not None:
            self._align.close()
            self._align = None
        self._clock = None
