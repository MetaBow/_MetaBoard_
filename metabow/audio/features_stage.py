"""Thread 1: librosa-backed feature extraction (hands off to existing extractor)."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from metabow.audio.types import (
    ML_FRAME_SIZE,
    ML_HOP_LENGTH,
    AudioSubsystemConfig,
    METABOW_AUDIO_SAMPLE_RATE_HZ,
)

_BASIC_PRESET = (
    "spectral_centroid",
    "rms",
    "mfcc",
    "zero_crossing_rate",
    "chroma_stft",
)


class FeatureExtractionStage:
    """
    Adapter around ``RealTimeAudioFeatureExtractor`` with MetaBow-safe defaults.

    Imports ``audio_feature_extractor`` only when this stage is constructed (needs librosa).
    Runs librosa work on the extractor's own background thread; this stage only enqueues samples.
    """

    def __init__(self, config: AudioSubsystemConfig):
        try:
            from audio_feature_extractor import RealTimeAudioFeatureExtractor
        except ModuleNotFoundError as exc:
            raise ImportError(
                "Feature extraction requires `audio_feature_extractor` on PYTHONPATH "
                "and its dependencies (e.g. librosa). "
                "Install with: pip install librosa"
            ) from exc
        sr = config.sample_rate_hz or METABOW_AUDIO_SAMPLE_RATE_HZ
        mono_t0 = config.session_clock_t0_mono
        if mono_t0 is None:
            mono_t0 = time.perf_counter()
        self._extractor: Any = RealTimeAudioFeatureExtractor(
            sample_rate=sr,
            frame_size=ML_FRAME_SIZE,
            hop_length=ML_HOP_LENGTH,
            frame_callback=config.on_feature_frame,
            mono_anchor_t0=mono_t0,
        )
        if config.apply_basic_ml_preset:
            for name in _BASIC_PRESET:
                if name in self._extractor.feature_configs:
                    self._extractor.set_feature_enabled(name, True)

    @property
    def extractor(self) -> Any:
        return self._extractor

    def start(self) -> None:
        self._extractor.set_enabled(True)

    def stop(self) -> None:
        self._extractor.set_enabled(False)

    def push_float32(self, mono: np.ndarray) -> None:
        if mono.size == 0:
            return
        x = np.asarray(mono, dtype=np.float32).reshape(-1)
        self._extractor.add_audio_data(x)
