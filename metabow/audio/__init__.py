"""
MetaBow audio package — three coordinated paths from one decoded PCM ingress.

* **Feature extraction** — :class:`FeatureExtractionStage` / ``RealTimeAudioFeatureExtractor``
* **Recoding** — :class:`DecodedPcmRecorder` (WAV + ADPCM sidecar) + :class:`AudioProcessingStats`
* **Streaming** — :class:`VirtualCableStreamStage` (VB-Cable via sounddevice)

Typical bridge integration::

    audio = AudioSubsystem(AudioSubsystemConfig(enable_feature_extraction=True))
    ...
    audio.push_pcm_int16(pcm_int16, adpcm_payload=raw_adpcm_45b)
"""

from __future__ import annotations

from metabow.audio.features_stage import FeatureExtractionStage
from metabow.audio.recoding import DecodedPcmRecorder
from metabow.audio.streaming_stage import VirtualCableDeviceProbe, VirtualCableStreamStage
from metabow.audio.subsystem import AudioSubsystem
from metabow.audio.types import (
    ML_FRAME_SIZE,
    ML_HOP_LENGTH,
    METABOW_AUDIO_SAMPLE_RATE_HZ,
    VIRTUAL_CABLE_OUTPUT_HZ,
    AudioProcessingStats,
    AudioSubsystemConfig,
)

__all__ = [
    "ML_FRAME_SIZE",
    "ML_HOP_LENGTH",
    "METABOW_AUDIO_SAMPLE_RATE_HZ",
    "VIRTUAL_CABLE_OUTPUT_HZ",
    "AudioProcessingStats",
    "AudioSubsystemConfig",
    "AudioSubsystem",
    "DecodedPcmRecorder",
    "FeatureExtractionStage",
    "VirtualCableDeviceProbe",
    "VirtualCableStreamStage",
]
