"""Shared session clock for time-aligning WAV, VB-Cable, ML frames, and monitor logs."""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, TextIO


@dataclass
class SessionClock:
    """
    One clock per BLE session: wall time + monotonic anchor + 16 kHz sample timeline.

    ``cumulative_pcm_samples`` advances by the length of each decoded PCM chunk pushed
    through :meth:`~metabow.audio.subsystem.AudioSubsystem.push_pcm_int16` so WAV byte position maps to the same
    timeline as alignment rows and (when present) ML frame indices.
    """

    session_id: str
    t0_wall_unix: float
    t0_mono: float
    cumulative_pcm_samples: int = 0
    pcm_push_seq: int = 0

    def host_mono_sec(self) -> float:
        return time.perf_counter() - self.t0_mono

    def wall_now(self) -> float:
        return time.time()

    def register_pcm_chunk(self, num_samples: int) -> Dict[str, Any]:
        """Call once per decoded audio packet; returns row fields for logging."""
        self.pcm_push_seq += 1
        self.cumulative_pcm_samples += int(num_samples)
        return {
            "session_id": self.session_id,
            "pcm_push_seq": self.pcm_push_seq,
            "cumulative_pcm_samples_end": self.cumulative_pcm_samples,
            "host_mono_sec": self.host_mono_sec(),
            "wall_unix": self.wall_now(),
            "event": "pcm_push",
            "samples_in_chunk": int(num_samples),
        }


class AlignmentJsonlWriter:
    """Append-only JSONL next to recordings for offline alignment."""

    def __init__(self, path: str, clock: SessionClock) -> None:
        self._path = path
        self._clock = clock
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._fp: Optional[TextIO] = open(path, "a", encoding="utf-8")
        self._write_manifest_row()

    def _write_manifest_row(self) -> None:
        if self._fp is None:
            return
        row = {
            "event": "session_start",
            "session_id": self._clock.session_id,
            "t0_wall_unix": self._clock.t0_wall_unix,
            "t0_mono_perf": self._clock.t0_mono,
            "nominal_sample_rate_hz": 16000,
        }
        self._fp.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._fp.flush()

    def write(self, row: Dict[str, Any]) -> None:
        if self._fp is None:
            return
        row = dict(row)
        row.setdefault("session_id", self._clock.session_id)
        self._fp.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._fp.flush()

    def close(self) -> None:
        if self._fp:
            try:
                self._fp.close()
            except OSError:
                pass
            self._fp = None
