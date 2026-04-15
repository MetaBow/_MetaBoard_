"""Background thread that decouples BLE packet processing from heavy audio fan-out."""

from __future__ import annotations

import queue
import threading
from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from metabow.audio.subsystem import AudioSubsystem


class AudioIngestThread:
    """Drains a queue of (pcm_int16, adpcm_payload) tuples and calls
    ``AudioSubsystem._sync_push`` on a dedicated daemon thread so that the
    BLE packet-processing thread never blocks on WAV I/O, librosa, or
    PortAudio writes."""

    def __init__(self, subsystem: AudioSubsystem) -> None:
        self._sub = subsystem
        self._q: queue.Queue = queue.Queue(maxsize=1024)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="audio_ingest"
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=3.0)

    def enqueue(self, pcm_int16: np.ndarray, adpcm_payload: Optional[bytes] = None) -> None:
        try:
            self._q.put_nowait((pcm_int16, adpcm_payload))
        except queue.Full:
            pass

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            pcm, adpcm = item
            self._sub._sync_push(pcm, adpcm_payload=adpcm)
