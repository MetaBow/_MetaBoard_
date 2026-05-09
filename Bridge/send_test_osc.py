#!/usr/bin/env python3
"""
send_test_osc.py — Sends dummy mel_spectrogram OSC messages to test infer_model.py.

Sends random float arrays to 127.0.0.1:8888 at /metabow/audio/mel_spectrogram.
"""

import time
import argparse
import numpy as np
from pythonosc import udp_client

OSC_HOST = "127.0.0.1"
OSC_PORT = 8888
OSC_ADDRESS = "/metabow/audio/mel_spectrogram"

N_MELS = 128
N_FRAMES = 4        # number of time frames per message (adjust as needed)
SEND_RATE_HZ = 10   # messages per second


def main(rate_hz: int, n_frames: int):
    client = udp_client.SimpleUDPClient(OSC_HOST, OSC_PORT)
    interval = 1.0 / rate_hz
    payload_size = N_MELS * n_frames

    print(f"[INFO] Sending to {OSC_HOST}:{OSC_PORT}  →  {OSC_ADDRESS}")
    print(f"[INFO] Payload: {N_MELS} mels × {n_frames} frames = {payload_size} floats")
    print(f"[INFO] Rate: {rate_hz} msg/s  |  Press Ctrl-C to stop.\n")

    count = 0
    try:
        while True:
            t0 = time.perf_counter()

            # Random mel spectrogram values in a plausible range
            data = np.random.rand(payload_size).astype(np.float32)
            client.send_message(OSC_ADDRESS, data.tolist())

            count += 1
            print(f"[SEND #{count}] {payload_size} floats sent")

            elapsed = time.perf_counter() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped after {count} messages.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OSC mel_spectrogram test sender")
    parser.add_argument("--rate", type=int, default=SEND_RATE_HZ,
                        help=f"Messages per second (default: {SEND_RATE_HZ})")
    parser.add_argument("--frames", type=int, default=N_FRAMES,
                        help=f"Time frames per message (default: {N_FRAMES})")
    args = parser.parse_args()

    main(args.rate, args.frames)
