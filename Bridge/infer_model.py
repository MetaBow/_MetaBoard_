#!/usr/bin/env python3
"""
infer_model.py — OSC listener that runs inference on incoming mel_spectrogram data.

Listens on localhost:8888 for /metabow/audio/mel_spectrogram messages.
Each message carries a flat list of floats (128 mel bins × N frames).
A dummy PyTorch model runs inference and prints the prediction.
"""

import torch
import torch.nn as nn
import numpy as np
from pythonosc import dispatcher, osc_server
from time import time

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OSC_PORT = 8888
OSC_ADDRESS = "/metabow/audio/mel_spectrogram"

N_MELS = 128       # must match audio_feature_extractor.py
N_CLASSES = 4      # arbitrary — change to match your actual label set

# ---------------------------------------------------------------------------
# Dummy model
# ---------------------------------------------------------------------------
'''
To be swapped with the train model.
This dummy is just a simple feedforward network.
'''
class DummyMelClassifier(nn.Module):
    def __init__(self, input_size: int, n_classes: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 64),
            nn.ReLU(),
            nn.Linear(64, n_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


# ---------------------------------------------------------------------------
# OSC handler
# ---------------------------------------------------------------------------

def handle_mel_spectrogram(address: str, *args):
    """Called once per OSC message on /metabow/audio/mel_spectrogram."""
    values = list(args)

    if not values:
        print("[WARN] Empty mel_spectrogram message — skipping.")
        return

    data = np.array(values, dtype=np.float32)

    # Reshape if the payload is a multiple of N_MELS; otherwise use flat.
    if data.size % N_MELS == 0:
        n_frames = data.size // N_MELS
        data = data.reshape(n_frames, N_MELS)

    flat = torch.from_numpy(data.flatten()).unsqueeze(0)  # (1, features)

    # Lazily build model on first call so input_size is inferred from data.
    if handle_mel_spectrogram.model is None:
        input_size = flat.shape[1]
        handle_mel_spectrogram.model = DummyMelClassifier(input_size, N_CLASSES)
        handle_mel_spectrogram.model.eval()
        print(f"[INFO] Model created — input_size={input_size}, n_classes={N_CLASSES}")

    with torch.no_grad():
        time_start = time()
        logits = handle_mel_spectrogram.model(flat)          # (1, N_CLASSES)
        probs = torch.softmax(logits, dim=-1).squeeze()      # (N_CLASSES,)
        pred_class = int(probs.argmax())
        confidence = float(probs[pred_class])
        time_end = time()

    print(f"[INFER] class={pred_class}  confidence={confidence:.3f}  "
          f"probs={[f'{p:.3f}' for p in probs.tolist()]}")
    
    # create attribute
    if not hasattr(handle_mel_spectrogram, "infer_count"):
        handle_mel_spectrogram.infer_count = 0
    handle_mel_spectrogram.infer_count += 1

    # throttling: only print timing every 10 inferences to avoid spamming logs
    if handle_mel_spectrogram.infer_count % 10 == 0:
        print(f"[TIMING] Inference took {(time_end - time_start)*1000:.2f} ms")


handle_mel_spectrogram.model = None  # lazy init


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    d = dispatcher.Dispatcher()
    d.map(OSC_ADDRESS, handle_mel_spectrogram)

    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", OSC_PORT), d)
    print(f"[INFO] Listening on 127.0.0.1:{OSC_PORT}  →  {OSC_ADDRESS}")
    print("[INFO] Press Ctrl-C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
