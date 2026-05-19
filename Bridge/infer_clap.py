#!/usr/bin/env python3
"""
infer_clap.py — OSC listener that runs zero-shot audio classification
using laion/clap-htsat-fused from HuggingFace.

Listens on localhost:8888 for /metabow/audio messages (raw float32 PCM,
16 kHz, normalized -1.0 to 1.0). Accumulates samples into a rolling window,
then runs CLAP inference and prints similarity scores against LABELS.
"""

import threading
import numpy as np
import torch
from scipy.signal import resample_poly
from math import gcd
from transformers import ClapModel, ClapProcessor
from pythonosc import dispatcher, osc_server

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OSC_PORT = 8888
OSC_ADDRESS = "/metabow/audio"

SAMPLE_RATE = 16_000          # must match firmware / metaboard_osc_monitor.py
CLAP_SAMPLE_RATE = 48_000     # CLAP was trained at 48 kHz
WINDOW_SECONDS = 1.0          # audio window fed to CLAP
OVERLAP_SECONDS = 0.5         # how much to retain after each inference
HOP_SECONDS = WINDOW_SECONDS - OVERLAP_SECONDS  # infer every 0.5 s

WINDOW_SAMPLES = int(SAMPLE_RATE * WINDOW_SECONDS)
HOP_SAMPLES = int(SAMPLE_RATE * HOP_SECONDS)

# Edit these labels to match your classification task
LABELS = [
    "bowing a violin string",
    "plucking a string",
    "silence",
    "background noise",
]

MODEL_NAME = "laion/clap-htsat-fused"

# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------

print(f"[INFO] Loading {MODEL_NAME} …")
_processor = ClapProcessor.from_pretrained(MODEL_NAME)
_model = ClapModel.from_pretrained(MODEL_NAME)
_model.eval()
print("[INFO] Model ready.")

# Pre-compute text embeddings once — they don't change
with torch.no_grad():
    _text_inputs = _processor(
        text=LABELS,
        return_tensors="pt",
        padding=True,
    )
    _text_embeds = _model.get_text_features(**_text_inputs)
    if not isinstance(_text_embeds, torch.Tensor):
        _text_embeds = _text_embeds.pooler_output  # (n_labels, d)
    _text_embeds = torch.nn.functional.normalize(_text_embeds, dim=-1)

# ---------------------------------------------------------------------------
# Audio buffer (shared between OSC thread and inference)
# ---------------------------------------------------------------------------

_buffer: list[float] = []
_buffer_lock = threading.Lock()
_infer_count = 0

# ---------------------------------------------------------------------------
# Inference
# ---------------------------------------------------------------------------

def _run_inference(waveform: np.ndarray) -> None:
    global _infer_count

    # Resample from 16 kHz to 48 kHz (CLAP requirement)
    g = gcd(CLAP_SAMPLE_RATE, SAMPLE_RATE)
    waveform_48k = resample_poly(waveform, CLAP_SAMPLE_RATE // g, SAMPLE_RATE // g).astype(np.float32)

    with torch.no_grad():
        audio_inputs = _processor(
            audio=waveform_48k,
            sampling_rate=CLAP_SAMPLE_RATE,
            return_tensors="pt",
        )
        audio_embed = _model.get_audio_features(**audio_inputs)
        if not isinstance(audio_embed, torch.Tensor):
            audio_embed = audio_embed.pooler_output
        audio_embed = torch.nn.functional.normalize(audio_embed, dim=-1)  # (1, d)

    # Cosine similarities → softmax probabilities
    sims = (audio_embed @ _text_embeds.T).squeeze(0)  # (n_labels,)
    probs = torch.softmax(sims * 10.0, dim=-1)        # temperature=10 sharpens scores
    best = int(probs.argmax())

    _infer_count += 1
    scores = "  ".join(f"{LABELS[i]}={probs[i]:.3f}" for i in range(len(LABELS)))
    print(f"[INFER #{_infer_count}] → {LABELS[best]}  |  {scores}")

# ---------------------------------------------------------------------------
# OSC handler
# ---------------------------------------------------------------------------

def handle_audio(address: str, *args: float) -> None:
    global _buffer

    samples = list(args)
    if not samples:
        return

    with _buffer_lock:
        _buffer.extend(samples)
        if len(_buffer) < WINDOW_SAMPLES:
            return

        # Grab the window and slide
        window = np.array(_buffer[:WINDOW_SAMPLES], dtype=np.float32)
        _buffer = _buffer[HOP_SAMPLES:]

    # Run inference off the OSC thread so we don't block packet reception
    threading.Thread(target=_run_inference, args=(window,), daemon=True).start()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    d = dispatcher.Dispatcher()
    d.map(OSC_ADDRESS, handle_audio)

    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", OSC_PORT), d)
    print(f"[INFO] Listening on 127.0.0.1:{OSC_PORT}  →  {OSC_ADDRESS}")
    print(f"[INFO] Window={WINDOW_SECONDS}s  Hop={HOP_SECONDS}s  Labels={LABELS}")
    print("[INFO] Press Ctrl-C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
