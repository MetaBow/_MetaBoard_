#!/usr/bin/env python3
"""
infer_clap_llm.py — CLAP zero-shot audio classification + lightweight LLM feedback.

OSC pipeline: /metabow/audio (raw float32 PCM, 16 kHz) →
  CLAP classification → flan-t5-small feedback on detected playing.

LLM is called only when the top label changes, to avoid flooding a slow model.
"""

import threading
import time
import numpy as np
import torch
from scipy.signal import resample_poly
from math import gcd
from transformers import ClapModel, ClapProcessor, T5ForConditionalGeneration, T5Tokenizer
from pythonosc import dispatcher, osc_server

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OSC_PORT = 8888
OSC_ADDRESS = "/metabow/audio"

SAMPLE_RATE = 16_000
CLAP_SAMPLE_RATE = 48_000
WINDOW_SECONDS = 1.0
OVERLAP_SECONDS = 0.5
HOP_SECONDS = WINDOW_SECONDS - OVERLAP_SECONDS

WINDOW_SAMPLES = int(SAMPLE_RATE * WINDOW_SECONDS)
HOP_SAMPLES = int(SAMPLE_RATE * HOP_SECONDS)

LABELS = [
    # --- Bowing quality ---
    "smooth and resonant legato bow stroke on a violin",
    "tense and scratchy bowing with too much bow pressure",
    "shaky and uneven bow stroke with unstable wavering tone",
    "slow and sustained bow stroke with full rich tone",
    "fast and light spiccato bowing bouncing off the string",
    "short and crisp staccato bow strokes",
    "rapid tremolo bowing with quick back-and-forth strokes",
    "loud and powerful forceful bow stroke at full volume",
    "soft and delicate pianissimo bowing barely audible",
    # --- Bow position / extended technique ---
    "sul tasto flute-like breathy tone bowing over the fingerboard",
    "sul ponticello bright glassy metallic tone bowing near the bridge",
    "col legno tapping the string with the wood of the bow",
    "natural harmonic producing a high clear ethereal tone",
    "double stop bowing across two strings at once",
    # --- Expressive / articulation ---
    "expressive vibrato with warm oscillating pitch",
    "clean and in-tune playing with steady stable pitch",
    "out of tune playing with pitch drifting off center",
    "accelerating run of notes played with the bow",
    # --- Plucking ---
    "light pizzicato plucking on a string instrument",
    "strong loud pizzicato pluck on a string",
    "snap pizzicato with string slapping against the fingerboard",
    # --- Dynamics extremes ---
    "extremely quiet playing almost inaudible",
    "very loud and intense playing at maximum volume",
    # --- Non-playing ---
    "silence or no sound",
    "ambient background noise in a room",
    "bow rosin friction noise without a clear pitch",
]

CLAP_MODEL_NAME = "laion/clap-htsat-fused"
LLM_MODEL_NAME = "google/flan-t5-small"   # ~300 MB, CPU-friendly

# Min confidence to trigger LLM feedback (avoids feedback on ambiguous detections)
FEEDBACK_THRESHOLD = 0.0   # disabled — rely on throttle alone

# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------

print(f"[INFO] Loading {CLAP_MODEL_NAME} …")
_clap_processor = ClapProcessor.from_pretrained(CLAP_MODEL_NAME)
_clap_model = ClapModel.from_pretrained(CLAP_MODEL_NAME)
_clap_model.eval()
print("[INFO] CLAP ready.")

print(f"[INFO] Loading {LLM_MODEL_NAME} …")
_llm_tokenizer = T5Tokenizer.from_pretrained(LLM_MODEL_NAME)
_llm_model = T5ForConditionalGeneration.from_pretrained(LLM_MODEL_NAME)
_llm_model.eval()
print("[INFO] LLM ready.\n")

# Pre-compute text embeddings once
with torch.no_grad():
    _text_inputs = _clap_processor(text=LABELS, return_tensors="pt", padding=True)
    _text_embeds = _clap_model.get_text_features(**_text_inputs)
    if not isinstance(_text_embeds, torch.Tensor):
        _text_embeds = _text_embeds.pooler_output
    _text_embeds = torch.nn.functional.normalize(_text_embeds, dim=-1)

# ---------------------------------------------------------------------------
# Audio buffer
# ---------------------------------------------------------------------------

_buffer: list[float] = []
_buffer_lock = threading.Lock()
_infer_count = 0

# ---------------------------------------------------------------------------
# LLM feedback
# ---------------------------------------------------------------------------

_last_label: str | None = None
_last_llm_time: float = 0.0
LLM_THROTTLE_SECONDS = 3.0
_llm_lock = threading.Lock()   # only one LLM call at a time


def _build_prompt(label: str, confidence: float, all_scores: list[tuple[str, float]]) -> str:
    scores_str = ", ".join(f"{l} ({p:.0%})" for l, p in all_scores)
    return (
        f"A musician is playing. The audio classifier detected: {label} "
        f"with {confidence:.0%} confidence. All scores: {scores_str}. "
        f"In one short sentence, give a feedback on the playing."
        f"Ignore background noise and silence."
    )


def _run_llm_feedback(label: str, confidence: float, all_scores: list[tuple[str, float]], infer_count: int) -> None:
    try:
        print(f"[LLM #{infer_count}] generating…", flush=True)
        with _llm_lock:
            prompt = _build_prompt(label, confidence, all_scores)
            inputs = _llm_tokenizer(prompt, return_tensors="pt", truncation=True, max_length=128)
            with torch.no_grad():
                output_ids = _llm_model.generate(
                    **inputs,
                    max_new_tokens=60,
                    num_beams=2,
                    early_stopping=True,
                )
            feedback = _llm_tokenizer.decode(output_ids[0], skip_special_tokens=True)
            print(f"[LLM #{infer_count}] {feedback}\n", flush=True)
    except Exception as e:
        print(f"[LLM ERROR] {e}", flush=True)

# ---------------------------------------------------------------------------
# CLAP inference
# ---------------------------------------------------------------------------

def _run_inference(waveform: np.ndarray) -> None:
    global _infer_count, _last_label, _last_llm_time

    g = gcd(CLAP_SAMPLE_RATE, SAMPLE_RATE)
    waveform_48k = resample_poly(
        waveform, CLAP_SAMPLE_RATE // g, SAMPLE_RATE // g
    ).astype(np.float32)

    with torch.no_grad():
        audio_inputs = _clap_processor(
            audio=waveform_48k,
            sampling_rate=CLAP_SAMPLE_RATE,
            return_tensors="pt",
        )
        audio_embed = _clap_model.get_audio_features(**audio_inputs)
        if not isinstance(audio_embed, torch.Tensor):
            audio_embed = audio_embed.pooler_output
        audio_embed = torch.nn.functional.normalize(audio_embed, dim=-1)

    sims = (audio_embed @ _text_embeds.T).squeeze(0)
    probs = torch.softmax(sims * 10.0, dim=-1)
    best = int(probs.argmax())
    best_label = LABELS[best]
    best_prob = float(probs[best])

    _infer_count += 1


    # Trigger LLM at most every LLM_THROTTLE_SECONDS, and only above confidence threshold
    now = time.monotonic()
    if best_prob >= FEEDBACK_THRESHOLD and (now - _last_llm_time) >= LLM_THROTTLE_SECONDS:
        _last_label = best_label
        _last_llm_time = now
        all_scores = sorted(
            [(LABELS[i], float(probs[i])) for i in range(len(LABELS))],
            key=lambda x: x[1], reverse=True
        )[:5]

        threading.Thread(
            target=_run_llm_feedback,
            args=(best_label, best_prob, all_scores, _infer_count),
            daemon=True,
        ).start()

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

        window = np.array(_buffer[:WINDOW_SAMPLES], dtype=np.float32)
        _buffer = _buffer[HOP_SAMPLES:]

    threading.Thread(target=_run_inference, args=(window,), daemon=True).start()

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    d = dispatcher.Dispatcher()
    d.map(OSC_ADDRESS, handle_audio)

    server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", OSC_PORT), d)
    print(f"[INFO] Listening on 127.0.0.1:{OSC_PORT}  →  {OSC_ADDRESS}")
    print(f"[INFO] Window={WINDOW_SECONDS}s  Hop={HOP_SECONDS}s")
    print(f"[INFO] LLM feedback throttle={LLM_THROTTLE_SECONDS}s  confidence threshold={FEEDBACK_THRESHOLD:.0%}")
    print("[INFO] Press Ctrl-C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
