/*
 * ADPCM Audio Codec Implementation
 * IMA ADPCM algorithm for audio compression
 */

#include "adpcm_codec.h"
#include <string.h>

/* Clamp value between min and max */
static inline int32_t clamp(int32_t value, int32_t min, int32_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* Initialize encoder state */
void adpcm_encoder_init(adpcm_encoder_state_t *state) {
    state->predicted_sample = 0;
    state->step_index = 0;
}

/* Initialize decoder state */
void adpcm_decoder_init(adpcm_decoder_state_t *state) {
    state->predicted_sample = 0;
    state->step_index = 0;
}

/* Encode a single PCM sample to 4-bit ADPCM */
static uint8_t adpcm_encode_sample(int16_t sample, adpcm_encoder_state_t *state) {
    int32_t diff;
    int32_t step;
    uint8_t adpcm_sample = 0;
    
    /* Get current step size */
    step = adpcm_step_table[state->step_index];
    
    /* Calculate difference from predicted sample */
    diff = sample - state->predicted_sample;
    
    /* Set sign bit and get absolute difference */
    if (diff < 0) {
        adpcm_sample = 8;  /* Set sign bit */
        diff = -diff;
    }
    
    /* Quantize the difference */
    if (diff >= step) {
        adpcm_sample |= 4;
        diff -= step;
    }
    if (diff >= (step >> 1)) {
        adpcm_sample |= 2;
        diff -= (step >> 1);
    }
    if (diff >= (step >> 2)) {
        adpcm_sample |= 1;
    }
    
    /* Update predicted sample */
    diff = 0;
    if (adpcm_sample & 4) diff += step;
    if (adpcm_sample & 2) diff += (step >> 1);
    if (adpcm_sample & 1) diff += (step >> 2);
    diff += (step >> 3);
    
    if (adpcm_sample & 8) {
        state->predicted_sample -= diff;
    } else {
        state->predicted_sample += diff;
    }
    
    /* Clamp predicted sample to valid range */
    state->predicted_sample = clamp(state->predicted_sample, -32768, 32767);
    
    /* Update step index */
    state->step_index += adpcm_index_table[adpcm_sample];
    state->step_index = clamp(state->step_index, 0, 88);
    
    return adpcm_sample;
}

/* Decode a single 4-bit ADPCM sample to PCM */
static int16_t adpcm_decode_sample(uint8_t adpcm_sample, adpcm_decoder_state_t *state) {
    int32_t diff;
    int32_t step;
    
    /* Get current step size */
    step = adpcm_step_table[state->step_index];
    
    /* Calculate difference */
    diff = 0;
    if (adpcm_sample & 4) diff += step;
    if (adpcm_sample & 2) diff += (step >> 1);
    if (adpcm_sample & 1) diff += (step >> 2);
    diff += (step >> 3);
    
    /* Apply sign */
    if (adpcm_sample & 8) {
        state->predicted_sample -= diff;
    } else {
        state->predicted_sample += diff;
    }
    
    /* Clamp predicted sample to valid range */
    state->predicted_sample = clamp(state->predicted_sample, -32768, 32767);
    
    /* Update step index */
    state->step_index += adpcm_index_table[adpcm_sample & 0x0F];
    state->step_index = clamp(state->step_index, 0, 88);
    
    return state->predicted_sample;
}

/* Encode PCM samples to ADPCM */
size_t adpcm_encode(const int16_t *pcm_data, size_t pcm_samples,
                    uint8_t *adpcm_data, adpcm_encoder_state_t *state) {
    size_t adpcm_bytes = 0;
    size_t i;
    uint8_t nibble_high, nibble_low;
    
    /* Process samples in pairs */
    for (i = 0; i < pcm_samples; i += 2) {
        /* Encode first sample (high nibble) */
        nibble_high = adpcm_encode_sample(pcm_data[i], state);
        
        /* Encode second sample (low nibble) if available */
        if (i + 1 < pcm_samples) {
            nibble_low = adpcm_encode_sample(pcm_data[i + 1], state);
        } else {
            nibble_low = 0;  /* Pad with zero if odd number of samples */
        }
        
        /* Pack two 4-bit samples into one byte */
        adpcm_data[adpcm_bytes++] = (nibble_high << 4) | (nibble_low & 0x0F);
    }
    
    return adpcm_bytes;
}

/* Decode ADPCM to PCM samples */
size_t adpcm_decode(const uint8_t *adpcm_data, size_t adpcm_bytes,
                    int16_t *pcm_data, adpcm_decoder_state_t *state) {
    size_t pcm_samples = 0;
    size_t i;
    uint8_t byte;
    
    for (i = 0; i < adpcm_bytes; i++) {
        byte = adpcm_data[i];
        
        /* Decode high nibble */
        pcm_data[pcm_samples++] = adpcm_decode_sample((byte >> 4) & 0x0F, state);
        
        /* Decode low nibble */
        pcm_data[pcm_samples++] = adpcm_decode_sample(byte & 0x0F, state);
    }
    
    return pcm_samples;
}