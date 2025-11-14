/*
 * ADPCM Audio Codec for BLE Audio Streaming
 * Provides 4:1 compression ratio (16-bit PCM to 4-bit ADPCM)
 */

#ifndef _ADPCM_CODEC_H_
#define _ADPCM_CODEC_H_

#include <stdint.h>
#include <stddef.h>

/* ADPCM index table for step size adaptation */
static const int8_t adpcm_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

/* ADPCM step size table */
static const uint16_t adpcm_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

/* ADPCM encoder state structure */
typedef struct {
    int16_t predicted_sample;  /* Predicted sample value */
    uint8_t step_index;       /* Current index into step table */
} adpcm_encoder_state_t;

/* ADPCM decoder state structure */
typedef struct {
    int16_t predicted_sample;  /* Predicted sample value */
    uint8_t step_index;       /* Current index into step table */
} adpcm_decoder_state_t;

/* Initialize encoder state */
void adpcm_encoder_init(adpcm_encoder_state_t *state);

/* Initialize decoder state */
void adpcm_decoder_init(adpcm_decoder_state_t *state);

/* Encode PCM samples to ADPCM
 * Input: pcm_data - 16-bit PCM samples
 *        pcm_samples - Number of PCM samples
 *        state - Encoder state
 * Output: adpcm_data - 4-bit ADPCM data (packed, 2 samples per byte)
 * Returns: Number of ADPCM bytes produced
 */
size_t adpcm_encode(const int16_t *pcm_data, size_t pcm_samples,
                    uint8_t *adpcm_data, adpcm_encoder_state_t *state);

/* Decode ADPCM to PCM samples
 * Input: adpcm_data - 4-bit ADPCM data (packed, 2 samples per byte)
 *        adpcm_bytes - Number of ADPCM bytes
 *        state - Decoder state
 * Output: pcm_data - 16-bit PCM samples
 * Returns: Number of PCM samples produced
 */
size_t adpcm_decode(const uint8_t *adpcm_data, size_t adpcm_bytes,
                    int16_t *pcm_data, adpcm_decoder_state_t *state);

/* Helper function to get compressed size for given PCM samples */
static inline size_t adpcm_get_compressed_size(size_t pcm_samples) {
    return (pcm_samples + 1) / 2;  /* 2 ADPCM samples per byte */
}

/* Helper function to get decompressed size for given ADPCM bytes */
static inline size_t adpcm_get_decompressed_samples(size_t adpcm_bytes) {
    return adpcm_bytes * 2;  /* 2 ADPCM samples per byte */
}

#endif /* _ADPCM_CODEC_H_ */