#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *bytes;
    size_t capacity, filled, sent;
} pcm_dma_stage_t;

/* Sink returns the accepted prefix, zero when blocked. It must never return
 * more than size. Normally the I2S sink accepts a full block or no bytes. */
typedef size_t (*pcm_dma_sink_t)(const void *data, size_t size, void *ctx);
void pcm_dma_stage_reset(pcm_dma_stage_t *stage, size_t capacity);
/* Returns input bytes durably accepted into the stage, including a blocked
 * full block. Callers retry only the unaccepted input suffix. No padding. */
size_t pcm_dma_stage_push(pcm_dma_stage_t *stage, const void *data, size_t size,
                          pcm_dma_sink_t sink, void *ctx);
