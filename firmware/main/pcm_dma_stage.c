#include "pcm_dma_stage.h"
#include <string.h>

void pcm_dma_stage_reset(pcm_dma_stage_t *stage, size_t capacity)
{
    stage->capacity = capacity;
    stage->filled = 0;
    stage->sent = 0;
}

size_t pcm_dma_stage_push(pcm_dma_stage_t *stage, const void *data, size_t size,
                          pcm_dma_sink_t sink, void *ctx)
{
    if (stage == NULL || stage->bytes == NULL || stage->capacity == 0 ||
        sink == NULL || (data == NULL && size != 0)) return 0;
    size_t accepted = 0;
    for (;;) {
        if (stage->filled == stage->capacity) {
            const size_t remaining = stage->capacity - stage->sent;
            const size_t written = sink(stage->bytes + stage->sent, remaining, ctx);
            if (written == 0 || written > remaining) break;
            stage->sent += written;
            if (stage->sent != stage->capacity) break;
            stage->filled = stage->sent = 0;
        }
        if (accepted == size) break;
        size_t take = stage->capacity - stage->filled;
        if (take > size - accepted) take = size - accepted;
        memcpy(stage->bytes + stage->filled, (const uint8_t *)data + accepted, take);
        stage->filled += take;
        accepted += take;
    }
    return accepted;
}
