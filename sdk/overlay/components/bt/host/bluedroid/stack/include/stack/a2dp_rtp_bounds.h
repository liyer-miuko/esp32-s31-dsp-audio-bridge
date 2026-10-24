/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stack/bt_types.h"

/* This fork forwards the complete media packet. Validate every RTP length
 * before changing BT_HDR; byte loads also permit unaligned L2CAP payloads. */
static inline bool a2dp_rtp_bounds(const BT_HDR *packet, size_t minimum_payload,
                                  size_t *header_bytes, size_t *payload_bytes,
                                  uint16_t *sequence)
{
    if (!packet || packet->len < 12) return false;
    const uint8_t *p = (const uint8_t *)(packet + 1) + packet->offset;
    const size_t length = packet->len;
    if ((p[0] >> 6) != 2) return false;
    size_t h = 12u + 4u * (p[0] & 15u);
    if (h > length) return false;
    if (p[0] & 0x10u) {
        if (length - h < 4u) return false;
        const size_t words = ((size_t)p[h + 2] << 8) | p[h + 3];
        if (words > (length - h - 4u) / 4u) return false;
        h += 4u + words * 4u;
    }
    const size_t padding = (p[0] & 0x20u) ? p[length - 1] : 0u;
    if ((p[0] & 0x20u) && (!padding || padding > length - h)) return false;
    const size_t payload = length - h - padding;
    if (payload < minimum_payload) return false;
    *header_bytes = h;
    *payload_bytes = payload;
    if (sequence) *sequence = ((uint16_t)p[2] << 8) | p[3];
    return true;
}
