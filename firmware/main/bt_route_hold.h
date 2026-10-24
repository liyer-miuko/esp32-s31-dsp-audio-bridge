/* Ownership across SUSPEND and recognized A2DP replacement, core0 only.
 * Callers serialize this state; time is the monotonic esp_timer clock. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool selected;
    bool waiting;
    bool reconfig;
    int64_t until_us;
    uint32_t epoch;
} bt_route_hold_t;

static inline void bt_route_hold_clear(bt_route_hold_t *s)
{
    s->waiting = false;
    s->reconfig = false;
    s->until_us = 0;
}

static inline void bt_route_hold_select(bt_route_hold_t *s, bool selected)
{
    if (s->selected != selected) {
        s->selected = selected;
        bt_route_hold_clear(s);
    }
}

static inline bool bt_route_hold_begin(bt_route_hold_t *s, bool link_up,
                                       int64_t now_us, int64_t window_us)
{
    if (!s->selected || !link_up || (s->waiting && s->reconfig)) return false;
    s->waiting = true;
    s->reconfig = true; /* A validated CFG promotes a tentative SUSPEND/profile close. */
    s->until_us = now_us + window_us;
    return true;
}

static inline void bt_route_hold_suspend(bt_route_hold_t *s, bool link_up,
                                         int64_t now_us, int64_t grace_us)
{
    if (!s->selected || !link_up) return;
    s->waiting = true;
    s->reconfig = false;
    s->until_us = now_us + grace_us;
}

static inline bool bt_route_hold_active(const bt_route_hold_t *s, bool link_up,
                                        int64_t now_us)
{
    /* Only an unconfirmed SUSPEND expires. For a recognized replacement the
     * deadline is diagnostic; readiness or a terminal event releases it. */
    return s->selected && s->waiting && link_up &&
           (s->reconfig || now_us < s->until_us);
}

static inline bool bt_route_hold_ready(bt_route_hold_t *s, bool link_up,
                                       uint32_t epoch)
{
    if (!s->selected || !link_up || epoch != s->epoch) return false;
    bool ended = s->waiting;
    bt_route_hold_clear(s);
    return ended;
}
