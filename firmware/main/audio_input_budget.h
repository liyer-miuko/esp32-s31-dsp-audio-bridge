/* Core0-only accounting. No allocation, scheduler or DSP ownership changes. */
#pragma once
#include <stdbool.h>
void audio_input_budget_poll(bool bt_selected);
