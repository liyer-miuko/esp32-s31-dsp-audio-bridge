/* Core0 amplifier data. This is NOT part of the frozen DSP configuration ABI. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMP_PORTS 2
#define AMP_JSON_MAX 8192
#define AMP_OPS_MAX 96
#define AMP_FUNCTIONS_MAX 7 /* startup, mute, unmute, and four user functions */
#define AMP_BURST_MAX 24
#define AMP_NAME_MAX 24

typedef enum { AMP_WRITE, AMP_UPDATE, AMP_CHECK, AMP_DELAY } amp_op_kind_t;
typedef struct {
    uint8_t kind, reg, count, mask;
    uint16_t delay_ms;
    uint8_t data[AMP_BURST_MAX];
} amp_op_t;
typedef struct { char name[AMP_NAME_MAX]; uint8_t first, count; } amp_function_t;
typedef struct {
    bool enabled;
    uint8_t address, function_count, op_count;
    char model[AMP_NAME_MAX];
    amp_function_t functions[AMP_FUNCTIONS_MAX];
    amp_op_t ops[AMP_OPS_MAX];
} amp_port_profile_t;
typedef struct {
    bool enabled;
    uint8_t sda, scl;
    uint32_t hz;
    amp_port_profile_t ports[AMP_PORTS];
} amp_profile_t;

/* Strict schema; duplicate/unknown keys, fractional integers, unsafe pins,
 * ambiguous addresses and incomplete mute functions are rejected. Output is
 * valid only after success. The caller stages it away from the active worker. */
bool amp_profile_parse(const char *json, size_t len, amp_profile_t *out,
                       char *error, size_t error_size);
bool amp_profile_pin_safe(int pin);
const char *amp_profile_default_json(void);

/* Execute one register operation through a real or test bus. Errors propagate
 * unchanged. Delays are handled by the low-priority core0 caller, never an ISR. */
typedef struct {
    void *context;
    int (*write)(void *, uint8_t reg, const uint8_t *, size_t);
    int (*read)(void *, uint8_t reg, uint8_t *, size_t);
} amp_bus_io_t;
int amp_profile_op(const amp_op_t *op, const amp_bus_io_t *io);
