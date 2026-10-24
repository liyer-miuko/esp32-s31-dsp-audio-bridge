#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define MSG_AMP_CONTROL 0xB8
/* Initialize after NVS, before serial. Start only after both I2S clocks exist.
 * Disabled defaults allocate no bus and perform no register transactions. */
esp_err_t amp_control_init(void);
esp_err_t amp_control_start(void);
void amp_control_abort_upload(void);
/* Called only by proto_task (UART and BLE share that same dispatcher).
 * The returned payload is <=255 bytes, including explicit operation status.
 * Commit ACK means queued. STATUS distinguishes requested/applied/error. */
size_t amp_control_request(const uint8_t *p, size_t len, uint64_t session,
                           uint8_t reply[255]);
