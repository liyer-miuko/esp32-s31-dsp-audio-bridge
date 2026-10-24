/* USB stays on core0. TinyUSB already permits levels 1/2/3; select level 2
 * for this one source to test reception latency without adding load to DSP.
 * Other allocations retain their exact flags, handler, argument, and result.
 */
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "soc/interrupts.h"

esp_err_t __real_esp_intr_alloc(int source, int flags, intr_handler_t handler,
                                void *arg, intr_handle_t *ret_handle);

esp_err_t __wrap_esp_intr_alloc(int source, int flags, intr_handler_t handler,
                                void *arg, intr_handle_t *ret_handle)
{
    const bool usb = source == ETS_USB_OTGHS_INTR_SOURCE &&
                     (flags & ESP_INTR_FLAG_LEVELMASK) == ESP_INTR_FLAG_LOWMED;
    if (usb) flags = (flags & ~ESP_INTR_FLAG_LEVELMASK) | ESP_INTR_FLAG_LEVEL2;
    const esp_err_t err = __real_esp_intr_alloc(source, flags, handler, arg, ret_handle);
    if (usb) ESP_LOGI("uac_usb_intr", "core0 USB level 2 allocation: %s", esp_err_to_name(err));
    return err;
}
