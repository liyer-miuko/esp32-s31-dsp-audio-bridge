/* Core0 BLE control transport, not a second audio path.
 * FF01 accepts AA55 bytes, FF02 notifies replies, 2902 enables notifications.
 * BTU callbacks only validate/copy bounded writes. The protocol task owns all
 * command execution. No logging flood, flash access or waiting from BTU. */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "ble_tuning.h"
#include "serial_proto.h"

#define APP_ID 0x00C0
#define SVC_UUID 0xFF00
#define RX_UUID 0xFF01
#define TX_UUID 0xFF02
#define RX_MAX 512
#define LOCAL_MTU 247
/* One AA55 frame may need several controller-credit windows. The bounded
 * protocol-task wait yields; BTU/audio tasks never wait for this sender. */
#define TX_WAIT_US 500000
static const char *TAG = "ble_tuning";
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    esp_gatt_if_t iface;
    uint16_t conn, mtu, tx;
    bool connected, subscribed, congested;
    uint32_t generation, dropped;
} link = {.iface = ESP_GATT_IF_NONE, .mtu = 23};
/* Setup handles are BTU-owned after init; only link crosses tasks. */
static uint16_t service, rx, cccd;
static bool initialized, advertising_ready;
static uint8_t initial, cccd_initial[2], adv[31];
static esp_attr_control_t app_response = {.auto_rsp = ESP_GATT_RSP_BY_APP};
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x00A0, .adv_int_max = 0x0140, .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC, .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL, .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static bool checked(esp_err_t err, const char *operation)
{
    if (err == ESP_OK) return true;
    ESP_LOGE(TAG, "%s failed: %s (Classic/USB continue)", operation, esp_err_to_name(err));
    return false;
}
static void advertise(void)
{
    if (advertising_ready) checked(esp_ble_gap_start_advertising(&adv_params), "advertise");
}
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *p)
{
    if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT) {
        advertising_ready = p->adv_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS;
        if (advertising_ready) advertise();
        else ESP_LOGE(TAG, "advertising data failed: %d", p->adv_data_raw_cmpl.status);
    } else if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT &&
               p->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "advertising start failed: %d", p->adv_start_cmpl.status);
    }
}
static void create_char(uint16_t uuid, bool input)
{
    esp_bt_uuid_t id = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = uuid};
    esp_attr_value_t value = {.attr_max_len = input ? RX_MAX : LOCAL_MTU,
                            .attr_len = 1, .attr_value = &initial};
    checked(esp_ble_gatts_add_char(service, &id, input ? ESP_GATT_PERM_WRITE : ESP_GATT_PERM_READ,
        input ? ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR : ESP_GATT_CHAR_PROP_BIT_NOTIFY,
        &value, &app_response), "add characteristic");
}
static void write_cb(esp_gatt_if_t iface, esp_ble_gatts_cb_param_t *p)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    portENTER_CRITICAL(&mux);
    const bool current = link.connected && link.conn == p->write.conn_id;
    const bool subscribed = link.subscribed;
    portEXIT_CRITICAL(&mux);
    if (!current) status = ESP_GATT_ERROR;
    else if (p->write.is_prep) status = ESP_GATT_REQ_NOT_SUPPORTED;
    else if (p->write.offset) status = ESP_GATT_INVALID_OFFSET;
    else if (p->write.handle == cccd) {
        if (p->write.len != 2 || p->write.value[1] || p->write.value[0] > 1) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        } else {
            const bool enabled = p->write.value[0] == 1;
            portENTER_CRITICAL(&mux);
            link.subscribed = enabled; ++link.generation;
            portEXIT_CRITICAL(&mux);
            proto_ble_link_changed(enabled); /* subscription resets partial requests */
        }
    } else if (p->write.handle == rx) {
        if (!subscribed) status = ESP_GATT_WRITE_NOT_PERMIT;
        else if (!p->write.len || p->write.len > RX_MAX) status = ESP_GATT_INVALID_ATTR_LEN;
        else if (!proto_feed_bytes(p->write.value, p->write.len)) status = ESP_GATT_NO_RESOURCES;
    } else status = ESP_GATT_INVALID_HANDLE;
    if (p->write.need_rsp)
        checked(esp_ble_gatts_send_response(iface, p->write.conn_id, p->write.trans_id, status, NULL), "write response");
}
static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t iface, esp_ble_gatts_cb_param_t *p)
{
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        if (p->reg.status != ESP_GATT_OK) { ESP_LOGE(TAG, "register status=%d", p->reg.status); break; }
        portENTER_CRITICAL(&mux); link.iface = iface; portEXIT_CRITICAL(&mux);
        if (!checked(esp_ble_gatt_set_local_mtu(LOCAL_MTU), "local MTU")) break;
        esp_gatt_srvc_id_t id = {.is_primary = true, .id.inst_id = 0,
                                .id.uuid.len = ESP_UUID_LEN_16, .id.uuid.uuid.uuid16 = SVC_UUID};
        checked(esp_ble_gatts_create_service(iface, &id, 8), "create service");
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        if (p->create.status != ESP_GATT_OK) { ESP_LOGE(TAG, "create status=%d", p->create.status); break; }
        service = p->create.service_handle; create_char(RX_UUID, true); break;
    case ESP_GATTS_ADD_CHAR_EVT:
        if (p->add_char.status != ESP_GATT_OK) { ESP_LOGE(TAG, "char status=%d", p->add_char.status); break; }
        if (p->add_char.char_uuid.uuid.uuid16 == RX_UUID) {
            rx = p->add_char.attr_handle; create_char(TX_UUID, false);
        } else if (p->add_char.char_uuid.uuid.uuid16 == TX_UUID) {
            portENTER_CRITICAL(&mux); link.tx = p->add_char.attr_handle; portEXIT_CRITICAL(&mux);
            esp_bt_uuid_t id = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG};
            esp_attr_value_t value = {.attr_max_len = 2, .attr_len = 2, .attr_value = cccd_initial};
            checked(esp_ble_gatts_add_char_descr(service, &id, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                               &value, &app_response), "add CCCD");
        }
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (p->add_char_descr.status != ESP_GATT_OK) { ESP_LOGE(TAG, "descriptor status=%d", p->add_char_descr.status); break; }
        cccd = p->add_char_descr.attr_handle;
        checked(esp_ble_gatts_start_service(service), "start service"); break;
    case ESP_GATTS_START_EVT: {
        if (p->start.status != ESP_GATT_OK) { ESP_LOGE(TAG, "start status=%d", p->start.status); break; }
        /* This fork rejects a structured 16-bit service UUID list; use raw AD. */
        const uint8_t prefix[] = {2, 1, 2, 3, 3, 0, 0xFF};
        memcpy(adv, prefix, sizeof(prefix));
        const size_t full = strlen(CONFIG_BT_A2DP_DEVICE_NAME), n = full > 22 ? 22 : full;
        adv[7] = (uint8_t)(1 + n); adv[8] = full > n ? 0x08 : 0x09;
        memcpy(adv + 9, CONFIG_BT_A2DP_DEVICE_NAME, n);
        checked(esp_ble_gap_config_adv_data_raw(adv, (uint32_t)(9 + n)), "advertising data");
        ESP_LOGI(TAG, "service ready FF00 RX=FF01 TX=FF02 mtu_max=%u", LOCAL_MTU);
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        portENTER_CRITICAL(&mux);
        const bool occupied = link.connected;
        if (!occupied) {
            link.conn = p->connect.conn_id; link.connected = true;
            link.subscribed = false; link.congested = false; link.mtu = 23; ++link.generation;
        }
        portEXIT_CRITICAL(&mux);
        if (occupied) checked(esp_ble_gatts_close(iface, p->connect.conn_id), "reject second BLE host");
        else proto_ble_link_changed(false); /* commands require CCCD first */
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        portENTER_CRITICAL(&mux);
        const bool current = link.connected && link.conn == p->disconnect.conn_id;
        if (current) {
            link.connected = false; link.subscribed = false; link.congested = false; ++link.generation;
        }
        portEXIT_CRITICAL(&mux);
        if (current) { proto_ble_link_changed(false); advertise(); }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        portENTER_CRITICAL(&mux);
        if (link.connected && link.conn == p->mtu.conn_id)
            link.mtu = p->mtu.mtu < 23 ? 23 : (p->mtu.mtu > LOCAL_MTU ? LOCAL_MTU : p->mtu.mtu);
        portEXIT_CRITICAL(&mux); break;
    case ESP_GATTS_CONGEST_EVT:
        portENTER_CRITICAL(&mux);
        if (link.connected && link.conn == p->congest.conn_id) link.congested = p->congest.congested;
        portEXIT_CRITICAL(&mux); break;
    case ESP_GATTS_WRITE_EVT: write_cb(iface, p); break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        checked(esp_ble_gatts_send_response(iface, p->exec_write.conn_id, p->exec_write.trans_id,
                                           ESP_GATT_REQ_NOT_SUPPORTED, NULL), "execute write response"); break;
    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t response = {0};
        response.attr_value.handle = p->read.handle;
        portENTER_CRITICAL(&mux);
        const bool current = link.connected && link.conn == p->read.conn_id;
        response.attr_value.value[0] = link.subscribed ? 1 : 0;
        portEXIT_CRITICAL(&mux);
        const esp_gatt_status_t status = !current ? ESP_GATT_ERROR :
            p->read.offset ? ESP_GATT_INVALID_OFFSET : p->read.handle == cccd ? ESP_GATT_OK : ESP_GATT_READ_NOT_PERMIT;
        response.attr_value.len = status == ESP_GATT_OK ? 2 : 0;
        checked(esp_ble_gatts_send_response(iface, p->read.conn_id, p->read.trans_id, status,
                                           status == ESP_GATT_OK ? &response : NULL), "read response"); break;
    }
    default: break;
    }
}
esp_err_t ble_tuning_init(void)
{
    if (initialized) return ESP_OK;
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) return ESP_ERR_INVALID_STATE;
    esp_err_t err = esp_ble_gap_set_device_name(CONFIG_BT_A2DP_DEVICE_NAME);
    if (err != ESP_OK) return err;
    if ((err = esp_ble_gatts_register_callback(gatts_cb)) != ESP_OK) return err;
    if ((err = esp_ble_gap_register_callback(gap_cb)) != ESP_OK) return err;
    if ((err = esp_ble_gatts_app_register(APP_ID)) != ESP_OK) return err;
    initialized = true; /* async service-ready is reported separately */
    return ESP_OK;
}
bool ble_tuning_send(const uint8_t *data, size_t len, uint32_t session_generation)
{
    if (!data || !len) return false;
    const int64_t deadline = esp_timer_get_time() + TX_WAIT_US;
    portENTER_CRITICAL(&mux); const uint32_t generation = link.generation; portEXIT_CRITICAL(&mux);
    size_t off = 0;
    while (off < len) {
        portENTER_CRITICAL(&mux);
        const bool current = link.connected && link.subscribed && link.generation == generation;
        const bool congested = link.congested;
        const uint16_t mtu = link.mtu, conn = link.conn, tx = link.tx;
        const esp_gatt_if_t iface = link.iface;
        portEXIT_CRITICAL(&mux);
        if (!current || !proto_ble_generation_matches(session_generation)) break;
        if (congested || esp_ble_get_cur_sendable_packets_num(conn) == 0) {
            if (esp_timer_get_time() >= deadline) break;
            vTaskDelay(1); /* only proto_task waits, never BTU or the pump */
            continue;
        }
        const uint16_t n = (uint16_t)(len - off > mtu - 3u ? mtu - 3u : len - off);
        const esp_err_t result = esp_ble_gatts_send_indicate(
            iface, conn, tx, n, (uint8_t *)(data + off), false);
        if (result == ESP_OK) {
            off += n;
            continue;
        }
        /* L2CAP may reject enqueue before CONGEST_EVT reaches BTU. ESP_FAIL
         * and allocation failure queued nothing: retry this same fragment,
         * never a fragment already accepted. Keep the original frame deadline,
         * generation fence and bounded proto-task yield; audio tasks do not wait. */
        if ((result != ESP_FAIL && result != ESP_ERR_NO_MEM) ||
            esp_timer_get_time() >= deadline) break;
        vTaskDelay(1);
    }
    if (off == len) return true; /* queued, not an over-air delivery receipt */
    portENTER_CRITICAL(&mux); link.dropped += (uint32_t)(len - off); portEXIT_CRITICAL(&mux);
    return false;
}
bool ble_tuning_is_connected(void)
{
    portENTER_CRITICAL(&mux); const bool result = link.connected; portEXIT_CRITICAL(&mux); return result;
}
uint32_t ble_tuning_dropped_bytes(void)
{
    portENTER_CRITICAL(&mux); const uint32_t result = link.dropped; portEXIT_CRITICAL(&mux); return result;
}
