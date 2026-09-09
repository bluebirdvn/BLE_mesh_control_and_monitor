#include <cstring>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h" // Lấy MAC Address không phụ thuộc Bluedroid/NimBLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_mesh_example_init.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"   // <-- API chuẩn Generic OnOff / Generic Level

#include "opcode.h"
#include "vendor_model.h"
#include "board.hpp"

static const char *TAG = "ACTUATOR_NODE";

/* =========================================================================
 *  Cấu hình - CHỈNH LẠI cho từng node vật lý
 * ========================================================================= */
#define ACTUATOR_ID     1          // đổi mỗi node nếu có nhiều actuator trong mạng

#define MOTION_HOLD_TIME_MS   30000 // giữ đèn sáng bao lâu sau lần chuyển động cuối, ở chế độ đèn-theo-người

#define THRESH_METRIC_MASK    0x03
#define THRESH_METRIC_TEMP    0x00
#define THRESH_METRIC_HUMID   0x01
#define THRESH_METRIC_LUX     0x02
#define THRESH_REQUIRE_MOTION 0x04

static uint8_t dev_uuid[16] = {0};
static uint8_t g_current_onoff = 0xFF;        // giá trị "chưa xác định" để lần đầu luôn coi là thay đổi
static int16_t g_current_setpoint = 0x7FFF;   // giá trị "chưa xác định" (Generic Level dùng int16_t)
static bool    g_is_auto_mode = false;        // mặc định MANUAL cho tới khi nhận được lệnh cấu hình ngưỡng
static mesh_cmd_threshold_t g_threshold = {0};
static esp_timer_handle_t g_hold_timer = nullptr;

/* =========================================================================
 *  Composition data BLE Mesh
 * ========================================================================= */
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/* ---- Generic OnOff Server chuẩn: điều khiển relay ON/OFF ---- */
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // tự xử lý để giữ đúng logic auto/manual + publish status
    },
};

/* ---- Generic Level Server chuẩn: dùng thay cho "setpoint" ---- */
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

/* ---- Vendor model: chỉ còn dùng cho cấu hình ngưỡng cảm biến + trạng thái sensor,
 *      vì BLE Mesh chuẩn không có model generic phù hợp cho dữ liệu này. ---- */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_vnd_pub, 20, ROLE_NODE);

static esp_ble_mesh_model_op_t sensor_vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_THRESHOLD_SET, sizeof(mesh_cmd_threshold_t)),
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_STATUS, sizeof(mesh_evt_sensor_status_t)),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub, &level_server),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_SENSOR, sensor_vnd_op, &sensor_vnd_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = 1,
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,
};

/* =========================================================================
 *  Điều khiển relay + báo trạng thái (dùng Generic OnOff Status chuẩn)
 * ========================================================================= */
static bool apply_relay(uint8_t onoff)
{
    bool changed = (onoff != g_current_onoff);
    g_current_onoff = onoff;
    onoff_server.state.onoff = onoff;      // đồng bộ state model chuẩn
    board_actuator_notify_onoff(onoff);
    return changed;
}

static void publish_onoff_status(void)
{
    if (root_models[1].pub == nullptr ||
        root_models[1].pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
        return;
    }
    esp_err_t err = esp_ble_mesh_model_publish(&root_models[1], ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
                                                sizeof(g_current_onoff), &g_current_onoff, ROLE_NODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Publish Generic OnOff status failed, err=%d", err);
    }
}

static void apply_setpoint(int16_t setpoint)
{
    bool changed = (setpoint != g_current_setpoint);
    g_current_setpoint = setpoint;
    level_server.state.level = setpoint;   // đồng bộ state model chuẩn
    if (changed) {
        board_actuator_notify_setpoint(setpoint);
    }
}

static void publish_level_status(void)
{
    if (root_models[2].pub == nullptr ||
        root_models[2].pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
        return;
    }
    esp_err_t err = esp_ble_mesh_model_publish(&root_models[2], ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,
                                                sizeof(g_current_setpoint), &g_current_setpoint, ROLE_NODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Publish Generic Level status failed, err=%d", err);
    }
}

static void set_relay_auto(uint8_t onoff)
{
    if (!apply_relay(onoff)) return;
    ESP_LOGI(TAG, "[AUTO] Relay -> %s", onoff ? "ON" : "OFF");
    publish_onoff_status();
}

/* =========================================================================
 *  Hold timer cho chế độ "đèn theo người"
 * ========================================================================= */
static void hold_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "Hết thời gian giữ (không có chuyển động mới) -> tắt đèn");
    set_relay_auto(0);
}

static void restart_hold_timer(void)
{
    if (!g_hold_timer) return;
    esp_timer_stop(g_hold_timer);
    esp_timer_start_once(g_hold_timer, (uint64_t)MOTION_HOLD_TIME_MS * 1000);
}

static void stop_hold_timer(void)
{
    if (!g_hold_timer) return;
    esp_timer_stop(g_hold_timer);
}

/* =========================================================================
 *  Logic ra quyết định khi nhận được 1 gói sensor status (giữ nguyên 100%)
 * ========================================================================= */
static void handle_sensor_update(const mesh_evt_sensor_status_t *sensor)
{
    ESP_LOGD(TAG, "Nhận sensor id=%u temp=%.1f hum=%u%% lux=%u motion=%u (auto_mode=%d)",
             sensor->sensor_id, sensor->temperature / 10.0f, sensor->humidity,
             sensor->lux, sensor->motion, g_is_auto_mode);

    if (!g_is_auto_mode) {
        ESP_LOGD(TAG, "Đang MANUAL mode -> bỏ qua, chờ cấu hình ngưỡng để chuyển sang AUTO");
        return;
    }

    float metric;
    switch (g_threshold.type & THRESH_METRIC_MASK) {
        case THRESH_METRIC_HUMID: metric = sensor->humidity; break;
        case THRESH_METRIC_LUX:   metric = sensor->lux; break;
        default:                  metric = sensor->temperature / 10.0f; break;
    }

    bool require_motion = (g_threshold.type & THRESH_REQUIRE_MOTION) != 0;

    if (require_motion) {
        if (metric >= g_threshold.threshold_off) {
            stop_hold_timer();
            set_relay_auto(0);
        } else if (sensor->motion && metric < g_threshold.threshold_on) {
            set_relay_auto(1);
            restart_hold_timer();
        }
    } else {
        if (g_threshold.threshold_on > g_threshold.threshold_off) {
            if (metric >= g_threshold.threshold_on) set_relay_auto(1);
            else if (metric <= g_threshold.threshold_off) set_relay_auto(0);
        } else {
            if (metric <= g_threshold.threshold_on) set_relay_auto(1);
            else if (metric >= g_threshold.threshold_off) set_relay_auto(0);
        }
    }
}

/* =========================================================================
 *  Generic Server callback (Generic OnOff + Generic Level) - API chuẩn
 * ========================================================================= */
static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                                esp_ble_mesh_generic_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Generic server event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
             event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(g_current_onoff), &g_current_onoff);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
            esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(g_current_setpoint), &g_current_setpoint);
        }
        break;

    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {

            uint8_t onoff = param->value.set.onoff.onoff;

            g_is_auto_mode = false;   // lệnh điều khiển trực tiếp -> chuyển sang MANUAL, giống VND_OP_ACTUATOR_SET cũ
            stop_hold_timer();
            apply_relay(onoff);

            ESP_LOGI(TAG, "[MANUAL] Set onoff=%u", onoff);

            if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
                esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                    ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(g_current_onoff), &g_current_onoff);
            }
            publish_onoff_status();

        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
                   param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {

            int16_t setpoint = param->value.set.level.level;

            g_is_auto_mode = false;
            apply_setpoint(setpoint);

            ESP_LOGI(TAG, "[MANUAL] Set setpoint=%d", setpoint);

            if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET) {
                esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                    ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(g_current_setpoint), &g_current_setpoint);
            }
            publish_level_status();
        }
        break;

    default:
        break;
    }
}

/* =========================================================================
 *  Vendor model callback - chỉ còn cấu hình ngưỡng + dữ liệu sensor
 * ========================================================================= */
static void sensor_vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) return;
    uint32_t opcode = param->model_operation.opcode;

    if (opcode == VND_OP_SENSOR_THRESHOLD_SET) {
        if (param->model_operation.length < sizeof(mesh_cmd_threshold_t)) return;
        memcpy(&g_threshold, param->model_operation.msg, sizeof(g_threshold));
        g_is_auto_mode = true;
        stop_hold_timer();
        ESP_LOGI(TAG, "[AUTO] type=0x%02x on=%.2f off=%.2f",
                 g_threshold.type, g_threshold.threshold_on, g_threshold.threshold_off);
    }
    else if (opcode == VND_OP_SENSOR_STATUS) {
        if (param->model_operation.length < sizeof(mesh_evt_sensor_status_t)) return;
        mesh_evt_sensor_status_t sensor;
        memcpy(&sensor, param->model_operation.msg, sizeof(sensor));
        handle_sensor_update(&sensor);
    }
}

extern "C" void vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    sensor_vendor_model_cb(event, param);
}

/* =========================================================================
 *  Bootstrap
 * ========================================================================= */
#define USE_FIXED_TEST_UUID 1
#define USE_FIXED_TEST_THRESHOLD 1

static void get_dev_uuid(uint8_t *uuid)
{
#if USE_FIXED_TEST_UUID
    static const uint8_t fixed[16] = {
        0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,
        0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0x01,
    };
    memcpy(uuid, fixed, 16);
#else
    memset(uuid, 0, 16);
    esp_read_mac(uuid, ESP_MAC_BT);
    uuid[6] = 'A'; uuid[7] = 'C'; uuid[8] = 'T'; uuid[9] = ACTUATOR_ID;
#endif
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(bluetooth_init());

    get_dev_uuid(dev_uuid);
    ESP_LOG_BUFFER_HEX(TAG, dev_uuid, sizeof(dev_uuid));

    board_actuator_init();
    apply_relay(0);
    apply_setpoint(0);

    const esp_timer_create_args_t hold_timer_args = {
        .callback = &hold_timer_cb,
        .arg = nullptr,
        .name = "actuator_hold_timer",
    };
    esp_timer_create(&hold_timer_args, &g_hold_timer);

    /* Đăng ký callback chuẩn cho Generic OnOff / Generic Level */
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);
    /* Vendor model callback chỉ còn phục vụ sensor threshold + sensor status */
    ESP_ERROR_CHECK(esp_ble_mesh_register_custom_model_callback(vendor_model_cb));

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_mesh_init failed %d", err);
        return;
    }

#if USE_FIXED_TEST_THRESHOLD
    g_threshold.actuator_id = ACTUATOR_ID;
    g_threshold.type = THRESH_METRIC_LUX | THRESH_REQUIRE_MOTION;
    g_threshold.threshold_on = 50.0f;
    g_threshold.threshold_off = 200.0f;
    g_is_auto_mode = true;
    ESP_LOGW(TAG, "USE_FIXED_TEST_THRESHOLD đang BẬT - AUTO mode sẵn, không cần chờ cấu hình ngưỡng");
#endif

    esp_ble_mesh_node_prov_enable(static_cast<esp_ble_mesh_prov_bearer_t>(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    ESP_LOGI(TAG, "Actuator node ready, waiting to be provisioned");
}