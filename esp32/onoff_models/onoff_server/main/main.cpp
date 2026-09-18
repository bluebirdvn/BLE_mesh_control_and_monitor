#include <cstring>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_mesh_example_init.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"

#include "opcode.hpp"
#include "vendor_model.h"
#include "board.hpp"
#include "mesh_uuid.h"
static const char *TAG = "ACTUATOR_NODE";

#define ACTUATOR_ID     1          // Đổi mỗi node nếu có nhiều actuator trong mạng

#define MOTION_HOLD_TIME_MS   30000 // Giữ đèn sáng bao lâu sau lần chuyển động cuối (chế độ đèn-theo-người)

#define THRESH_METRIC_MASK    0x03
#define THRESH_METRIC_TEMP    0x00
#define THRESH_METRIC_HUMID   0x01
#define THRESH_METRIC_LUX     0x02
#define THRESH_REQUIRE_MOTION 0x04

static uint16_t s_net_idx = ESP_BLE_MESH_KEY_UNUSED;
static uint16_t s_app_idx = ESP_BLE_MESH_KEY_UNUSED;


static uint8_t dev_uuid[16] = {0};
static uint8_t g_current_onoff = 0xFF;     
static int16_t g_current_setpoint = 0x7FFF;   
static bool    g_is_auto_mode = false;        
static mesh_cmd_threshold_t g_threshold = {0};
static esp_timer_handle_t g_hold_timer = nullptr;

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(actuator_vnd_pub, 20, ROLE_NODE);

static esp_ble_mesh_model_op_t actuator_vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_ACTUATOR_SET, sizeof(mesh_cmd_actuator_set_t)),
    
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_THRESHOLD_SET, sizeof(mesh_cmd_threshold_t)),
    
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_STATUS, sizeof(mesh_evt_sensor_status_t)),
    
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_ACTUATOR, actuator_vnd_op, &actuator_vnd_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count =  ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static bool apply_relay(uint8_t onoff)
{
    bool changed = (onoff != g_current_onoff);
    g_current_onoff = onoff;
    board_actuator_notify_onoff(onoff);
    return changed;
}

static void apply_setpoint(int16_t setpoint)
{
    bool changed = (setpoint != g_current_setpoint);
    g_current_setpoint = setpoint;
    if (changed) {
        board_actuator_notify_setpoint(setpoint);
    }
}

static void publish_actuator_status(void)
{
    if (vnd_models[0].pub == nullptr ||
        vnd_models[0].pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED) {
        return;
    }

    mesh_evt_actuator_status_t status = {};
    status.actuator_id = ACTUATOR_ID;
    status.actuator_type = 0; 
    status.present_setpoint = (uint16_t)g_current_setpoint;
    status.target_setpoint = (uint16_t)g_current_setpoint;
    status.present_onoff = g_current_onoff;
    status.target_onoff = g_current_onoff;
    status.status = 0; 

    esp_err_t err = esp_ble_mesh_model_publish(&vnd_models[0], VND_OP_ACTUATOR_STATUS,
                                               sizeof(status), (uint8_t *)&status, ROLE_NODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Publish Actuator status failed, err=%d", err);
    }
}

static void set_relay_auto(uint8_t onoff)
{
    if (!apply_relay(onoff)) return;
    ESP_LOGI(TAG, "[AUTO] Relay -> %s", onoff ? "ON" : "OFF");
    publish_actuator_status();
}

static void hold_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "Hết thời gian giữ (không có chuyển động/sự kiện mới) -> tắt thiết bị");
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

static void handle_sensor_update(const mesh_evt_sensor_status_t *sensor)
{
    ESP_LOGD(TAG, "Nhận data cảm biến: id=%u temp=%.1f hum=%u%% lux=%u motion=%u (auto_mode=%d)",
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

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    s_net_idx = net_idx;

}


static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                              esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d",
                 param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}




static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (param == NULL) {
        return;
    }

    switch (event) {


    case ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            s_app_idx = param->value.state_change.appkey_add.app_idx;
            break;

        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            s_app_idx = param->value.state_change.mod_app_bind.app_idx;
            break;

        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "SUB ADD from Provisioner!");
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE:
            ESP_LOGI(TAG, "SUB DELETE from Provisioner!");
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "PUB SET from Provisioner!");
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}



extern "C" void vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) return;
    uint32_t opcode = param->model_operation.opcode;

    if (opcode == VND_OP_SENSOR_THRESHOLD_SET) {
        if (param->model_operation.length < sizeof(mesh_cmd_threshold_t)) return;
        memcpy(&g_threshold, param->model_operation.msg, sizeof(g_threshold));
        
        g_is_auto_mode = true;
        stop_hold_timer();
        ESP_LOGI(TAG, "[AUTO MODE ENABLED] Cấu hình mới: type=0x%02x on=%.2f off=%.2f",
                 g_threshold.type, g_threshold.threshold_on, g_threshold.threshold_off);
                 
    }
    
    else if (opcode == VND_OP_SENSOR_STATUS) {
        if (param->model_operation.length < sizeof(mesh_evt_sensor_status_t)) return;
        mesh_evt_sensor_status_t sensor;
        memcpy(&sensor, param->model_operation.msg, sizeof(sensor));
        handle_sensor_update(&sensor);
    }
    
    else if (opcode == VND_OP_ACTUATOR_SET) {
        if (param->model_operation.length < sizeof(mesh_cmd_actuator_set_t)) return;
        
        mesh_cmd_actuator_set_t cmd;
        memcpy(&cmd, param->model_operation.msg, sizeof(cmd));

        if (cmd.actuator_id != ACTUATOR_ID) return; 

        ESP_LOGI(TAG, "[MANUAL MODE] Gateway ra lệnh: onoff=%u, setpoint=%u", cmd.onoff, cmd.setpoint);

        g_is_auto_mode = false;   
        stop_hold_timer();
        
        apply_relay(cmd.onoff);
        apply_setpoint(cmd.setpoint);

        mesh_evt_actuator_status_t status = {};
        status.actuator_id = ACTUATOR_ID;
        status.actuator_type = 0;
        status.present_setpoint = cmd.setpoint;
        status.target_setpoint = cmd.setpoint;
        status.present_onoff = cmd.onoff;
        status.target_onoff = cmd.onoff;
        status.status = 0; 

        esp_ble_mesh_server_model_send_msg(param->model_operation.model, 
                                           param->model_operation.ctx,
                                           VND_OP_ACTUATOR_STATUS, 
                                           sizeof(status), (uint8_t*)&status);
        
        publish_actuator_status();
    }
}

#define USE_FIXED_TEST_UUID 1
#define USE_FIXED_TEST_THRESHOLD 1

static void get_dev_uuid(uint8_t *uuid)
{
    generate(uuid, CLASS_ACTUATOR, TYPE_ACT_AC, HW_VERSION_1_0);
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS default partition initialized successfully");

    err = nvs_flash_init_partition("ble_mesh");
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "BLE Mesh NVS partition needs erase...");
        ESP_ERROR_CHECK(nvs_flash_erase_partition("ble_mesh"));
        err = nvs_flash_init_partition("ble_mesh");
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "BLE Mesh NVS partition initialized successfully");

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

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);

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