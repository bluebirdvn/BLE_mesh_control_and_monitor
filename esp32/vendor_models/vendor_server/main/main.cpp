

#include <cstdio>
#include <cstring>
#include <cinttypes>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/i2c_master.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#include "vendor_model.h"
#include "sensor_manager.hpp"
#include "mesh_uuid.h"
static const char *TAG = "SENSOR_NODE";

#define CID_ESP 0x02E5
#define SENSOR_ID 1
#define SENSOR_PERIODIC_INTERVAL_MS 3000


#define I2C_MASTER_FREQ_HZ 400000

static uint16_t pro_addr = ESP_BLE_MESH_ADDR_UNASSIGNED;
static uint8_t dev_uuid[16] = {0};

static TaskHandle_t sensor_task_handle = nullptr;
static SensorManager *sensor_manager = nullptr;

static uint16_t s_net_idx = ESP_BLE_MESH_KEY_UNUSED;
static uint16_t s_app_idx = ESP_BLE_MESH_KEY_UNUSED;

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);

static esp_ble_mesh_model_op_t sensor_srv_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_GET, 0),
    
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_SENSOR, sensor_srv_op, &sensor_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

/**
 * @brief Send a SENSOR_STATUS message with the given payload to dst_addr.
 */
static esp_err_t send_sensor_status(uint16_t dst_addr, const sensor_data_t &data)
{
    if (s_app_idx == ESP_BLE_MESH_KEY_UNUSED)
    {
        ESP_LOGW(TAG, "AppKey not bound yet, cannot send sensor status");
        return ESP_ERR_INVALID_STATE;
    }

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx = s_net_idx,
        .app_idx = s_app_idx,
        .addr = dst_addr,
        .send_ttl = ESP_BLE_MESH_TTL_DEFAULT,
    };

    return esp_ble_mesh_server_model_send_msg(&vnd_models[0], &ctx, VND_OP_SENSOR_STATUS,
                                               sizeof(data), (uint8_t *)&data);
}

/**
 * @brief Periodic task: reads all sensors and reports the data.
 *
 * - If the model's publish address is not configured yet, the data is sent
 *   directly to the provisioner instead, so the node still reports in.
 * - Once a publish (group) address has been configured, data is sent there
 *   on every cycle.
 */
static void sensor_task(void *arg)
{
    SensorManager *manager = static_cast<SensorManager *>(arg);

    while (true)
    {
        sensor_data_t data = {};

        if (!manager->readAll(data))
        {
            ESP_LOGE(TAG, "Sensor read failed, skipping this cycle");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIODIC_INTERVAL_MS));
            continue;
        }

        uint16_t publish_addr = sensor_pub.publish_addr;

        if (publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED)
        {
            ESP_LOGI(TAG, "Publish not configured yet, reporting to provisioner");
            send_sensor_status(pro_addr, data);
        }
        else
        {
            ESP_LOGI(TAG, "Publishing sensor data to group 0x%04x", publish_addr);
            send_sensor_status(publish_addr, data);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIODIC_INTERVAL_MS));
    }
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
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

    if(pro_addr  == ESP_BLE_MESH_ADDR_UNASSIGNED && param->ctx.addr != ESP_BLE_MESH_ADDR_UNASSIGNED) {
        pro_addr = param->ctx.addr;
        ESP_LOGI(TAG, "catch provisioner addr");
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

        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET, pub_addr 0x%04x",
                     param->value.state_change.mod_pub_set.pub_addr);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "SUB ADD from Provisioner!");
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE:
            ESP_LOGI(TAG, "SUB DELETE from Provisioner!");
            break;
        default:
            break;

        }

    default:
        break;
    }
}


static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                              esp_ble_mesh_model_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == VND_OP_SENSOR_GET)
        {
            board_led_signal_rx();

            sensor_data_t data = {};

            if (sensor_manager == nullptr || !sensor_manager->readAll(data))
            {
                ESP_LOGE(TAG, "Failed to read sensors for SENSOR_GET request");
                break;
            }

            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], param->model_operation.ctx,
                                                                VND_OP_SENSOR_STATUS, sizeof(data), (uint8_t *)&data);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to send message 0x%06x", VND_OP_SENSOR_STATUS);
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code)
        {
            ESP_LOGE(TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        board_led_signal_tx();
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    esp_err_t err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");
    return ESP_OK;
}

/**
 * @brief Create the I2C bus shared by both sensors.
 */
static i2c_master_bus_handle_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_IO,
        .scl_io_num = BOARD_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };

    i2c_master_bus_handle_t bus_handle = nullptr;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return nullptr;
    }

    return bus_handle;
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing...");

    generate(dev_uuid, CLASS_SENSOR, TYPE_SENS_COMBO, HW_VERSION_1_0);

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

    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    // ble_mesh_get_dev_uuid(dev_uuid);

    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
        return;
    }

    i2c_master_bus_handle_t i2c_bus = i2c_bus_init();
    if (i2c_bus == nullptr)
    {
        ESP_LOGE(TAG, "I2C bus init failed, sensor task not started");
        return;
    }

    static SensorManager manager(i2c_bus, SENSOR_ID);
    if (!manager.init())
    {
        ESP_LOGE(TAG, "Sensor init failed, sensor task not started");
        return;
    }
    sensor_manager = &manager;

    xTaskCreate(sensor_task, "sensor_task", 4096, sensor_manager, 5, &sensor_task_handle);
}