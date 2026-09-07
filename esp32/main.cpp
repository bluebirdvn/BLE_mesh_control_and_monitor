#include <cstring>
#include <memory>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "provisioner.hpp"
#include "mesh_config.hpp"
#include "mesh_command_sender.hpp"
#include "mesh_event_dispatcher.hpp"
#include "mesh_frame.hpp"
#include "vendor_model.h"

#include "transport/serial_port.hpp"
#include "reliable_transport.hpp"
#include "frame_codec.hpp"
#include "raw_frame.hpp"  
#include "config.h"
static const char *TAG = "MAIN";

extern "C" void prov_cb(esp_ble_mesh_prov_cb_event_t, esp_ble_mesh_prov_cb_param_t*);
extern "C" void config_client_cb(esp_ble_mesh_cfg_client_cb_event_t, esp_ble_mesh_cfg_client_cb_param_t*);
extern "C" void generic_client_cb(esp_ble_mesh_generic_client_cb_event_t, esp_ble_mesh_generic_client_cb_param_t*);
extern "C" void rpr_client_cb(esp_ble_mesh_rpr_client_cb_event_t, esp_ble_mesh_rpr_client_cb_param_t*);
extern "C" void vendor_model_cb(esp_ble_mesh_model_cb_event_t, esp_ble_mesh_model_cb_param_t*);

static uint8_t test_net_key[16] = {
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
    0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
};
static uint8_t test_app_key[16] = {
    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
    0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,
};

static uint8_t dev_uuid[16] = {0};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[8];
static esp_ble_mesh_elem_t  elements[1];
static esp_ble_mesh_comp_t  composition;

static esp_ble_mesh_prov_t provision = {
    .prov_uuid           = dev_uuid,
    .prov_unicast_addr   = PROV_OWN_ADDR,
    .prov_start_address  = PROV_START_ADDRESS,
};

static esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(TAG, "bt_controller_init failed %d", ret); return ret; }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(TAG, "bt_controller_enable failed %d", ret); return ret; }

    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(TAG, "bluedroid_init failed %d", ret); return ret; }

    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(TAG, "bluedroid_enable failed %d", ret); return ret; }

    return ESP_OK;
}

static void get_dev_uuid(uint8_t *uuid)
{
    const uint8_t *mac = esp_bt_dev_get_address();
    memset(uuid, 0, 16);
    if (mac) memcpy(uuid, mac, 6);
}

static esp_err_t ble_mesh_init(std::shared_ptr<MeshCommandSender> sender, std::shared_ptr<MeshEventDispatcher> dispatcher)
{
    esp_err_t err;

    err = esp_ble_mesh_register_prov_callback(prov_cb);
    if (err) { ESP_LOGE(TAG, "register_prov_callback failed %d", err); return err; }

    err = esp_ble_mesh_register_config_client_callback(config_client_cb);
    if (err) { ESP_LOGE(TAG, "register_config_client_callback failed %d", err); return err; }

    err = esp_ble_mesh_register_generic_client_callback(generic_client_cb);
    if (err) { ESP_LOGE(TAG, "register_generic_client_callback failed %d", err); return err; }

    err = esp_ble_mesh_register_custom_model_callback(vendor_model_cb);
    if (err) { ESP_LOGE(TAG, "register_custom_model_callback failed %d", err); return err; }

    err = esp_ble_mesh_register_rpr_client_callback(rpr_client_cb);
    if (err) { ESP_LOGE(TAG, "register_rpr_client_callback failed %d", err); return err; }

    size_t n_local = 0;
    esp_ble_mesh_model_t *local = Provisioner::getInstance().get_local_models(n_local);
    if (n_local + 1 > sizeof(root_models) / sizeof(root_models[0])) {
        ESP_LOGE(TAG, "root_models buffer too small: need %zu", n_local + 1);
        return ESP_ERR_NO_MEM;
    }
    root_models[0] = ESP_BLE_MESH_MODEL_CFG_SRV(&config_server);
    for (size_t i = 0; i < n_local; i++) {
        root_models[1 + i] = local[i];
    }

    elements[0] = ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE);

    composition.cid = CID_ESP;
    composition.element_count = 1;
    composition.elements = elements;

    err = esp_ble_mesh_init(&provision, &composition);
    if (err) { ESP_LOGE(TAG, "esp_ble_mesh_init failed %d", err); return err; }

    err = esp_ble_mesh_provisioner_add_local_net_key(test_net_key, ESP_BLE_MESH_NET_PRIMARY);
    if (err) { ESP_LOGE(TAG, "add_local_net_key failed %d", err); return err; }

    err = esp_ble_mesh_provisioner_add_local_app_key(test_app_key, ESP_BLE_MESH_NET_PRIMARY, ESP_BLE_MESH_APP_KEY_UNUSED);
    if (err) { ESP_LOGE(TAG, "add_local_app_key failed %d", err); return err; }

    Provisioner::getInstance().init(sender, dispatcher);
    Provisioner::getInstance().start_periodic_rpr_scan(30000);

    ESP_LOGI(TAG, "ble_mesh_init done, waiting for CMD_PROV_ENABLE from gateway");
    return ESP_OK;
}

static std::shared_ptr<SerialPort>           g_serial_port;
static std::shared_ptr<ReliableTransport>    g_transport;
static std::shared_ptr<FrameCodec>           g_codec;
static std::shared_ptr<MeshEventDispatcher>  g_dispatcher;

static void on_gateway_frame(const RawFrame& raw)
{
    MeshFrame mf;
    mf.opcode  = static_cast<OpCode>(raw.opcode);
    mf.addr    = raw.addr;
    mf.type    = raw.type;
    mf.payload = raw.payload;
    g_dispatcher->dispatch(mf);
}

extern "C" void app_main(void)
{
    esp_err_t err;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "bluetooth_init failed %d", err);
        return;
    }

    get_dev_uuid(dev_uuid);

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };


    g_serial_port = std::make_shared<SerialPort>(uart_config);
    g_dispatcher  = std::make_shared<MeshEventDispatcher>();
    g_transport   = std::make_shared<ReliableTransport>(g_serial_port);
    g_transport->on_frame_cb = on_gateway_frame;

    communication_status_t tstart = g_transport->start();
    if (tstart != COMMUNICATION_SUCCESS) {
        ESP_LOGE(TAG, "ReliableTransport start() failed, status=%d", static_cast<int>(tstart));
        return;
    }

    g_codec = std::make_shared<FrameCodec>();
    auto sender = std::make_shared<MeshCommandSender>(g_transport, g_codec);

    err = ble_mesh_init(sender, g_dispatcher);
    if (err) {
        ESP_LOGE(TAG, "ble_mesh_init failed %d", err);
        return;
    }
}