#ifndef PROVISIONER_HPP
#define PROVISIONER_HPP

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_rpr_model_api.h"   // API RPR client (kiểm tra đúng tên header theo bản ESP-IDF đang dùng)
#include "esp_timer.h"

#include "mesh_command_sender.hpp"
#include "mesh_event_dispatcher.hpp"
#include "device_management.hpp"
#include "mesh_frame.hpp"
#include "vendor_model.h"

#include <memory>
#include <vector>
#include <array>
#include <cstdint>

class Provisioner {
public:
    static Provisioner& getInstance() {
        static Provisioner instance;
        return instance;
    }
    void init(std::shared_ptr<MeshCommandSender> sender_ptr, std::shared_ptr<MeshEventDispatcher> dispatcher_ptr);

    esp_ble_mesh_model_t* get_local_models(size_t& count);

    void start_periodic_rpr_scan(uint32_t period_ms = 30000);
    void stop_periodic_rpr_scan();

    void add_uuid_to_whitelist(const uint8_t uuid[16]);

    void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t* param);
    void ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event, esp_ble_mesh_cfg_client_cb_param_t* param);
    void ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event, esp_ble_mesh_generic_client_cb_param_t* param);
    void ble_mesh_rpr_client_cb(esp_ble_mesh_rpr_client_cb_event_t event, esp_ble_mesh_rpr_client_cb_param_t* param);
    void ble_mesh_vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t* param);

private:
    Provisioner() = default;
    ~Provisioner() = default;
    Provisioner(const Provisioner&) = delete;
    Provisioner& operator=(const Provisioner&) = delete;

    void handle_cmd_prov_enable(const MeshFrame& f);
    void handle_cmd_prov_disable(const MeshFrame& f);
    void handle_cmd_add_unprov_dev(const MeshFrame& f);
    void handle_cmd_set_uuid_match(const MeshFrame& f);
    void handle_cmd_delete_node(const MeshFrame& f);
    void handle_cmd_group_add(const MeshFrame& f);
    void handle_cmd_group_delete(const MeshFrame& f);
    void handle_cmd_model_pub_set(const MeshFrame& f);
    void handle_cmd_sensor_get(const MeshFrame& f);
    void handle_cmd_actuator_set(const MeshFrame& f);
    void handle_cmd_threshold_config(const MeshFrame& f);

    void set_msg_common(esp_ble_mesh_client_common_param_t* common, uint16_t addr, esp_ble_mesh_model_t* model, uint32_t opcode);
    void request_composition_data(uint16_t unicast);
    void request_app_key_add(uint16_t unicast);
    void request_next_model_bind(uint16_t unicast); 
    void mark_node_ready(uint16_t unicast);
    bool query_element_have_model(uint16_t addr, uint16_t model_id, uint16_t company_id);

    esp_err_t send_vendor_msg(uint16_t addr, esp_ble_mesh_model_t* model, uint32_t opcode, const uint8_t* data, uint16_t len);

    void rpr_scan_timer_tick();
    static void rpr_scan_timer_cb(void* arg);
    void trigger_rpr_scan(uint16_t rpr_srv_addr);
    bool uuid_in_whitelist(const uint8_t uuid[16]) const;

    std::shared_ptr<MeshCommandSender> sender;
    std::shared_ptr<MeshEventDispatcher> dispatcher;
    std::vector<std::array<uint8_t, 16>> uuid_whitelist;

    esp_ble_mesh_client_t config_client{};
    esp_ble_mesh_client_t onoff_client{};
    esp_ble_mesh_client_t sensor_client{};
    esp_ble_mesh_client_t actuator_client{};
    esp_ble_mesh_client_t remote_prov_client{};

    std::vector<esp_ble_mesh_model_t> local_models;

    esp_timer_handle_t rpr_timer = nullptr;
    std::vector<uint16_t> rpr_targets;
    size_t rpr_cursor = 0;
    bool rpr_busy = false;
    uint16_t rpr_current_srv_addr = 0;
    uint32_t cur_rpr_opcode = 0;
    uint32_t rpr_scan_period_ms = 30000;

    struct {
        bool is_busy = false;
    } rpr_state;
};

#endif