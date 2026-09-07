#include "provisioner.hpp"
#include <cstring>
#include <cinttypes>
#include "esp_log.h"

static const char* TAG = "PROVISIONER";

static constexpr uint16_t PROV_OWN_ADDR = 0x0001;

static esp_ble_mesh_model_op_t sensor_client_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_SENSOR_STATUS, 0),
    ESP_BLE_MESH_MODEL_OP_END,
};


static esp_ble_mesh_model_op_t actuator_client_op[] = {
    ESP_BLE_MESH_MODEL_OP(VND_OP_ACTUATOR_STATUS, 0),
    ESP_BLE_MESH_MODEL_OP_END,
};


extern "C" void prov_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {
    Provisioner::getInstance().ble_mesh_provisioning_cb(event, param);
}
extern "C" void config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event, esp_ble_mesh_cfg_client_cb_param_t* param) {
    Provisioner::getInstance().ble_mesh_config_client_cb(event, param);
}
extern "C" void generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event, esp_ble_mesh_generic_client_cb_param_t *param) {
    Provisioner::getInstance().ble_mesh_generic_client_cb(event, param);
}
extern "C" void rpr_client_cb(esp_ble_mesh_rpr_client_cb_event_t event, esp_ble_mesh_rpr_client_cb_param_t *param) {
    Provisioner::getInstance().ble_mesh_rpr_client_cb(event, param);
}
extern "C" void vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param) {
    Provisioner::getInstance().ble_mesh_vendor_model_cb(event, param);
}

void Provisioner::init(std::shared_ptr<MeshCommandSender> sender_ptr, std::shared_ptr<MeshEventDispatcher> dispatcher_ptr)
{
    sender = sender_ptr;
    dispatcher = dispatcher_ptr;

    dispatcher->on(OpCode::CMD_PROV_ENABLE,        [this](const MeshFrame& f) { handle_cmd_prov_enable(f); });
    dispatcher->on(OpCode::CMD_PROV_DISABLE,       [this](const MeshFrame& f) { handle_cmd_prov_disable(f); });
    dispatcher->on(OpCode::CMD_ADD_UNPROV_DEV,     [this](const MeshFrame& f) { handle_cmd_add_unprov_dev(f); });
    dispatcher->on(OpCode::CMD_SET_DEV_UUID_MATCH, [this](const MeshFrame& f) { handle_cmd_set_uuid_match(f); });
    dispatcher->on(OpCode::CMD_DELETE_NODE,        [this](const MeshFrame& f) { handle_cmd_delete_node(f); });
    dispatcher->on(OpCode::CMD_GROUP_ADD,          [this](const MeshFrame& f) { handle_cmd_group_add(f); });
    dispatcher->on(OpCode::CMD_GROUP_DELETE,       [this](const MeshFrame& f) { handle_cmd_group_delete(f); });
    dispatcher->on(OpCode::CMD_MODEL_PUB_SET,      [this](const MeshFrame& f) { handle_cmd_model_pub_set(f); });
    dispatcher->on(OpCode::CMD_SENSOR_GET,         [this](const MeshFrame& f) { handle_cmd_sensor_get(f); });
    dispatcher->on(OpCode::CMD_ACTUATOR_SET,       [this](const MeshFrame& f) { handle_cmd_actuator_set(f); });
    dispatcher->on(OpCode::CMD_THRESHOLD_CONFIG,   [this](const MeshFrame& f) { handle_cmd_threshold_config(f); });
    dispatcher->on(OpCode::CMD_RPR_SCAN_START,     [this](const MeshFrame& f) { handle_cmd_rpr_scan_start(f); });
    dispatcher->on(OpCode::CMD_RPR_SCAN_STOP,      [this](const MeshFrame& f) { handle_cmd_rpr_scan_stop(f); });
}

esp_ble_mesh_model_t* Provisioner::get_local_models(size_t& count)
{
    local_models.clear();
    local_models.push_back(ESP_BLE_MESH_MODEL_CFG_CLI(&config_client));
    local_models.push_back(ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_client));
    local_models.push_back(ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_SENSOR, sensor_client_op, NULL, &sensor_client));
    local_models.push_back(ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, VND_MODEL_ID_ACTUATOR, actuator_client_op, NULL, &actuator_client));
    local_models.push_back(ESP_BLE_MESH_MODEL_RPR_CLI(&remote_prov_client));

    count = local_models.size();
    return local_models.data();
}

void Provisioner::add_uuid_to_whitelist(const uint8_t uuid[16]) {
    std::array<uint8_t, 16> a{};
    memcpy(a.data(), uuid, 16);
    uuid_whitelist.push_back(a);
}

bool Provisioner::uuid_in_whitelist(const uint8_t uuid[16]) const {
    if (uuid_whitelist.empty()) {
        return false;
    }
    for (auto& w : uuid_whitelist) {
        if (memcmp(w.data(), uuid, 16) == 0) {
            return true;
        }
    }
    return false;
}

void Provisioner::set_msg_common(esp_ble_mesh_client_common_param_t* common, uint16_t addr, esp_ble_mesh_model_t* model, uint32_t opcode)
{
    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = DeviceManager::getInstance().get_prov_key().net_idx;
    common->ctx.app_idx = DeviceManager::getInstance().get_prov_key().app_idx;
    common->ctx.addr = addr;
    common->ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    common->msg_timeout = 0;
}

bool Provisioner::query_element_have_model(uint16_t addr, uint16_t model_id, uint16_t company_id)
{
    DeviceInfo* node = DeviceManager::getInstance().find_node(addr);
    if (node == nullptr) {
        return false;
    }
    for (auto& elem : node->elements) {
        if (elem.element_addr != addr) {
            continue;
        }
        if (company_id == ESP_BLE_MESH_CID_NVAL) {
            for (auto sig : elem.sig_models) {
                if (sig == model_id) {
                    return true;
                }
            }
        } else {
            uint32_t packed = (static_cast<uint32_t>(company_id) << 16) | model_id;
            for (auto v : elem.vendor_models) {
                if (v == packed) {
                    return true;
                }
            }
        }
    }
    return false;
}

esp_err_t Provisioner::send_vendor_msg(uint16_t addr, esp_ble_mesh_model_t* model, uint32_t opcode, const uint8_t* data, uint16_t len)
{
    esp_ble_mesh_msg_ctx_t ctx = {};
    ctx.net_idx = DeviceManager::getInstance().get_prov_key().net_idx;
    ctx.app_idx = DeviceManager::getInstance().get_prov_key().app_idx;
    ctx.addr = addr;
    ctx.send_ttl = ESP_BLE_MESH_TTL_DEFAULT;
    return esp_ble_mesh_client_model_send_msg(model, &ctx, opcode, len, const_cast<uint8_t*>(data), 0, false, ROLE_NODE);
}

void Provisioner::ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t* param)
{
    switch (event) {

    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT: {
        ESP_LOGI(TAG, "provisioner enabled");
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT: {
        ESP_LOGI(TAG, "provisioner disabled");
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT: {
        const uint8_t *dev_uuid = param->provisioner_recv_unprov_adv_pkt.dev_uuid;
        uint8_t bearer = (param->provisioner_recv_unprov_adv_pkt.bearer & ESP_BLE_MESH_PROV_ADV) ? 0x00 : 0x01;

        if (uuid_in_whitelist(dev_uuid)) {
            esp_ble_mesh_unprov_dev_add_t add_dev = {};
            memcpy(add_dev.addr, param->provisioner_recv_unprov_adv_pkt.addr, ESP_BLE_MESH_ADDR_LEN);
            add_dev.addr_type = static_cast<esp_ble_mesh_addr_type_t>(param->provisioner_recv_unprov_adv_pkt.addr_type);
            memcpy(add_dev.uuid, dev_uuid, 16);
            add_dev.oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info;
            add_dev.bearer = param->provisioner_recv_unprov_adv_pkt.bearer;

            esp_err_t err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev, static_cast<esp_ble_mesh_dev_add_flag_t>(ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG));
            if (err) {
                ESP_LOGE(TAG, "Auto add unprov dev failed, err=%d", err);
            }
        } else {
            mesh_evt_unprov_adv_t evt = {};
            memcpy(evt.uuid, dev_uuid, 16);
            evt.oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info;
            evt.bearer = bearer;
            evt.rssi = param->provisioner_recv_unprov_adv_pkt.rssi;
            sender->unprov_device_adv(evt);
        }
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT: {
        ESP_LOGI(TAG, "prov link open");
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT: {
        ESP_LOGI(TAG, "prov link close with reason: %d", param->provisioner_prov_link_close.reason);
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        uint16_t unicast = param->provisioner_prov_complete.unicast_addr;
        uint8_t elem_num = param->provisioner_prov_complete.element_num;
        uint16_t net_idx = param->provisioner_prov_complete.netkey_idx;
        const uint8_t *uuid = param->provisioner_prov_complete.device_uuid;

        DeviceInfo *node = DeviceManager::getInstance().alloc_node(uuid, unicast, elem_num);
        if (!node) {
            ESP_LOGE(TAG, "No node slot for 0x%04x", unicast);
            break;
        }

        mesh_evt_prov_complete_t evt = {};
        evt.net_idx = net_idx;
        evt.elem_num = elem_num;
        memcpy(evt.uuid, uuid, 16);
        sender->prov_complete(unicast, evt);

        request_composition_data(unicast);
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT: {
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK) {
            uint16_t net_idx = 0; 
            uint16_t app_idx = param->provisioner_add_app_key_comp.app_idx;
            DeviceManager::getInstance().set_prov_key(net_idx, app_idx, nullptr);

            esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, app_idx, VND_MODEL_ID_SENSOR, CID_ESP);
            esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, app_idx, VND_MODEL_ID_ACTUATOR, CID_ESP);
            esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, app_idx, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
        } else {
            ESP_LOGE(TAG, "Add local app key failed, err=%d", param->provisioner_add_app_key_comp.err_code);
        }
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_DELETE_NODE_WITH_ADDR_COMP_EVT: {
        uint16_t del_addr = param->provisioner_delete_node_with_addr_comp.unicast_addr;
        DeviceManager::getInstance().delete_device(del_addr);
        break;
    }

    case ESP_BLE_MESH_PROVISIONER_RECV_HEARTBEAT_MESSAGE_EVT: {
        mesh_evt_heartbeat_t hb = {};
        hb.init_ttl = param->provisioner_recv_heartbeat.init_ttl;
        hb.hops = param->provisioner_recv_heartbeat.hops;
        hb.features = param->provisioner_recv_heartbeat.feature;
        sender->heartbeat_status(param->provisioner_recv_heartbeat.hb_src, hb);
        break;
    }

    default:
        break;
    }
}

void Provisioner::request_composition_data(uint16_t unicast)
{
    DeviceManager::getInstance().set_state(unicast, ConfigState::WAIT_COMP_DATA);
    esp_ble_mesh_client_common_param_t  common    = {};
    esp_ble_mesh_cfg_client_get_state_t get_state = {};
    set_msg_common(&common, unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    esp_err_t err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err){
         ESP_LOGE(TAG, "Composition Data Get failed, addr=0x%04x", unicast);
        }
}

void Provisioner::request_app_key_add(uint16_t unicast)
{
    DeviceManager::getInstance().set_state(unicast, ConfigState::WAIT_ADD_APPKEY);
    ProvKeyInfo key = DeviceManager::getInstance().get_prov_key();

    esp_ble_mesh_client_common_param_t common    = {};
    esp_ble_mesh_cfg_client_set_state_t set_state = {};
    set_msg_common(&common, unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
    set_state.app_key_add.net_idx = key.net_idx;
    set_state.app_key_add.app_idx = key.app_idx;
    memcpy(set_state.app_key_add.app_key, key.app_key, 16);
    esp_err_t err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) {
        ESP_LOGE(TAG, "AppKey Add failed, addr=0x%04x", unicast);
    }
}

void Provisioner::request_next_model_bind(uint16_t unicast)
{
    uint16_t model_id = 0, company_id = 0;
    bool is_sig = false;
    if (!DeviceManager::getInstance().get_next_bind_target(unicast, model_id, company_id, is_sig)) {
        mark_node_ready(unicast);
        return;
    }

    DeviceManager::getInstance().set_state(unicast, ConfigState::WAIT_MODEL_BIND);
    ProvKeyInfo key = DeviceManager::getInstance().get_prov_key();

    esp_ble_mesh_client_common_param_t  common = {};
    esp_ble_mesh_cfg_client_set_state_t set_state = {};
    set_msg_common(&common, unicast, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
    set_state.model_app_bind.element_addr = unicast; 
    set_state.model_app_bind.model_app_idx = key.app_idx;
    set_state.model_app_bind.model_id = model_id;
    set_state.model_app_bind.company_id = is_sig ? ESP_BLE_MESH_CID_NVAL : company_id;
    esp_err_t err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    
    if (err) {
        ESP_LOGE(TAG, "Model App Bind failed, addr=0x%04x model=0x%04x", unicast, model_id);
    }
}


void Provisioner::mark_node_ready(uint16_t unicast)
{
    DeviceManager::getInstance().set_state(unicast, ConfigState::READY);
    ESP_LOGI(TAG, "Node 0x%04x is fully configured (READY)", unicast);
}

void Provisioner::ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event, esp_ble_mesh_cfg_client_cb_param_t* param)
{
    uint32_t opcode = param->params->opcode;
    uint16_t addr   = param->params->ctx.addr;

    ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04" PRIx32, __func__, param->error_code, event, addr, opcode);

    if (param->error_code) {
        ESP_LOGE(TAG, "Config client message failed, opcode 0x%04" PRIx32, opcode);
        if (event == ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT) {
            ConfigState st = DeviceManager::getInstance().get_state(addr);
            switch (st) {
                case ConfigState::WAIT_COMP_DATA: {
                    request_composition_data(addr); 
                    break;
                }
                case ConfigState::WAIT_ADD_APPKEY: {
                    request_app_key_add(addr); 
                    break;
                }
                case ConfigState::WAIT_MODEL_BIND: {
                    request_next_model_bind(addr); 
                    break;
                }
                default: {
                    break;
                }

            }
        }
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        if (opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            auto *comp = param->status_cb.comp_data_status.composition_data;
            DeviceManager::getInstance().parse_composition_data(addr, comp->data, comp->len);
            request_app_key_add(addr);
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        if (opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            request_next_model_bind(addr);
        } else if (opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            request_next_model_bind(addr); 
        } else if (opcode == ESP_BLE_MESH_MODEL_OP_MODEL_SUBSCRIPTION_ADD || opcode == ESP_BLE_MESH_MODEL_OP_MODEL_SUBSCRIPTION_DELETE) {
            mesh_evt_group_status g = {};
            g.element_addr = addr;
            g.group_addr = param->status_cb.model_sub_status.sub_addr;
            g.model_id = param->status_cb.model_sub_status.model_id;
            g.is_sub = true;
            g.is_add = (opcode == ESP_BLE_MESH_MODEL_OP_MODEL_SUBSCRIPTION_ADD);
            g.success = (param->status_cb.model_sub_status.status == 0);
            sender->group_status(addr, g);
        } else if (opcode == ESP_BLE_MESH_MODEL_OP_MODEL_PUBLICATION_SET) {
            mesh_evt_group_status g = {};
            g.element_addr = addr;
            g.group_addr = param->status_cb.model_pub_status.publish_addr;
            g.model_id = param->status_cb.model_pub_status.model_id;
            g.is_sub = false;
            g.is_add = true;
            g.success = (param->status_cb.model_pub_status.status == 0);
            sender->group_status(addr, g);
        }
        break;

    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        break;

    default:
        break;
    }
}

void Provisioner::ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event, esp_ble_mesh_generic_client_cb_param_t* param)
{
    uint32_t opcode = param->params->opcode;
    uint16_t addr   = param->params->ctx.addr;

    if (param->error_code) {
        ESP_LOGE(TAG, "Generic client message failed, opcode 0x%04" PRIx32, opcode);
        if (event == ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT) {
            sender->generic_client_timeout(addr, static_cast<uint8_t>(opcode & 0xFF));
        }
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        if (opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET ||
            opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            ESP_LOGI(TAG, "addr 0x%04x onoff = 0x%02x", addr, param->status_cb.onoff_status.present_onoff);
        }
        break;
    default:
        break;
    }
}

void Provisioner::ble_mesh_rpr_client_cb(esp_ble_mesh_rpr_client_cb_event_t event, esp_ble_mesh_rpr_client_cb_param_t* param)
{
    esp_ble_mesh_client_common_param_t common = {};
    esp_err_t err;
    uint16_t addr;

    switch (event) {
    case ESP_BLE_MESH_RPR_CLIENT_RECV_PUB_EVT:
    case ESP_BLE_MESH_RPR_CLIENT_RECV_RSP_EVT:
        addr = param->recv.params->ctx.addr;

        switch (param->recv.params->ctx.recv_op) {

        case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_STATUS: {
            ESP_LOGI(TAG, "RPR scan status from 0x%04x: status=%d scanning=%d", addr, param->recv.val.scan_status.status, param->recv.val.scan_status.rpr_scanning);
            break;
        }

        case ESP_BLE_MESH_MODEL_OP_RPR_SCAN_REPORT: {
            const uint8_t *uuid = param->recv.val.scan_report.uuid;
            ESP_LOGI(TAG, "RPR scan report from 0x%04x, rssi=%d", addr, param->recv.val.scan_report.rssi);


            if (!rpr_state.is_busy && uuid_in_whitelist(uuid)) {
                rpr_state.is_busy = true;
                cur_rpr_opcode = ESP_BLE_MESH_MODEL_OP_RPR_LINK_OPEN;
                esp_ble_mesh_rpr_client_act_param_t act = {};
                act.link_open.model = remote_prov_client.model;
                act.link_open.rpr_srv_addr = addr;
                memcpy(act.link_open.uuid, r.uuid, 16);
                err = esp_ble_mesh_rpr_client_action(ESP_BLE_MESH_RPR_CLIENT_ACT_LINK_OPEN, &act);
                if (err) {
                    ESP_LOGE(TAG, "RPR Link Open failed %d", err);
                    rpr_state.is_busy = false;
                }
            } else if (uuid_in_whitelist(uuid) == false) {
                mesh_evt_unprov_adv_t evt = {};
                memcpy(evt.uuid, uuid, 16);
                evt.oob_info = param->recv.val.scan_report.oob_info;
                evt.bearer = param->recv.val.scan_report.bearer;
                evt.rssi = param->recv.val.scan_report.rssi;
                sender->unprov_device_adv(evt);
            }
            break;
        }

        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_STATUS: {
            ESP_LOGI(TAG, "RPR link status from 0x%04x: status=%d state=%d", addr, param->recv.val.link_status.status, param->recv.val.link_status.rpr_state);
            break;
        }

        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_REPORT: {
            ESP_LOGI(TAG, "RPR link report from 0x%04x: status=%d state=%d", addr, param->recv.val.link_report.status, param->recv.val.link_report.rpr_state);


            if (param->recv.val.link_report.status == ESP_BLE_MESH_RPR_STATUS_SUCCESS && param->recv.val.link_report.rpr_state == ESP_BLE_MESH_RPR_LINK_ACTIVE && cur_rpr_opcode == ESP_BLE_MESH_MODEL_OP_RPR_LINK_OPEN) {
                esp_ble_mesh_rpr_client_act_param_t act = {};
                act.start_rpr.model = remote_prov_client.model;
                act.start_rpr.rpr_srv_addr = addr;
                err = esp_ble_mesh_rpr_client_action(ESP_BLE_MESH_RPR_CLIENT_ACT_START_RPR, &act);
                if (err) ESP_LOGE(TAG, "RPR Start Prov failed %d", err);
                cur_rpr_opcode = ESP_BLE_MESH_MODEL_OP_RPR_LINK_REPORT; 
            }
            break;
        }

        case ESP_BLE_MESH_MODEL_OP_RPR_LINK_CLOSE: {
            ESP_LOGI(TAG, "RPR link close");
            rpr_state.is_busy = false;
            break;
        }

        default: break;
        }
        break;

    case ESP_BLE_MESH_RPR_CLIENT_ACT_COMP_EVT:
        if (param->act.sub_evt == ESP_BLE_MESH_START_RPR_COMP_SUB_EVT) {
            bool ok = (param->act.start_rpr_comp.err_code == ESP_OK);
            sender->rpr_start_prov_comp(param->act.start_rpr_comp.rpr_srv_addr, ok);
            if (!ok) {
                rpr_state.is_busy = false;
            }
        }
        break;

    case ESP_BLE_MESH_RPR_CLIENT_LINK_CLOSE_EVT:
        rpr_state.is_busy = false;
        sender->rpr_link_close(0);
        break;

    case ESP_BLE_MESH_RPR_CLIENT_PROV_COMP_EVT: {
        rpr_state.is_busy = false;

        uint16_t unicast  = param->prov.unicast_addr;
        uint8_t  elem_num = param->prov.element_num;
        const uint8_t *uuid = param->prov.uuid;


        mesh_evt_prov_complete_t evt = {};
        evt.net_idx = net_idx;
        evt.elem_num = elem_num;
        memcpy(evt.uuid, uuid, 16);
        sender->prov_complete(unicast, evt);

        esp_ble_mesh_rpr_client_msg_t msg = {};
        set_msg_common(&common, param->prov.rpr_srv_addr, remote_prov_client.model, ESP_BLE_MESH_MODEL_OP_RPR_LINK_CLOSE);
        msg.link_close.reason = ESP_BLE_MESH_RPR_REASON_SUCCESS;
        esp_ble_mesh_rpr_client_send(&common, &msg);

        DeviceInfo *node = DeviceManager::getInstance().alloc_node(p.uuid, p.unicast, p.elem_num);
        if (node) {
            request_composition_data(p.unicast);
        }
        break;
    }

    default: break;
    }
}

void Provisioner::ble_mesh_vendor_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t* param)
{
    if (!param) return;

    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT: {
        uint32_t opcode;
        uint16_t src_addr;
        const uint8_t *msg;
        uint16_t msg_len;

        if (event == ESP_BLE_MESH_MODEL_OPERATION_EVT) {
            opcode   = param->model_operation.opcode;
            src_addr = param->model_operation.ctx->addr;
            msg      = param->model_operation.msg;
            msg_len  = param->model_operation.length;
        } else {
            opcode   = param->client_recv_publish_msg.opcode;
            src_addr = param->client_recv_publish_msg.ctx->addr;
            msg      = param->client_recv_publish_msg.msg;
            msg_len  = param->client_recv_publish_msg.length;
        }

        if (opcode == VND_OP_SENSOR_STATUS) {
            if (msg_len < sizeof(mesh_evt_sensor_status_t)) break;
            mesh_evt_sensor_status_t evt;
            memcpy(&evt, msg, sizeof(evt));
            sender->sensor_status(src_addr, evt);

        } else if (opcode == VND_OP_ACTUATOR_STATUS) {
            if (msg_len < sizeof(mesh_evt_actuator_status_t)) break;
            mesh_evt_actuator_status_t evt;
            memcpy(&evt, msg, sizeof(evt));
            sender->actuator_status(src_addr, evt);
        }
        break;
    }

    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT: {
        sender->generic_client_timeout(param->client_send_timeout.ctx->addr, static_cast<uint8_t>(param->client_send_timeout.opcode & 0xFF));
        break;
    }

    default: break;
    }
}

void Provisioner::handle_cmd_prov_enable(const MeshFrame&) {
    esp_err_t err = esp_ble_mesh_provisioner_prov_enable(
        static_cast<esp_ble_mesh_prov_bearer_t>(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err) ESP_LOGE(TAG, "Prov enable failed %d", err);
}

void Provisioner::handle_cmd_prov_disable(const MeshFrame&) {
    esp_err_t err = esp_ble_mesh_provisioner_prov_disable(
        static_cast<esp_ble_mesh_prov_bearer_t>(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err) ESP_LOGE(TAG, "Prov disable failed %d", err);
}

void Provisioner::handle_cmd_add_unprov_dev(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_add_unprov_dev_t)) return;
    mesh_cmd_add_unprov_dev_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));

    add_uuid_to_whitelist(cmd.uuid);
    ESP_LOGI(TAG, "UUID added to whitelist, will auto-provision on next sighting");
}

void Provisioner::handle_cmd_set_uuid_match(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_set_uuid_match_t)) return;
    mesh_cmd_set_uuid_match_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));
    esp_err_t err = esp_ble_mesh_provisioner_set_dev_uuid_match(cmd.uuid_match, sizeof(cmd.uuid_match), 0, false);
    if (err) ESP_LOGE(TAG, "Set uuid match failed %d", err);
}

void Provisioner::handle_cmd_delete_node(const MeshFrame& f) {
    esp_err_t err = esp_ble_mesh_provisioner_delete_node_with_addr(f.addr);
    if (err) ESP_LOGE(TAG, "Delete node 0x%04x failed %d", f.addr, err);
}

void Provisioner::handle_cmd_group_add(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_add_dev_to_group_t)) return;
    mesh_cmd_add_dev_to_group_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));

    esp_ble_mesh_client_common_param_t  common    = {};
    esp_ble_mesh_cfg_client_set_state_t set_state = {};
    set_msg_common(&common, cmd.element_addr, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_SUBSCRIPTION_ADD);
    set_state.model_sub_add.element_addr = cmd.element_addr;
    set_state.model_sub_add.sub_addr     = cmd.group_addr;
    set_state.model_sub_add.model_id     = cmd.model_id;
    set_state.model_sub_add.company_id   = cmd.company_id;
    esp_err_t err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) ESP_LOGE(TAG, "Model Subscription Add failed %d", err);
}

void Provisioner::handle_cmd_group_delete(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_remove_dev_from_group_t)) return;
    mesh_cmd_remove_dev_from_group_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));

    esp_ble_mesh_client_common_param_t  common    = {};
    esp_ble_mesh_cfg_client_set_state_t set_state = {};
    set_msg_common(&common, cmd.element_addr, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_SUBSCRIPTION_DELETE);
    set_state.model_sub_delete.element_addr = cmd.element_addr;
    set_state.model_sub_delete.sub_addr     = cmd.group_addr;
    set_state.model_sub_delete.model_id     = cmd.model_id;
    set_state.model_sub_delete.company_id   = cmd.company_id;
    esp_err_t err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) ESP_LOGE(TAG, "Model Subscription Delete failed %d", err);
}

void Provisioner::handle_cmd_model_pub_set(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_model_pub_set_t)) {
        return;
    }
    mesh_cmd_model_pub_set_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));

    esp_ble_mesh_client_common_param_t  common    = {};
    esp_ble_mesh_cfg_client_set_state_t set_state = {};
    set_msg_common(&common, cmd.element_addr, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_PUBLICATION_SET);
    set_state.model_pub_set.element_addr        = cmd.element_addr;
    set_state.model_pub_set.publish_addr        = cmd.pub_addr;
    set_state.model_pub_set.publish_app_idx     = DeviceManager::getInstance().get_prov_key().app_idx;
    set_state.model_pub_set.publish_ttl         = cmd.pub_ttl;
    set_state.model_pub_set.publish_period      = cmd.pub_period;
    set_state.model_pub_set.publish_retransmit  = 0; 
    set_state.model_pub_set.model_id            = cmd.model_id;
    set_state.model_pub_set.company_id          = cmd.company_id;
    esp_err_t err = esp_ble_mesh_config_client_set_state(&common, &set_state);
    if (err) ESP_LOGE(TAG, "Model Publication Set failed %d", err);
}

void Provisioner::handle_cmd_sensor_get(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_sensor_get_t)) return;
    mesh_cmd_sensor_get_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));
    uint16_t target_addr = f.addr + cmd.sensor_id;
    send_vendor_msg(target_addr, sensor_client.model, VND_OP_SENSOR_GET, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

void Provisioner::handle_cmd_actuator_set(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_actuator_set_t)) return;
    mesh_cmd_actuator_set_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));
    uint16_t target_addr = f.addr + cmd.actuator_id;
    send_vendor_msg(target_addr, actuator_client.model, VND_OP_ACTUATOR_SET, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

void Provisioner::handle_cmd_threshold_config(const MeshFrame& f) {
    if (f.payload.size() < sizeof(mesh_cmd_threshold_t)) return;
    mesh_cmd_threshold_t cmd;
    memcpy(&cmd, f.payload.data(), sizeof(cmd));
    uint16_t target_addr = f.addr + cmd.actuator_id;
    send_vendor_msg(target_addr, sensor_client.model, VND_OP_SENSOR_THRESHOLD_SET, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

void Provisioner::handle_cmd_rpr_scan_start(const MeshFrame& f) {
    trigger_rpr_scan(f.addr);
}

void Provisioner::handle_cmd_rpr_scan_stop(const MeshFrame& f) {
    esp_ble_mesh_rpr_client_act_param_t act = {};
    act.scan_stop.model        = remote_prov_client.model;
    act.scan_stop.rpr_srv_addr = f.addr;
    esp_ble_mesh_rpr_client_action(ESP_BLE_MESH_RPR_CLIENT_ACT_SCAN_STOP, &act);
}

void Provisioner::start_periodic_rpr_scan(uint32_t period_ms)
{
    rpr_scan_period_ms = period_ms;
    if (rpr_timer) {
        return;
    } 

    const esp_timer_create_args_t args = {
        .callback = &Provisioner::rpr_scan_timer_cb,
        .arg = this,
        .name = "rpr_scan_timer",
    };
    esp_timer_create(&args, &rpr_timer);
    esp_timer_start_periodic(rpr_timer, static_cast<uint64_t>(period_ms) * 1000ULL);
}

void Provisioner::stop_periodic_rpr_scan()
{
    if (rpr_timer) {
        esp_timer_stop(rpr_timer);
        esp_timer_delete(rpr_timer);
        rpr_timer = nullptr;
    }
}

void Provisioner::rpr_scan_timer_cb(void* arg)
{
    static_cast<Provisioner*>(arg)->rpr_scan_timer_tick();
}

void Provisioner::rpr_scan_timer_tick()
{
    if (rpr_state.is_busy) {
        return;
    }

    rpr_targets = DeviceManager::getInstance().get_rpr_capable_ready_nodes();
    if (rpr_targets.empty()) {
        return;
    }

    if (rpr_cursor >= rpr_targets.size()) {
        rpr_cursor = 0;
    }
    uint16_t addr = rpr_targets[rpr_cursor];
    rpr_cursor++;

    trigger_rpr_scan(addr);
}

void Provisioner::trigger_rpr_scan(uint16_t rpr_srv_addr)
{
    rpr_current_srv_addr = rpr_srv_addr;
    esp_ble_mesh_rpr_client_act_param_t act = {};
    act.scan_start.model = remote_prov_client.model;
    act.scan_start.rpr_srv_addr = rpr_srv_addr;
    act.scan_start.scan_item_type = ESP_BLE_MESH_RPR_SCAN_UNPROV_DEV;
    act.scan_start.single_scan_timeout = 10; 

    esp_err_t err = esp_ble_mesh_rpr_client_action(ESP_BLE_MESH_RPR_CLIENT_ACT_SCAN_START, &act);
    if (err) {
        ESP_LOGE(TAG, "RPR Scan Start failed on 0x%04x, err=%d", rpr_srv_addr, err);
    }
}