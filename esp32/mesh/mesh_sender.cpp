#include "mesh_command_sender.hpp"
#include <cstring>

MeshCommandSender::MeshCommandSender(std::shared_ptr<ReliableTransport> transport, std::shared_ptr<FrameCodec> codec)
    : transport(transport), codec(codec) {}

void MeshCommandSender::send_cmd(OpCode opcode, uint16_t addr, const uint8_t* payload, uint8_t len)
{
    MeshFrame f;
    f.opcode = opcode;
    f.addr = addr;
    f.type = UART_TYPE_DATA;
    if (payload && len > 0) {
        f.payload.assign(payload, payload + len);
    }
    auto raw = codec->encode(f);
    transport->enqueue_frame(raw, true);
}

void MeshCommandSender::unprov_device_adv(const mesh_evt_unprov_adv_t& unprov_dev)
{
    send_struct(OpCode::EVT_RECV_UNPROV_ADV_PKT, 0x0000, unprov_dev);
}

void MeshCommandSender::prov_complete(uint16_t addr, const mesh_evt_prov_complete_t& prov_data)
{
    send_struct(OpCode::EVT_PROV_COMPLETE, addr, prov_data);
}

void MeshCommandSender::heartbeat_status(uint16_t addr, const mesh_evt_heartbeat_t &hb)
{
    send_struct(OpCode::EVT_HEARTBEAT, addr, hb);
}

void MeshCommandSender::sensor_status(uint16_t addr, const mesh_evt_sensor_status_t &sensor)
{
    send_struct(OpCode::EVT_SENSOR_STATUS, addr, sensor);
}

void MeshCommandSender::actuator_status(uint16_t addr, const mesh_evt_actuator_status_t& actuator)
{
    send_struct(OpCode::EVT_ACTUATOR_STATUS, addr, actuator);
}

void MeshCommandSender::group_status(uint16_t addr, const mesh_evt_group_status& group)
{
    send_struct(OpCode::EVT_MODEL_SUBSCRIBE_STATUS, addr, group);
}

void MeshCommandSender::rpr_scan_status(uint16_t addr, const mesh_evt_rpr_scan_status_t& s)
{
    send_struct(OpCode::EVT_RPR_SCAN_STATUS, addr, s);
}

void MeshCommandSender::rpr_scan_report(uint16_t addr, const mesh_evt_rpr_scan_report_t& r)
{
    send_struct(OpCode::EVT_RPR_SCAN_REPORT, addr, r);
}

void MeshCommandSender::rpr_link_status(uint16_t addr, const mesh_evt_rpr_link_status_t& s)
{
    send_struct(OpCode::EVT_RPR_LINK_STATUS, addr, s);
}

void MeshCommandSender::rpr_link_report(uint16_t addr, const mesh_evt_rpr_link_status_t& r)
{
    send_struct(OpCode::EVT_RPR_LINK_REPORT, addr, r);
}

void MeshCommandSender::rpr_link_close(uint16_t addr)
{
    mesh_evt_rpr_simple_t s{ 0 };
    send_struct(OpCode::EVT_RPR_LINK_CLOSE, addr, s);
}

void MeshCommandSender::rpr_start_prov_comp(uint16_t addr, bool success)
{
    mesh_evt_rpr_simple_t s{ static_cast<uint8_t>(success ? 0 : 1) };
    send_struct(OpCode::EVT_RPR_START_PROV_COMP, addr, s);
}

void MeshCommandSender::rpr_prov_complete(uint16_t addr, const mesh_evt_rpr_prov_complete_t& p)
{
    send_struct(OpCode::EVT_RPR_PROV_COMPLETE, addr, p);
}

void MeshCommandSender::generic_client_timeout(uint16_t addr, uint8_t orig_opcode_low_byte)
{
    mesh_evt_generic_timeout_t t{ orig_opcode_low_byte };
    send_struct(OpCode::EVT_GENERIC_CLIENT_TIMEOUT, addr, t);
}