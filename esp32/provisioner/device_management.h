#ifndef DEVICE_MANAGEMENT_H
#define DEVICE_MANAGEMENT_H

#include <cstdint>
#include <vector>
#include <map>
#include <mutex>
#include <cstring>

struct ElementInfo {
    uint16_t element_addr;
    std::vector<uint16_t> sig_models;
    std::vector<uint32_t> vendor_models; // packed: (company_id << 16) | model_id
};

enum class ConfigState {
    IDLE = 0,
    WAIT_COMP_DATA,
    WAIT_ADD_APPKEY,
    WAIT_MODEL_BIND,
    READY
};

struct DeviceInfo {
    uint8_t  uuid[16] = {0};
    uint16_t unicast = 0;
    uint8_t  elem_num = 0;
    uint16_t features = 0;
    bool     rpr_server = false;
    std::vector<ElementInfo> elements;
    ConfigState state = ConfigState::IDLE;

    // Con trỏ tuần tự dùng để bind app key cho từng model một, resume được
    // giữa các lần gọi get_next_bind_target() (mỗi lần chỉ bind 1 model vì
    // Config Model App Bind là bản tin đơn, phải chờ status trả về mới gửi tiếp).
    uint8_t current_bind_element_idx = 0;
    uint8_t current_bind_model_idx = 0;
    bool    binding_vendor = false;
};

struct ProvKeyInfo {
    uint16_t net_idx = 0;
    uint16_t app_idx = 0;
    uint8_t  app_key[16] = {0};
};

// company_id giả (không tồn tại thật trong SIG) dùng để đánh dấu "đây là SIG model"
// khi trả về từ get_next_bind_target(), vì mọi SIG model đều company_id = 0x0000
// theo chuẩn mesh, nhưng ta cần phân biệt với "không có company_id" -> dùng cờ riêng.
constexpr uint16_t BIND_TARGET_IS_SIG_MODEL = 0xFFFF;

class DeviceManager {
public:
    static DeviceManager& getInstance() {
        static DeviceManager instance;
        return instance;
    }

    void set_prov_key(uint16_t net_idx, uint16_t app_idx, const uint8_t* app_key);
    ProvKeyInfo get_prov_key();

    DeviceInfo* alloc_node(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num);
    DeviceInfo* find_node(uint16_t addr); // hỗ trợ tìm theo Element Address luôn
    int delete_device(uint16_t unicast);
    int parse_composition_data(uint16_t unicast, const uint8_t *data, uint16_t length);

    // Đặt/lấy trạng thái cấu hình của 1 node (dùng bởi state machine trong Provisioner)
    void set_state(uint16_t unicast, ConfigState st);
    ConfigState get_state(uint16_t unicast);

    // Trả về true nếu còn model cần bind app key, false nếu đã hết (node sẵn sàng).
    // is_sig_model = true  -> model_id là SIG model (company_id không dùng)
    // is_sig_model = false -> company_id/model_id là của vendor model
    bool get_next_bind_target(uint16_t unicast, uint16_t& model_id, uint16_t& company_id, bool& is_sig_model);

    // Danh sách các node đã READY và có RPR server, dùng cho vòng quét định kỳ.
    std::vector<uint16_t> get_rpr_capable_ready_nodes();

    bool exists(uint16_t unicast);

private:
    DeviceManager() = default;
    ~DeviceManager() = default;

    std::map<uint16_t, DeviceInfo> devices;
    ProvKeyInfo prov_key;
    std::mutex dev_mutex;
};

#endif