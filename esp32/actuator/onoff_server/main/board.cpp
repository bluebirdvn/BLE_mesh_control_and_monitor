#include "board.hpp"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

static const char *TAG = "BOARD";

#if CONFIG_IDF_TARGET_ESP32
    #define BOARD_HAS_GPIO_LED 1
    #define BOARD_LED_GPIO GPIO_NUM_2   // LED onboard phổ biến trên devkit ESP32 gốc - đổi lại nếu board khác

#elif CONFIG_IDF_TARGET_ESP32H2 || CONFIG_IDF_TARGET_ESP32C6
    #define BOARD_HAS_RGB_LED 1
    #define BOARD_RGB_GPIO GPIO_NUM_8   // LED RGB onboard phổ biến trên DevKitM/DevKitC H2/C6 - KIỂM TRA LẠI theo đúng board thật của bạn
    #include "led_strip.h"

#else
    #warning "board.cpp: chip chưa được liệt kê rõ, dùng fallback GPIO LED ở GPIO2 - kiểm tra lại chân cho đúng board của bạn"
    #define BOARD_HAS_GPIO_LED 1
    #define BOARD_LED_GPIO GPIO_NUM_2
#endif

#if BOARD_HAS_RGB_LED
static led_strip_handle_t s_strip;
#endif

enum board_evt_type_t : uint8_t {
    BOARD_EVT_ONOFF = 1,
    BOARD_EVT_SETPOINT = 2,
};

struct board_msg_t {
    board_evt_type_t type;
    uint16_t value;
};

static QueueHandle_t s_queue;
static uint8_t s_last_onoff = 0;

/* =========================================================================
 *  Hàm set màu/mức thấp nhất - duy nhất chỗ này biết tới GPIO/RGB thật
 * ========================================================================= */
static void board_set_raw(uint8_t r, uint8_t g, uint8_t b)
{
#if BOARD_HAS_RGB_LED
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
#elif BOARD_HAS_GPIO_LED
    gpio_set_level(BOARD_LED_GPIO, (r || g || b) ? 1 : 0);
#endif
}

static void board_set_steady_state(void)
{
    // Trạng thái ổn định: xanh lá nhạt = ON, tắt hẳn = OFF (dùng chung cho
    // cả GPIO LED lẫn RGB - GPIO LED chỉ quan tâm "có màu hay không").
    board_set_raw(0, s_last_onoff ? 40 : 0, 0);
}

static void board_blink_once(uint8_t r, uint8_t g, uint8_t b, uint32_t on_ms, uint32_t off_ms)
{
    board_set_raw(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    board_set_raw(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

/* =========================================================================
 *  Task nền: nhận sự kiện qua queue, tự chạy hiệu ứng, không chặn người gọi
 *  (actuator_model_cb chạy trong context callback của ngăn xếp BLE Mesh -
 *  không nên block ở đó để xử lý nhấp nháy nhiều lần).
 * ========================================================================= */
static void board_task(void *arg)
{
    board_msg_t msg;
    board_set_raw(0, 0, 0); // trạng thái khởi động: tắt

    while (true) {
        if (xQueueReceive(s_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        switch (msg.type) {
        case BOARD_EVT_ONOFF:
            s_last_onoff = (uint8_t)msg.value;
            board_set_steady_state();
            break;

        case BOARD_EVT_SETPOINT:
            // Chớp xanh dương 3 lần để báo "vừa nhận setpoint mới", rồi quay
            // lại đúng trạng thái ON/OFF hiện tại. (GPIO LED sẽ chớp sáng/tắt
            // bình thường vì board_set_raw chỉ nhìn "có màu hay không").
            for (int i = 0; i < 3; i++) {
                board_blink_once(0, 0, 40, 80, 80);
            }
            board_set_steady_state();
            break;
        }
    }
}

/* =========================================================================
 *  API public
 * ========================================================================= */
void board_actuator_init(void)
{
#if BOARD_HAS_GPIO_LED
    gpio_reset_pin(BOARD_LED_GPIO);
    gpio_set_direction(BOARD_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_LED_GPIO, 0);
#endif

#if BOARD_HAS_RGB_LED
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = BOARD_RGB_GPIO;
    strip_config.max_leds = 1;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB; // TODO: đổi thành .color_component_format nếu dùng component bản >= 3.x (xem README)
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz - giá trị chuẩn cho WS2812 theo ví dụ của Espressif

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %d - kiểm tra lại GPIO/thư viện led_strip đã thêm chưa", err);
    } else {
        led_strip_clear(s_strip);
    }
#endif

    s_queue = xQueueCreate(8, sizeof(board_msg_t));
    xTaskCreate(board_task, "board_task", 3072, nullptr, 4, nullptr);

    ESP_LOGI(TAG, "board_actuator_init done (%s)",
#if BOARD_HAS_RGB_LED
             "RGB LED WS2812 qua led_strip"
#else
             "GPIO LED"
#endif
    );
}

void board_actuator_notify_onoff(uint8_t onoff)
{
    board_msg_t msg = { BOARD_EVT_ONOFF, onoff };
    xQueueSend(s_queue, &msg, 0); // không chờ - nếu queue đầy thì bỏ qua, ưu tiên không block caller
}

void board_actuator_notify_setpoint(uint16_t setpoint)
{
    board_msg_t msg = { BOARD_EVT_SETPOINT, setpoint };
    xQueueSend(s_queue, &msg, 0);
}