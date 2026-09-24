#include "board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdbool.h>

#if !CONFIG_IDF_TARGET_ESP32
#define BOARD_LED_MODE_RGB 1
#include "led_strip.h"
#define BOARD_RGB_LED_GPIO GPIO_NUM_8 /* Common default on H2/C6 devkits, check your board's schematic */
#else
#define BOARD_LED_MODE_RGB 0
#define BOARD_LED_GPIO GPIO_NUM_2 /* Common default on ESP32 devkits, check your board's schematic */
#endif

#define BOARD_LED_FLASH_MS 80
#define BOARD_LED_GAP_MS 60

static const char *TAG = "BOARD";
static QueueHandle_t s_led_queue = NULL;

#if BOARD_LED_MODE_RGB
static led_strip_handle_t s_led_strip = NULL;
#endif

static void board_led_set(bool on, uint8_t r, uint8_t g, uint8_t b)
{
#if BOARD_LED_MODE_RGB
    if (s_led_strip == NULL)
    {
        return;
    }

    if (on)
    {
        led_strip_set_pixel(s_led_strip, 0, r, g, b);
    }
    else
    {
        led_strip_clear(s_led_strip);
    }

    led_strip_refresh(s_led_strip);
#else
    (void)r;
    (void)g;
    (void)b;
    gpio_set_level(BOARD_LED_GPIO, on ? 1 : 0);
#endif
}

static void board_led_flash(uint8_t r, uint8_t g, uint8_t b)
{
    board_led_set(true, r, g, b);
    vTaskDelay(pdMS_TO_TICKS(BOARD_LED_FLASH_MS));
    board_led_set(false, 0, 0, 0);
}

static void board_led_task(void *arg)
{
    board_led_event_t event;

    while (true)
    {
        if (xQueueReceive(s_led_queue, &event, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if (event == BOARD_LED_EVENT_RX)
        {
            /* 1 flash, green on RGB boards. */
            board_led_flash(0, 40, 0);
        }
        else
        {
            /* 2 quick flashes, blue on RGB boards. */
            board_led_flash(0, 0, 40);
            vTaskDelay(pdMS_TO_TICKS(BOARD_LED_GAP_MS));
            board_led_flash(0, 0, 40);
        }
    }
}

static bool board_led_hw_init(void)
{
#if BOARD_LED_MODE_RGB
    led_strip_config_t strip_config = {0};
    strip_config.strip_gpio_num = BOARD_RGB_LED_GPIO;
    strip_config.max_leds = 1;

    led_strip_rmt_config_t rmt_config = {0};
    rmt_config.resolution_hz = 10 * 1000 * 1000;

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init RGB LED: %s", esp_err_to_name(err));
        return false;
    }

    led_strip_clear(s_led_strip);
    return true;
#else
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = 1ULL << BOARD_LED_GPIO;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level(BOARD_LED_GPIO, 0);
    return true;
#endif
}

void board_init(void)
{
    if (!board_led_hw_init())
    {
        ESP_LOGE(TAG, "LED hardware init failed, indicator disabled");
        return;
    }

    s_led_queue = xQueueCreate(4, sizeof(board_led_event_t));
    if (s_led_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LED queue, indicator disabled");
        return;
    }

    xTaskCreate(board_led_task, "board_led", 2048, NULL, 3, NULL);
}

static void board_led_signal(board_led_event_t event)
{
    if (s_led_queue == NULL)
    {
        return;
    }

    /* Never block the caller (mesh callback / sensor task): drop the event
     * if the queue is already full instead of waiting. */
    xQueueSend(s_led_queue, &event, 0);
}

void board_led_signal_tx(void)
{
    board_led_signal(BOARD_LED_EVENT_TX);
}

void board_led_signal_rx(void)
{
    board_led_signal(BOARD_LED_EVENT_RX);
}