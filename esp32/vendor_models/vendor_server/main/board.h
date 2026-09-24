#pragma once

#include "driver/gpio.h"
#include "sdkconfig.h"
#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------------
 * I2C pins per target board. Adjust to match your actual wiring/devkit.
 * ---------------------------------------------------------------------- */
#if CONFIG_IDF_TARGET_ESP32
#define BOARD_I2C_SDA_IO GPIO_NUM_21
#define BOARD_I2C_SCL_IO GPIO_NUM_22
#elif CONFIG_IDF_TARGET_ESP32H2
#define BOARD_I2C_SDA_IO GPIO_NUM_1
#define BOARD_I2C_SCL_IO GPIO_NUM_0
#elif CONFIG_IDF_TARGET_ESP32C6
#define BOARD_I2C_SDA_IO GPIO_NUM_6
#define BOARD_I2C_SCL_IO GPIO_NUM_7
#else
#error "Unsupported target: add I2C pin definitions for it in board.h"
#endif

/* ------------------------------------------------------------------------
 * LED signaling.
 *
 * - ESP32 devkits only have a single mono LED: RX = 1 flash, TX = 2 quick
 *   flashes, so the two events stay distinguishable without color.
 * - ESP32-H2 / ESP32-C6 devkits have an onboard RGB (WS2812) LED: RX flashes
 *   green, TX flashes blue.
 *
 * Both board_led_signal_tx()/rx() are non-blocking: they only queue an
 * event for a dedicated LED task, so they are safe to call from BLE Mesh
 * callbacks or the sensor task without stalling them.
 * ---------------------------------------------------------------------- */
typedef enum
{
    BOARD_LED_EVENT_RX,
    BOARD_LED_EVENT_TX,
} board_led_event_t;

/**
 * @brief Initialize board peripherals: LED indicator (and its background task).
 *        Must be called once before board_led_signal_tx()/rx().
 */
void board_init(void);

/**
 * @brief Signal that data was sent (non-blocking).
 */
void board_led_signal_tx(void);

/**
 * @brief Signal that data was received (non-blocking).
 */
void board_led_signal_rx(void);

#ifdef __cplusplus
}
#endif