#ifndef CONFIG_H
#define CONFIG_H

#define UART_PORT_NUM      1
#define UART_BAUD_RATE     115200
#define BUF_SIZE           1024
#define BUFFER_MAX         10
#define MSG_ARG_NONE       0

#define TX_UART_NUM         17
#define RX_UART_NUM         16

constexpr uint16_t PROV_OWN_ADDR = 0x0001;
constexpr uint16_t PROV_START_ADDRESS = 0x0005;

#endif