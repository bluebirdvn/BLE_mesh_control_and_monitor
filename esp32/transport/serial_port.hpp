#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include "iserial_port.hpp"
#include <string>


class SerialPort : public ISerialPort {
public:
    explicit SerialPort(const uart_config_t& config);
    ~SerialPort() override;

    communication_status_t open() override;
    communication_status_t close() override;
    int read_byte(uint8_t& out) override;
    int write_bytes(const std::vector<uint8_t>& data) override;
    bool is_open() const override { 
        return true;
    }

private:
    uart_config_t uart_config;

};

#endif
