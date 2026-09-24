#pragma once

#include <memory>
#include "sensor.hpp"
#include "aht30_sensor.h"

/**
 * @brief Owns the physical AHT30 I2C device and performs the real reads.
 *
 * One AHT30 chip produces two independent values (temperature and
 * humidity), so this class is not an ISensor itself. AHT30TemperatureSensor
 * and AHT30HumiditySensor below wrap a shared instance of this class to
 * expose each value through the common ISensor interface, while the I2C
 * device is only added to the bus once.
 */
class AHT30Sensor
{
public:
    AHT30Sensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x38);
    ~AHT30Sensor();

    /**
     * @brief Initialize the sensor. Safe to call more than once.
     */
    bool init();

    bool readTemperature(float &temperature);
    bool readHumidity(float &humidity);

private:
    bool read(float &temperature, float &humidity);

    aht_init_config_t m_config;
    aht_handle_t *m_handle{nullptr};
};

/**
 * @brief Exposes AHT30 temperature through the ISensor interface.
 */
class AHT30TemperatureSensor : public ISensor
{
public:
    explicit AHT30TemperatureSensor(std::shared_ptr<AHT30Sensor> sensor);

    bool init() override;
    float readSensor() override;

private:
    std::shared_ptr<AHT30Sensor> m_sensor;
};

/**
 * @brief Exposes AHT30 humidity through the ISensor interface.
 */
class AHT30HumiditySensor : public ISensor
{
public:
    explicit AHT30HumiditySensor(std::shared_ptr<AHT30Sensor> sensor);

    bool init() override;
    float readSensor() override;

private:
    std::shared_ptr<AHT30Sensor> m_sensor;
};