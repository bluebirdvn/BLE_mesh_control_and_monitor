#pragma once

#include <memory>
#include "aht30_sensor.hpp"
#include "bh1750.hpp"
#include "vendor_model.h"

/**
 * @brief Reads every physical sensor on the node and builds the
 *        sensor_data_t payload that gets sent over BLE Mesh.
 */
class SensorManager
{
public:
    SensorManager(i2c_master_bus_handle_t i2c_bus, uint8_t sensor_id);

    /**
     * @brief Initialize every sensor. Uses the common ISensor interface so
     *        AHT30 (temperature + humidity) and BH1750 (lux) are all
     *        initialized the exact same way.
     * @return true if all sensors initialized successfully.
     */
    bool init();

    /**
     * @brief Read every sensor and fill the payload to publish.
     * @param out Filled with the latest readings.
     * @return true if every value was read successfully.
     */
    bool readAll(sensor_data_t &out);

private:
    /* TODO: no motion (PIR) sensor wired yet, reserved for a future addition. */
    uint8_t readMotion();

    uint8_t m_sensor_id;

    std::shared_ptr<AHT30Sensor> m_aht30;
    AHT30TemperatureSensor m_temperature_sensor;
    AHT30HumiditySensor m_humidity_sensor;
    BH1750Sensor m_lux_sensor;

    ISensor *m_sensors[3];
};