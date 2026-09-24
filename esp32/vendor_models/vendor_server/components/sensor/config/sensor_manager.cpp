#include "sensor_manager.hpp"
#include "esp_log.h"

static const char *TAG = "SENSOR_MANAGER";

SensorManager::SensorManager(i2c_master_bus_handle_t i2c_bus, uint8_t sensor_id)
    : m_sensor_id(sensor_id),
      m_aht30(std::make_shared<AHT30Sensor>(i2c_bus, 0x38)),
      m_temperature_sensor(m_aht30),
      m_humidity_sensor(m_aht30),
      m_lux_sensor(i2c_bus),
      m_sensors{&m_temperature_sensor, &m_humidity_sensor, &m_lux_sensor}
{
}

bool SensorManager::init()
{
    for (ISensor *sensor : m_sensors)
    {
        if (!sensor->init())
        {
            ESP_LOGE(TAG, "A sensor failed to initialize");
            return false;
        }
    }

    return true;
}

uint8_t SensorManager::readMotion()
{
    /* TODO: read the real PIR GPIO once the motion sensor is wired up. */
    return 0;
}

bool SensorManager::readAll(sensor_data_t &out)
{
    float temperature = 0.0f;
    float humidity = 0.0f;

    bool aht_ok = m_aht30->readTemperature(temperature) && m_aht30->readHumidity(humidity);
    if (!aht_ok)
    {
        ESP_LOGE(TAG, "Failed to read AHT30");
        return false;
    }

    float lux = m_lux_sensor.readSensor();
    if (lux < 0.0f)
    {
        ESP_LOGE(TAG, "Failed to read BH1750");
        return false;
    }

    // out.sensor_id = m_sensor_id;
    out.soil_moisture = 0; /* TODO: no soil moisture sensor wired yet. */
    out.temperature = (int8_t)temperature;
    out.humidity = (uint8_t)humidity;
    out.lux = (lux > 0xFFFF) ? 0xFFFF : (uint16_t)lux;
    out.battery = 100; /* TODO: read the real battery level. */

    return true;
}