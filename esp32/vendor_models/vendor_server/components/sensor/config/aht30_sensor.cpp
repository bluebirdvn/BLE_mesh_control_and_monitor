#include "aht30_sensor.hpp"
#include "esp_log.h"

static const char *TAG = "AHT30_CPP";

AHT30Sensor::AHT30Sensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
{
    m_config.i2c_handle = i2c_bus;
    m_config.i2c_address = addr;
    m_config.i2c_frequency = 400000;
}

AHT30Sensor::~AHT30Sensor()
{
    if (m_handle != nullptr)
    {
        aht_deinit(&m_handle);
    }
}

bool AHT30Sensor::init()
{
    if (m_handle != nullptr)
    {
        return true;
    }

    esp_err_t err = aht_init(&m_config, &m_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Init AHT30 failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool AHT30Sensor::read(float &temperature, float &humidity)
{
    if (m_handle == nullptr)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return false;
    }

    esp_err_t err = aht_read(&m_handle, &humidity, &temperature);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Read AHT30 failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool AHT30Sensor::readTemperature(float &temperature)
{
    float humidity = 0.0f;
    return read(temperature, humidity);
}

bool AHT30Sensor::readHumidity(float &humidity)
{
    float temperature = 0.0f;
    return read(temperature, humidity);
}

AHT30TemperatureSensor::AHT30TemperatureSensor(std::shared_ptr<AHT30Sensor> sensor)
    : m_sensor(std::move(sensor))
{
}

bool AHT30TemperatureSensor::init()
{
    return m_sensor->init();
}

float AHT30TemperatureSensor::readSensor()
{
    float temperature = -1.0f;

    if (!m_sensor->readTemperature(temperature))
    {
        return -1.0f;
    }

    return temperature;
}

AHT30HumiditySensor::AHT30HumiditySensor(std::shared_ptr<AHT30Sensor> sensor)
    : m_sensor(std::move(sensor))
{
}

bool AHT30HumiditySensor::init()
{
    return m_sensor->init();
}

float AHT30HumiditySensor::readSensor()
{
    float humidity = -1.0f;

    if (!m_sensor->readHumidity(humidity))
    {
        return -1.0f;
    }

    return humidity;
}