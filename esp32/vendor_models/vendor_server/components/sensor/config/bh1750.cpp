#include "bh1750.hpp"
#include "esp_log.h"

static const char *TAG = "BH1750_CPP";

BH1750Sensor::BH1750Sensor(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : m_i2c_bus(i2c_bus), m_addr(addr)
{
    m_dev.dev_handle = nullptr;
    m_dev.mtreg_val = DEFAULT_MEAS_TIME_REG_VAL;
    m_dev.meas_time = H_RES_MODE_MEASUREMENT_TIME_MS;
    m_dev.meas_time_mul = 1;
}

BH1750Sensor::~BH1750Sensor()
{
    if (m_dev.dev_handle != nullptr)
    {
        i2c_master_bus_rm_device(m_dev.dev_handle);
        m_dev.dev_handle = nullptr;
    }
}

bool BH1750Sensor::init()
{
    if (m_dev.dev_handle == nullptr)
    {
        if (bh1750_i2c_hal_init(m_i2c_bus, m_addr, 400000, &m_dev.dev_handle) != BH1750_OK)
        {
            ESP_LOGE(TAG, "Failed to add BH1750 on I2C bus");
            return false;
        }
    }

    if (bh1750_i2c_set_power_mode(m_dev, BH1750_POWER_ON) != BH1750_OK)
    {
        ESP_LOGE(TAG, "Power ON failed");
        return false;
    }

    if (bh1750_i2c_set_resolution_mode(&m_dev, BH1750_CONT_H_RES_MODE) != BH1750_OK)
    {
        ESP_LOGE(TAG, "Set resolution mode failed");
        return false;
    }

    return true;
}

float BH1750Sensor::readSensor()
{
    uint16_t lux_val = 0;

    if (bh1750_i2c_read_data(m_dev, &lux_val) != BH1750_OK)
    {
        ESP_LOGE(TAG, "Read lux failed");
        return -1.0f;
    }

    return static_cast<float>(lux_val);
}