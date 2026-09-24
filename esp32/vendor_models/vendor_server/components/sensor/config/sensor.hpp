#pragma once

/**
 * @brief Common interface implemented by every sensor wrapper (AHT30, BH1750, ...).
 *
 * Letting every sensor speak the same interface means the rest of the
 * application (init loops, diagnostics, generic sensor lists) does not need
 * to know which concrete sensor it is talking to.
 */
class ISensor
{
public:
    virtual ~ISensor() = default;

    /**
     * @brief Initialize the underlying hardware.
     * @return true on success, false on failure.
     */
    virtual bool init() = 0;

    /**
     * @brief Read one value from the sensor.
     * @return sensor value, or a negative value on error.
     */
    virtual float readSensor() = 0;
};