idf_component_register (
    SRCS
        "config/aht30_sensor.cpp"
        "config/aht30_sensor.c"
        "config/bh1750_i2c_hal.c"
        "config/bh1750_i2c.c"
        "config/bh1750.cpp"
        "config/sensor_manager.cpp"
    INCLUDE_DIRS
        "config"

    REQUIRES driver freertos nvs_flash
)