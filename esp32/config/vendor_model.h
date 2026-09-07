#ifndef _VENDOR_MODEL_H
#define _VENDOR_MODEL_H

#include <stdint.h>

#define CID_ESP 0x02E5

#define VND_MODEL_ID_SENSOR   0x0001
#define VND_MODEL_ID_ACTUATOR 0x0002

#define VND_OP_SENSOR_GET             ESP_BLE_MESH_VND_MODEL_OP_3(0x05, CID_ESP)
#define VND_OP_SENSOR_STATUS          ESP_BLE_MESH_VND_MODEL_OP_3(0x06, CID_ESP)
#define VND_OP_SENSOR_THRESHOLD_SET   ESP_BLE_MESH_VND_MODEL_OP_3(0x07, CID_ESP)
#define VND_OP_ACTUATOR_SET           ESP_BLE_MESH_VND_MODEL_OP_3(0x03, CID_ESP)
#define VND_OP_ACTUATOR_STATUS        ESP_BLE_MESH_VND_MODEL_OP_3(0x04, CID_ESP)

#pragma pack(push, 1)

typedef struct {
    uint8_t  sensor_id;
    uint8_t  soil_moisture;
    int8_t   temperature;
    uint8_t  humidity;
    uint16_t lux;
    uint8_t  battery;
} sensor_data_t;

typedef struct {
    uint8_t  actuator_id;
    uint8_t  device_type;
    uint8_t  power;
    uint8_t  target_temp;
    uint8_t  fan_speed;
    uint8_t  position;
} vnd_actuator_set_t;

typedef struct {
    uint16_t sensor_id;
    float    threshold_on;
    float    threshold_off;
    uint8_t  type;
} vnd_sensor_threshold_t;

#pragma pack(pop)

#endif 