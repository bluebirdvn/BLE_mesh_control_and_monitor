#ifndef OPCODE_HPP
#define OPCODE_HPP

#pragma once
#include <cstdint>

#define MSG_ARG_NONE 0

enum class OpCode : uint8_t {
    CMD_PROV_ENABLE         = 0x01,
    CMD_PROV_DISABLE        = 0x02,
    CMD_ADD_UNPROV_DEV      = 0x03,
    CMD_SET_DEV_UUID_MATCH  = 0x04,
    CMD_DELETE_NODE         = 0x06,

    CMD_GROUP_ADD           = 0x0A,
    CMD_GROUP_DELETE        = 0x0B,
    CMD_MODEL_PUB_SET       = 0x0C,

    CMD_SENSOR_GET          = 0x30,
    CMD_ACTUATOR_SET        = 0x32,
    CMD_THRESHOLD_CONFIG    = 0x34,

    EVT_RECV_UNPROV_ADV_PKT = 0x82,
    EVT_PROV_COMPLETE       = 0x85,
    EVT_NODE_RESET          = 0x50,

    EVT_SENSOR_STATUS       = 0xB1,
    EVT_ACTUATOR_STATUS     = 0xB2,

    EVT_MODEL_SUBSCRIBE_STATUS  = 0xD0,
    EVT_MODEL_UNSUBSCRIBE_STATUS= 0xD1,
    EVT_MODEL_PUBLISH_STATUS    = 0xD2,
    EVT_HEARTBEAT               = 0xD3,
};

#pragma pack(push, 1)

struct mesh_cmd_threshold_t {
    uint16_t actuator_id;
    float    threshold_on;
    float    threshold_off;
    uint8_t  type;
};

struct mesh_cmd_add_unprov_dev_t {
    uint8_t uuid[16];
    uint8_t bearer; 
};

struct mesh_cmd_set_uuid_match_t {
    uint8_t uuid_match[8]; 
};

struct mesh_cmd_add_dev_to_group_t {
    uint16_t element_addr; 
    uint16_t group_addr;   
    uint16_t company_id;   
    uint16_t model_id;     
};

struct mesh_cmd_remove_dev_from_group_t {
    uint16_t element_addr; 
    uint16_t group_addr;   
    uint16_t company_id;  
    uint16_t model_id;     
};

struct mesh_cmd_model_pub_set_t {
    uint16_t element_addr;  
    uint16_t pub_addr;      
    uint16_t company_id;   
    uint16_t model_id;      
    uint8_t  pub_ttl;      
    uint8_t  pub_period;    
};

struct mesh_cmd_sensor_get_t {
    uint16_t sensor_id;  
};

struct mesh_evt_sensor_status_t {
    uint16_t sensor_id;   
    uint8_t  sensor_type;
    uint16_t lux;         
    int16_t  temperature; 
    uint8_t  humidity;    
    uint8_t  motion;      
    uint8_t  battery;     
    uint8_t  status;      
};

struct mesh_cmd_actuator_set_t {
    uint16_t actuator_id; 
    uint16_t setpoint;    
    uint8_t  onoff;       
};

struct mesh_evt_actuator_status_t {
    uint16_t actuator_id;      
    uint8_t  actuator_type;
    uint16_t present_setpoint; 
    uint16_t target_setpoint;  
    uint8_t  present_onoff;    
    uint8_t  target_onoff;     
    uint8_t  status;           
};

struct mesh_evt_unprov_adv_t {
    uint8_t  uuid[16];    
    uint16_t oob_info;    
    uint8_t  bearer;      
    int8_t   rssi;        
};

struct mesh_evt_prov_complete_t {
    uint16_t net_idx;     
    uint8_t  elem_num;    
    uint8_t  uuid[16];    
};

struct mesh_evt_heartbeat_t {
    uint8_t  init_ttl; 
    uint8_t  hops;     
    uint16_t features; 
};

struct mesh_evt_group_status {
    uint16_t element_addr;
    uint16_t group_addr;
    uint16_t model_id;
    bool     is_sub;
    bool     is_add;
    bool     success;
};

#pragma pack(pop)

#define PAYLOAD_SIZE_CMD_PROV_ENABLE        0                                        
#define PAYLOAD_SIZE_CMD_PROV_DISABLE       0                                        
#define PAYLOAD_SIZE_CMD_DELETE_NODE        0                                        
#define PAYLOAD_SIZE_CMD_ADD_UNPROV_DEV     sizeof(mesh_cmd_add_unprov_dev_t)        
#define PAYLOAD_SIZE_CMD_SET_UUID_MATCH     sizeof(mesh_cmd_set_uuid_match_t)        
#define PAYLOAD_SIZE_CMD_GROUP_ADD          sizeof(mesh_cmd_add_dev_to_group_t)      
#define PAYLOAD_SIZE_CMD_GROUP_DELETE       sizeof(mesh_cmd_remove_dev_from_group_t) 
#define PAYLOAD_SIZE_CMD_MODEL_PUB_SET      sizeof(mesh_cmd_model_pub_set_t)

#define PAYLOAD_SIZE_CMD_SENSOR_GET         sizeof(mesh_cmd_sensor_get_t)            
#define PAYLOAD_SIZE_CMD_ACTUATOR_SET       sizeof(mesh_cmd_actuator_set_t)          

#define PAYLOAD_SIZE_EVT_UNPROV_ADV         sizeof(mesh_evt_unprov_adv_t)            
#define PAYLOAD_SIZE_EVT_PROV_COMPLETE      sizeof(mesh_evt_prov_complete_t)         
#define PAYLOAD_SIZE_EVT_SENSOR_STATUS      sizeof(mesh_evt_sensor_status_t)         
#define PAYLOAD_SIZE_EVT_ACTUATOR_STATUS    sizeof(mesh_evt_actuator_status_t)       
#define PAYLOAD_SIZE_EVT_HEARTBEAT          sizeof(mesh_evt_heartbeat_t)             
#define PAYLOAD_SIZE_EVT_GROUP_STATUS       sizeof(mesh_evt_group_status)
#define PAYLOAD_THRESHOLD_CONFIG            sizeof(mesh_cmd_threshold_t)
#define PAYLOAD_SIZE_MAX                    sizeof(mesh_evt_sensor_status_t)         

#endif