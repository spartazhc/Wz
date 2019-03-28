/**
 * @file max44009.h
 *
 * ESP-IDF driver for MAX44009/BME280 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 sheinz (https://github.com/sheinz)
 * Copyright (C) 2018 Ruslan V. Uss (https://github.com/UncleRus)
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DOIOT_H__
#define __DOIOT_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <malloc.h>




#define DOIOT_NORMAL        0
#define DOIOT_OBSERVE       1
#define DOIOT_DISCOVER      2
#define DOIOT_IP_CONNECTED  3
#define DOIOT_REG_SUCCESS   4

#define OBJ_ILLUMINANCE     3301
#define OBJ_TEMPERATURE     3303
#define OBJ_HUMIDITY        3304
#define OBJ_PRESSURE        3323

#define DOIOT_OBJ_NUM 4
// typedef enum {
//     DOIOT_NORMAL        = 0,
//     DOIOT_OBSERVE       = 1,
//     DOIOT_DISCOVER      = 2,
//     DOIOT_IP_CONNECTED  = 3,
//     DOIOT_REG_SUCCESS   = 4
// } doiot_event;

// typedef struct {
//     float value,
//     float 
// } lwm2m_source_t;

// typedef struct {
//     int id,
//     float value,        //5700
//     float max_measure,  //5601
//     float min_measure   //5602
//     // uint8_t reset
// } lwm2m_object_t;

// typedef struct {
//     doiot_event     event,
//     uint8_t        status[8],
//     char   msgid_observe[8],
//     char   msgid_discover[8],
//     char   is_gp2y,
//     // lwm2m_object_t  illuminance,    //3301
//     // lwm2m_object_t  temperature,    //3303
//     // lwm2m_object_t  humidity,       //3304
//     // lwm2m_object_t  pressure,       //3323
// } doiot_t;

// typedef struct doiot_obj_ *doiot_object;
typedef struct {
    char id[10];
    float value;
    bool observe;
    bool discover;
    char msgid_observe[10];
    char msgid_discover[10];
} doiot_obj_t;


typedef struct {
    int event;      // DOIOT_EVENT_
    int cur_obj;    // save current obj index
    // char args[10];  // args for DOIOT_EVENT_
    bool flag_ip;
    bool flag_miplopen;
    bool upload_en;
    int observe_count;
    int discover_count;
} doiot_event_t;

// doiot_event_t* get_doiot();




#endif  // __MAX44009_H__
