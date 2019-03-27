/**
 * @file max44009.c
 *
 * ESP-IDF driver for MAX44009/BME280 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016 sheinz (https://github.com/sheinz)
 * Copyright (C) 2018 Ruslan V. Uss (https://github.com/UncleRus)
 * MIT Licensed as described in the file LICENSE
 */
#include "doiot.h"

// const char* obj_list[DOIOT_OBJ_NUM] = {"3301", "3303", "3304", "3323"};
// doiot_event_t* doiot = NULL;

doiot_event_t doiot = {DOIOT_NORMAL, -1, 0, 0, 0, 0, 0};
doiot_obj_t obj[DOIOT_OBJ_NUM] = {
    {"3301", 0, 0, 0, "", ""},
    {"3303", 0, 0, 0, "", ""},
    {"3304", 0, 0, 0, "", ""},
    {"3323", 0, 0, 0, "", ""}
};

// doiot_obj_t* get_obj(const char* id)
// {
//     doiot_obj_t* obj = (doiot_obj_t*)malloc(sizeof(doiot_obj_t));
//     strcpy(obj->id, id);
//     obj->value = 0;
//     obj->observe = 0;
//     obj->discover = 0;
//     obj->msgid_observe[0] = '\0';
//     obj->msgid_discover[0] = '\0';
//     return obj;
// }


// doiot_event_t* get_doiot()
// {
//     if (doiot != NULL) return doiot;
//     doiot_event_t* doiot = (doiot_event_t*)malloc(sizeof(doiot_event_t));
//     doiot->event = DOIOT_NORMAL;
//     // doiot->args[0] = '\0';
//     doiot->cur_obj = -1;
//     doiot->flag_ip = 0;
//     doiot->flag_miplopen = 0;
//     return doiot;
// }




