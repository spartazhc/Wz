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
#ifndef __ME3616_H__
#define __ME3616_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <malloc.h>
#include "esp_err.h"

#define GPIO_PWR_ME3616     4
#define GPIO_RESET_ME3616   0

#define ME3616_NORMAL        0
#define ME3616_OBSERVE       1
#define ME3616_DISCOVER      2
#define ME3616_IP_CONNECTED  3
#define ME3616_REG_SUCCESS   4

#define OBJ_ILLUMINANCE     3301
#define OBJ_TEMPERATURE     3303
#define OBJ_HUMIDITY        3304
#define OBJ_PRESSURE        3323

#define ME3616_OBJ_NUM 4


typedef struct {
    char id[10];
    float value;
    bool observe;
    bool discover;
    char msgid_observe[10];
    char msgid_discover[10];
} me3616_obj_t;


typedef struct {
    int event;      // ME3616_EVENT_
    int cur_obj;    // save current obj index
    // char args[10];  // args for ME3616_EVENT_
    bool flag_ip;
    bool flag_miplopen;
    bool upload_en;
    int observe_count;
    int discover_count;
} me3616_event_t;



int me3616_send_cmd(char* str, bool flag_ok, int waitms);

void me3616_power_on(void);
void me3616_hardware_reset(void);
void me3616_wake_up(void);

int me3616_sleep_config(int mode);

#endif  
