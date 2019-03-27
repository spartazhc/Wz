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
#ifndef __MAX44009_H__
#define __MAX44009_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * MAX44009 or BME280 address is 0x77 if SDO pin is high, and is 0x76 if
 * SDO pin is low.
 */
#define MAX44009_I2C_ADDRESS_0  0x4A
#define MAX44009_I2C_ADDRESS_1  0x4B

// #define MAX44009_CHIP_ID  0x58 /* MAX44009 has chip-id 0x58 */
// #define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */

/**
 * Mode of MAX44009 module operation.
 * Forced - Measurement is initiated by user.
 * Normal - Continues measurement.
 */
typedef enum {
    MAX44009_INT_DISABLE = 0,
    MAX44009_INT_ENABLE = 1
} MAX44009_IntEnable;

typedef enum {
    MAX44009_MODE_DEFAULT = 0,
    MAX44009_MODE_CONT = 1
} MAX44009_Mode;

typedef enum {
    MAX44009_MANUAL_DISABLE = 0,
    MAX44009_MANUAL_ENABLE = 1
} MAX44009_Manual;

typedef enum {
    MAX44009_CDR_DISABLE = 0,
    MAX44009_CDR_8 = 1
} MAX44009_CDR;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    MAX44009_INTEGRATION_800 = 0,     /* This is a preferred mode for boosting low-light sensitivity */
    MAX44009_INTEGRATION_400 = 1,     /* -- */
    MAX44009_INTEGRATION_200 = 2,     /* -- */
    MAX44009_INTEGRATION_100 = 3,     /* This is a preferred mode for high-brightness applications */
    MAX44009_INTEGRATION_50 = 4,      /* Manual mode only */
    MAX44009_INTEGRATION_25 = 5,      /* Manual mode only */
    MAX44009_INTEGRATION_12 = 6,      /* Manual mode only */
    MAX44009_INTEGRATION_6 = 7,       /* Manual mode only */
} MAX44009_IntegrationTime;

typedef enum {
    MAX44009_TH_0000 = 0,     
    MAX44009_TH_0001 = 1,     
    MAX44009_TH_0010 = 2,     
    MAX44009_TH_0011 = 3,     
    MAX44009_TH_0100 = 4,     
    MAX44009_TH_0101 = 5,     
    MAX44009_TH_0110 = 6,      
    MAX44009_TH_0111 = 7,  
    MAX44009_TH_1000 = 8,     
    MAX44009_TH_1001 = 9,     
    MAX44009_TH_1010 = 10,     
    MAX44009_TH_1011 = 11,     
    MAX44009_TH_1100 = 12,     
    MAX44009_TH_1101 = 13,     
    MAX44009_TH_1110 = 14,            
} MAX44009_ThresholdTimeE;

/**
 * Configuration parameters for MAX44009 module.
 * Use function max44009_init_default_params to use default configuration.
 */
typedef struct {
    MAX44009_IntEnable  int_enable;
    MAX44009_Mode       mode;
    MAX44009_Manual     manual;
    MAX44009_CDR        cdr;
    MAX44009_IntegrationTime integ_time;
    MAX44009_ThresholdTimeE eth_l;
    MAX44009_ThresholdTimeE eth_h;
} max44009_params_t;
/* CONT MANUAL - - CDR TIM[2:0] */

typedef struct {
    i2c_dev_t  i2c_dev;  /* I2C dev setting. */
} max44009_t;

/**
 * @brief Initialize device descriptior
 * @param[out] dev Pointer to device descriptor
 * @param[in] addr MAX44009 address
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin for SDA
 * @param[in] scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t max44009_init_desc(max44009_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t max44009_free_desc(max44009_t *dev);

/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
esp_err_t max44009_init_default_params(max44009_params_t *params);


/**
 * Initialize MAX44009 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns true on success otherwise false.
 *
 * The I2C address is assumed to have been initialized in the dev, and
 * may be either MAX44009_I2C_ADDRESS_0 or MAX44009_I2C_ADDRESS_1. If the I2C
 * address is unknown then try initializing each in turn.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
esp_err_t max44009_init(max44009_t *dev, max44009_params_t *params);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */
esp_err_t max44009_read_float(max44009_t *dev, float *lux_f, uint8_t *lux_raw);


esp_err_t max44009_set_threshold_etc(max44009_t *dev, max44009_params_t *params, float lux_f, uint8_t lux_raw);

esp_err_t max44009_read_regs(max44009_t *dev);
#ifdef __cplusplus
}
#endif

#endif  // __MAX44009_H__
