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
#include "max44009.h"

#include <esp_log.h>

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf

static const char *TAG = "MAX44009";

/**
 * MAX44009 registers
 */
#define MAX44009_REG_INT_STATUS  0x00 /* bits: 0 */
#define MAX44009_REG_INT_ENABLE  0x01 /* bits: 0 */
/* CONT MANUAL - - CDR TIM[2:0] */
#define MAX44009_REG_CONFIG      0x02 /* Power on state: 0x03 */
#define MAX44009_REG_LUX_H       0x03 /* E: bits: 7-4; M: 3-0 */
#define MAX44009_REG_LUX_L	     0x04 /* bits: 3-0 */
#define MAX44009_REG_TH_H 	     0x05 /* E: bits: 7-4; M: 3-0 */
#define MAX44009_REG_TH_L        0x06 /* E: bits: 7-4; M: 3-0 */
#define MAX44009_REG_TH_T        0x07 

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

// static esp_err_t read_register16(i2c_dev_t *dev, uint8_t reg, uint16_t *r)
// {
//     uint8_t d[] = { 0, 0 };

//     CHECK(i2c_dev_read_reg(dev, reg, d, 2));
//     *r = d[0] | (d[1] << 8);

//     return ESP_OK;
// }

inline static esp_err_t write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    return i2c_dev_write_reg(dev, addr, &value, 1);
}





esp_err_t max44009_init_desc(max44009_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != MAX44009_I2C_ADDRESS_0 && addr != MAX44009_I2C_ADDRESS_1)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;

    CHECK(i2c_dev_create_mutex(&dev->i2c_dev));

    return ESP_OK;
}

esp_err_t max44009_free_desc(max44009_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t max44009_init_default_params(max44009_params_t *params)
{
    CHECK_ARG(params);

    params->int_enable = MAX44009_INT_ENABLE;
    params->mode = MAX44009_MODE_DEFAULT;
    params->manual = MAX44009_MANUAL_ENABLE;
    params->cdr = MAX44009_CDR_DISABLE;
    params->integ_time = MAX44009_INTEGRATION_100;
    params->eth_h = MAX44009_TH_1110;   /* not important */
    params->eth_l = MAX44009_TH_0000;
    return ESP_OK;
}

esp_err_t max44009_init_auto_params(max44009_params_t *params)
{
    CHECK_ARG(params);

    params->int_enable = MAX44009_INT_DISABLE;
    params->mode = MAX44009_MODE_DEFAULT;
    params->manual = MAX44009_MANUAL_DISABLE;
    params->cdr = MAX44009_CDR_DISABLE;
    params->integ_time = MAX44009_INTEGRATION_800;
    params->eth_h = MAX44009_TH_1110;   /* not important */
    params->eth_l = MAX44009_TH_0000;
    return ESP_OK;
}

esp_err_t max44009_init(max44009_t *dev, max44009_params_t *params)
{
    CHECK_ARG(dev);
    CHECK_ARG(params);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "Writing config interrupt reg=%x", params->int_enable);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_INT_ENABLE, params->int_enable), "Failed setting interrupt bit");

    uint8_t config = (params->mode << 7) | (params->manual << 6) | 
                        (params->cdr << 3) | (params->integ_time);
    ESP_LOGD(TAG, "Writing config configure reg=%x", config);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_CONFIG, config), "Failed configuring sensor");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}
esp_err_t max44009_read_regs(max44009_t *dev)
{
    uint8_t tmp;
    i2c_dev_read_reg(dev, MAX44009_REG_INT_ENABLE, &tmp, 1);
    ESP_LOGI(TAG, "intr enable reg: %x", tmp);
    i2c_dev_read_reg(dev, MAX44009_REG_INT_STATUS, &tmp, 1);
    ESP_LOGI(TAG, "intr status reg: %x", tmp);
    i2c_dev_read_reg(dev, MAX44009_REG_CONFIG, &tmp, 1);
    ESP_LOGI(TAG, "config reg: %x", tmp);
    i2c_dev_read_reg(dev, MAX44009_REG_TH_H, &tmp, 1);
    ESP_LOGI(TAG, "threshold_h reg: %x", tmp);
    i2c_dev_read_reg(dev, MAX44009_REG_TH_L, &tmp, 1);
    ESP_LOGI(TAG, "threshold_l reg: %x", tmp);
    i2c_dev_read_reg(dev, MAX44009_REG_TH_T, &tmp, 1);
    ESP_LOGI(TAG, "threshold_t %x", tmp);

    return ESP_OK;
}
    
esp_err_t max44009_set_threshold_etc(max44009_t *dev, max44009_params_t *params, float lux_f, uint8_t lux_raw)
{   
    int lux = (int) lux_f;
    uint8_t tmp = 0;
    uint8_t e = (lux_raw >> 4) & 0b1111;
    uint8_t m = lux_raw & 0b1111;


    if (e == 0b1111) e--; //avoid overload
    if (lux < 2938) {
        tmp = 0;
    } else if (lux < 5875) {
        tmp = 1;
	} else if (lux < 11750) {
        tmp = 2;
	} else if (lux < 23501) {
        tmp = 3;
	} else if (lux < 47002) {
        tmp = 4;
	} else if (lux < 94003) {
        tmp = 5;
	} else if (lux < 188007) {
        tmp = 6;
	} 
    
    uint8_t config = (params->mode << 7) | (params->manual << 6) | 
                        (params->cdr << 3) | (tmp);
    ESP_LOGI(TAG, "Writing config configure reg=%x", config);

    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_CONFIG, config), "Failed configuring sensor");
    
    uint8_t th[] = {0, 0, 0x0A};//0x01100100};//64+32+4 = 100*100ms=10s
    if (m == 0b1111) {
        th[0] = ((e+1) << 4) | 0b0000;
        th[1] = (e << 4 | (m-1));
    } else if (m == 0b0000) {
        th[0] = (e << 4) | (m+1);
        th[1] = ((e-1) << 4) | 0b1111;
    } else {
        th[0] = (e << 4) | (m+1);
        th[1] = (e << 4) | (m-1);
    }

    // uint8_t th[] = {(1 << 7) | (tmp << 4) | 0b0111,     /* th_h */
    //                            (tmp << 4) | 0b0000,     /* th_l */
    //                  0x08};                             /* th_timer */
    ESP_LOGI(TAG, "Writing threshold configure reg=%02x,%02x,%02x", th[0], th[1], th[2]);

    // CHECK_LOGE(dev, i2c_dev_write(&dev->i2c_dev, (const void *)MAX44009_REG_TH_H, 1, th, 3), "Failed configuring th_h/l/T");
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_TH_H, th[0]), "Failed configuring th_h");
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_TH_L, th[1]), "Failed configuring th_l");
    CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_TH_T, th[2]), "Failed configuring th_t");
    return ESP_OK;
}

// esp_err_t max44009_force_measurement(max44009_t *dev)
// {
//     CHECK_ARG(dev);

//     I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

//     uint8_t ctrl;
//     I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, MAX44009_REG_CTRL, &ctrl, 1));
//     ctrl &= ~0b11;  // clear two lower bits
//     ctrl |= MAX44009_MODE_FORCED;
//     ESP_LOGD(TAG, "Writing ctrl reg=%x", ctrl);
//     CHECK_LOGE(dev, write_register8(&dev->i2c_dev, MAX44009_REG_CTRL, ctrl), "Failed starting forced mode");

//     I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

//     return ESP_OK;
// }
// static esp_err_t read_register16(i2c_dev_t *dev, uint8_t reg, uint16_t *r)
// {
//     uint8_t d[] = { 0, 0 };

//     CHECK(i2c_dev_read_reg(dev, reg, d, 2));
//     *r = d[0] | (d[1] << 8);

//     return ESP_OK;
// }

esp_err_t max44009_read_float(max44009_t *dev, float *lux_f, uint8_t *lux_raw)
{
    // uint8_t lux[] = {0, 0};
    uint8_t lux_h, lux_l;
    CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, MAX44009_REG_LUX_H, &lux_h, 1), "Failed reading");
    CHECK_LOGE(dev, i2c_dev_read_reg(&dev->i2c_dev, MAX44009_REG_LUX_L, &lux_l, 1), "Failed reading");
    // uint16_t lux;
    // CHECK_LOGE(dev, read_register16(&dev->i2c_dev, MAX44009_REG_LUX_H, &lux), "Failed reading");

    // ESP_LOGI(TAG, "max44009 raw data %x, %x", lux_h, lux_l);
    uint8_t exponent = (lux_h & 0xf0) >> 4;
    uint8_t mant = (lux_h & 0x0f) << 4 | lux_l;
    *lux_f = (float)(((0x00000001 << exponent) * (float)mant) * 0.045);
    *lux_raw = (lux_l << 8) | lux_h;
    // CHECK_LOGE(dev, max44009_set_threshold_etc(dev, params, (int32_t)*lux_f), "Failed setting threshold");

    return ESP_OK;
}

// esp_err_t max44009_int_status_check