#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include <esp_system.h>
#include <bmp280.h>
#include <max44009.h>
//adc
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "rom/ets_sys.h"

#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_INTR_IO            5  // INTR pin for max44009
#define ESP_INTR_FLAG_DEFAULT   0   
#define GPIO_LED_CONTROL        25  // LED control pin for gp2y1014au
#define GPIO_UV_EN              0

// adc 
#define DEFAULT_VREF        1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES       8          //Multisampling

// gp2y time define in microsecoend !
#define GP2Y_SAMPLE_TIME    280
#define GP2Y_DELTA_TIME     40
#define GP2Y_SLEEP_TIME     9680

// 
SemaphoreHandle_t xSemaphore = NULL;
esp_err_t res;

//bmp280 & max44009
bmp280_params_t params_b;
max44009_params_t params_m;
bmp280_t dev_b;
max44009_t dev_m;

// adc
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel2 = ADC_CHANNEL_7;     //GPIO35
// static const adc_channel_t channel3 = ADC_CHANNEL_4;     //GPIO32
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// interrupt service routine, called when the button is pressed
void IRAM_ATTR max44009_isr_handler(void* arg) {
	
    // notify the button task
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

/**
 * depend on 
 * static esp_adc_cal_characteristics_t *adc_chars;
 * static const adc_channel_t channel = ADC_CHANNEL_6;
 */
uint32_t analog_read(adc_channel_t channel)
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    return voltage;
}

void init_adc()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel1, atten);
        adc1_config_channel_atten(channel2, atten);
    } else {
        // adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

/**
 * init gpio
 * GPIO_INTR_IO         | max44009 
 * GPIO_LED_CONTROL     | gp2y1014au
 */
void init_gpio()
{
    gpio_config_t io_conf;
    //enable interrupt as negedge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_INTR_IO);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << GPIO_LED_CONTROL) | (1ULL << GPIO_UV_EN));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}


void init_bme280()
{
    // bmp280_init_default_params(&params_b);
    bmp280_init_weather_params(&params_b);   
    
    while (bmp280_init_desc(&dev_b, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = bmp280_init(&dev_b, &params_b)) != ESP_OK)
    {
        printf("Could not init BMP280, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    // bool bme280p = dev.id == BME280_CHIP_ID;
    // printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void init_max44009()
{
    max44009_init_default_params(&params_m);   
    // dev_m.i2c_dev = dev_b.i2c_dev;
    while (max44009_init_desc(&dev_m, MAX44009_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = max44009_init(&dev_m, &params_m)) != ESP_OK)
    {
        printf("Could not init MAX44009, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    float lux;
    uint8_t lux_raw;
    if (max44009_read_float(&dev_m, &lux, &lux_raw) != ESP_OK)
    {
        printf("Temperature/pressure reading failed\n");
    }
    printf("init Lux: %.3f\n", lux);
    if (max44009_set_threshold_etc(&dev_m, &params_m, lux, lux_raw) != ESP_OK)
    {
        printf("Threshold setting failed\n");
    }
}

void bmp280_read(void *pvParamters)
{
    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(2000  / portTICK_PERIOD_MS);
        if (bmp280_force_measurement(&dev_b) != ESP_OK)
        {
            printf("Force measurement failed\n");
            continue;
        }
        if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Pres: %.2f Pa, Temp: %.2f C, Hum: %.2f%%\n", pressure, temperature, humidity);
        // printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%\n", pressure, temperature, humidity);
    }
}

void max44009_read(void *pvParameters)
{
    float lux;
    uint8_t lux_raw;
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        if (max44009_read_float(&dev_m, &lux, &lux_raw) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }
        printf("Lux: %.3f\n", lux);
    }
}

void max44009_th_test(void *pvParameters)
{
    float lux;
    uint8_t lux_raw;
    while(1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (max44009_read_float(&dev_m, &lux, &lux_raw) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
        }
        printf("init Lux: %.3f\n", lux);
        if (max44009_set_threshold_etc(&dev_m, &params_m, lux, lux_raw) != ESP_OK)
        {
            printf("Threshold setting failed\n");
        }

        max44009_read_regs(&dev_m);
    }
    
}


/**
 * max44009 enable intr 
 */
void max44009_task(void* arg) {
	uint8_t intr_status;
    float lux_f;
    uint8_t lux_raw;
	// infinite loop
	for(;;) {
		// wait for the notification from the ISR
		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
            // read the ints bit to confirm
            // printf("xSemaphore: %d\n", (int)xSemaphore);
            // printf("read intr_status\n");
			i2c_dev_read_reg(&dev_m.i2c_dev, 0x00, &intr_status, 1);
            printf("intr_status: %x\n", intr_status);
            if (intr_status == 1) {
                /**max44009 caused the interrupt
                 * 1. read max44009 amibient lux
                 * 2. write upper/lower lux threshold and 
                 *      threshold  timer registers
                 * 3. wait until next hardware interrupt
                 */
                if (max44009_read_float(&dev_m, &lux_f, &lux_raw) != ESP_OK)
                {
                    printf("Temperature/pressure reading failed\n");
                    continue;
                }
                printf("Lux: %.3f\n", lux_f);
                if (max44009_set_threshold_etc(&dev_m, &params_m, lux_f, lux_raw) != ESP_OK)
                {
                    printf("Threshold setting failed\n");
                    continue;
                }
            }
		}
	}
}

void gp2y1014au0f_read()
{
    int voltage;
    float dustDensity;

    while(1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_LED_CONTROL, 0);
        // printf("GPIO_25 set low\n");
        ets_delay_us(GP2Y_SAMPLE_TIME); //delay microsecond 
        
        // printf("analog_read start\n");
        voltage = analog_read(channel1);
        // printf("analog_read end\n");

        ets_delay_us(GP2Y_DELTA_TIME);
        // printf("delay delta finished\n");
        
        gpio_set_level(GPIO_LED_CONTROL, 1);
        // printf("GPIO_25 set high\n");
        ets_delay_us(GP2Y_SLEEP_TIME);

        // printf("delay sleep finished\n");

        dustDensity = 0.17 * voltage / 1000 - 0.1;

        if (dustDensity < 0) dustDensity = 0;

        printf("Dust Density: %.2f mg/m3\n", dustDensity);
    }
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ml8511_read()
{
    int uvLevel;//, refLevel;

    float outputVoltage, uvIntensity;
    while(1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_UV_EN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        uvLevel = analog_read(channel2);
        printf("uvLevel = %d\n", uvLevel);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_UV_EN, 0);

        // outputVoltage = 3.3 / refLevel * uvLevel;
        if (uvLevel <= 990) uvLevel = 990;
        outputVoltage =  (float)uvLevel / 1000;
        printf("outputVoltage = %.2f\n", outputVoltage);
        uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

        printf("UV Intensity: %.2f mw/cm^2\n", uvIntensity);
    }
}

void app_main()
{
    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();
    init_gpio();
    init_adc();

    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();

    // xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(max44009_task, "max44009_task", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(gp2y1014au0f_read, "gp2y1014au0f_read", configMINIMAL_STACK_SIZE * 8, NULL, 4, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ml8511_read, "ml8511_read", configMINIMAL_STACK_SIZE * 8, NULL, 4, NULL, APP_CPU_NUM);
    
    // install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(GPIO_INTR_IO, max44009_isr_handler, NULL);
}

