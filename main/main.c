#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include <esp_system.h>
#include <bmp280.h>
#include <max44009.h>
#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_INTR_IO            2
#define ESP_INTR_FLAG_DEFAULT   0
  
SemaphoreHandle_t xSemaphore = NULL;
bmp280_params_t params_b;
max44009_params_t params_m;
bmp280_t dev_b;
max44009_t dev_m;
esp_err_t res;

// interrupt service routine, called when the button is pressed
void IRAM_ATTR max44009_isr_handler(void* arg) {
	
    // notify the button task
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void init_gpio()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_INTR_IO);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
// task that will react to button clicks
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

void init_bme280()
{
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
        vTaskDelay(60 * 1000  / portTICK_PERIOD_MS);
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

void app_main()
{
    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();
    init_gpio();
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();
    xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(max44009_task, "max44009_task", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, APP_CPU_NUM);

    // install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(GPIO_INTR_IO, max44009_isr_handler, NULL);
}

