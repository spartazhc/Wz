#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "soc/rtc_cntl_reg.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "bmp280.h"
#include "max44009.h"

#include "driver/gpio.h"
//adc
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "rom/ets_sys.h"

#include "config.h"
#include "debug.h"

#include "mqtt.h"
#include "os.h"
#include "onenet.h"
// ssd1306
#include "ssd1306.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"
#include "ssd1306_default_if.h"

// GPIO
#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_UV_EN              0
// adc 
#define DEFAULT_VREF        1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES       1          //Multisampling

static bool onenet_initialised = false;
static TaskHandle_t xOneNetTask = NULL;

esp_err_t res;

//bmp280 & max44009
bmp280_params_t params_b;
max44009_params_t params_m;
bmp280_t dev_b;
max44009_t dev_m;

const char* data_stream[6] = {"illuminance", "temperature", "humidity", "pressure", 
                         "ultraviolet", "dustDensity"};
static float data[6];


// adc
static esp_adc_cal_characteristics_t *adc_chars;
// static const adc_channel_t channel1 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel2 = ADC_CHANNEL_7;     //GPIO35
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

void onenet_task(void *param);
void data_cb(void *self, void *params);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
void wifi_conn_init(void);

void init_gpio();
void init_bme280();
void init_max44009();
void bmp280_read();
void max44009_task();
void ml8511_read();
void display_task();
static void check_efuse();
static void print_char_val_type(esp_adc_cal_value_t val_type);
uint32_t analog_read(adc_channel_t channel);
void init_adc();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);


void onenet_start(mqtt_client *client)
{
    if(!onenet_initialised) {
        xTaskCreate(&onenet_task, "onenet_task", 2048, client, CONFIG_MQTT_PRIORITY + 1, &xOneNetTask);
        onenet_initialised = true;
    }
}

void onenet_stop(mqtt_client *client)
{
    if(onenet_initialised) {
        if(xOneNetTask) {
            vTaskDelete(xOneNetTask);
        }
        onenet_initialised = false;
    }
}

void connected_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    //mqtt_subscribe(client, "/test", 0);
    //mqtt_publish(client, "/test", "howdy!", 6, 0, 0);
    onenet_start(client);
}
void disconnected_cb(void *self, void *params)
{
     mqtt_client *client = (mqtt_client *)self;
     onenet_stop(client);
}
void reconnect_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    onenet_start(client);
}
void subscribe_cb(void *self, void *params)
{
    INFO("[APP] Subscribe ok, test publish msg\n");
    mqtt_client *client = (mqtt_client *)self;
    //mqtt_publish(client, "/test", "abcde", 5, 0, 0);
}

void publish_cb(void *self, void *params)
{

}

mqtt_settings settings = {
    .host = ONENET_HOST,
    .port = ONENET_PORT,
    .client_id = ONENET_DEVICE_ID,
    .username = ONENET_PROJECT_ID,
    .password = ONENET_AUTH_INFO,
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "/lwt",
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
    .reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};

void app_main()
{
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // wifi_conn_init();
    init_gpio();
    init_adc();
    
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();



    // xTaskCreate(&display_task, "display_task", 8192, NULL, 5, NULL);
    xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 7, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(max44009_task, "max44009_task", configMINIMAL_STACK_SIZE * 8, NULL, 8, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ml8511_read, "ml8511_read", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, APP_CPU_NUM);
}

// void display_task() {
//     int i = 0;
//     char num[10];
//     while (1) {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         // clear LCD, and select font 1
//         ssd1306_clear(0);
//         ssd1306_select_font(0, 1);

        
//         i++;
//         sprintf(num, "%d", i);
//         printf("i = %d, num = %s\n", i, num);
//         // print weather data on LCD
//         ssd1306_draw_string(0, 0, 0, "Shanghai", 1, 0);
//         ssd1306_draw_string(0, 0, 16, "temperature", 1, 0);
//         ssd1306_draw_string(0, 0, 26, num, 1, 0);
//         ssd1306_draw_string(0, 0, 40, "Description:", 1, 0);
//         ssd1306_draw_string(0, 0, 50, "cool", 1, 0);
//         ssd1306_refresh(0, true);

//     }   

// }

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
        // adc1_config_channel_atten(channel1, atten);
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
    
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_UV_EN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
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
    bmp280_force_measurement(&dev_b);
    bmp280_read_float(&dev_b, &data[1], &data[3], &data[2]);
    printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[1], data[2], data[3]);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    bmp280_force_measurement(&dev_b);
    bmp280_read_float(&dev_b, &data[1], &data[3], &data[2]);
    printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[1], data[2], data[3]);
    // bool bme280p = dev.id == BME280_CHIP_ID;
    // printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void init_max44009()
{
    max44009_init_auto_params(&params_m);   
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
}

void bmp280_read()
{
    // float pressure=0, temperature=0, humidity=0;
    // int ret = 0;
    while (1)
    {
        vTaskDelay(15 * 1000  / portTICK_PERIOD_MS);
        if (bmp280_force_measurement(&dev_b) != ESP_OK)
        {
            printf("Force measurement failed\n");
            continue;
        }
        // if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
        if (bmp280_read_float(&dev_b, &data[1], &data[3], &data[2]) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[1], data[2], data[3]);
        // printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%\n", pressure, temperature, humidity);
    }
}

// task  will react to button clicks
void max44009_task() 
{
    float lux_f = 0;
    uint8_t lux_raw;
	// infinite loop
    while (1) {
        vTaskDelay(15 * 1000 / portTICK_PERIOD_MS);
        if (max44009_read_float(&dev_m, &lux_f, &lux_raw) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }
        printf("Lux: %.3f\n", lux_f);
        data[0] = lux_f;
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
        vTaskDelay(59*1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_UV_EN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        uvLevel = analog_read(channel2);
        // printf("uvLevel = %d\n", uvLevel);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_UV_EN, 0);

        // outputVoltage = 3.3 / refLevel * uvLevel;
        if (uvLevel <= 990) uvLevel = 990;
        outputVoltage =  (float)uvLevel / 1000;
        // printf("outputVoltage = %.2f\n", outputVoltage);
        uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
        
        data[4] = uvIntensity;
        printf("UV Intensity: %.2f mw/cm^2\n", uvIntensity);
    }
}

void onenet_task(void *param)
{
    mqtt_client* client = (mqtt_client *)param;
    char buf[128];
    memset(buf, 0, sizeof(buf));

   while(1) {
       vTaskDelay((unsigned long long)ONENET_PUB_INTERVAL* 1000 / portTICK_RATE_MS);
        for (int i = 0; i < 6; ++i) {
            sprintf(&buf[3], "{\"%s\":%.2f}", data_stream[i], data[i]);
            uint16_t len = strlen(&buf[3]);
            buf[0] = data_type_simple_json_without_time;
            buf[1] = len >> 8;
            buf[2] = len & 0xFF;
            mqtt_publish(client, "$dp", buf, len + 3, 0, 0);

            for (int k = 0 ; k < len + 3; k++){
                printf("0x%02x ", buf[k]);
            }
            printf(", len:%d\n", len+3);
        }
   }    
}

void data_cb(void *self, void *params)
{
    (void)self;
    mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

    if (event_data->data_offset == 0) {

        char *topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
        INFO("[APP] Publish topic: %s\n", topic);
        free(topic);
    }

    // char *data = malloc(event_data->data_length + 1);
    // memcpy(data, event_data->data, event_data->data_length);
    // data[event_data->data_length] = 0;
    INFO("[APP] Publish data[%d/%d bytes]\n",
         event_data->data_length + event_data->data_offset,
         event_data->data_total_length);
         // data);

    // free(data);

}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

    case SYSTEM_EVENT_STA_GOT_IP:

        mqtt_start(&settings);
        // Notice that, all callback will called in mqtt_task
        // All function publish, subscribe
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        
        mqtt_stop();
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    default:
        break;
    }
    return ESP_OK;


}

void wifi_conn_init(void)
{
    INFO("[APP] Start, connect to Wifi network: %s ..\n", WIFI_SSID);

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&icfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
}