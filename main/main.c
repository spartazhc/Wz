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
// weather app
#include <cJSON.h>
#include "app_config.h"
#include "app_http.h"
#include "app_rest.h"
// ssd 1306
static const int I2CDisplayAddress = 0x3C;
static const int I2CDisplayWidth = 128;
static const int I2CDisplayHeight = 32;
static const int I2CResetPin = -1;
struct SSD1306_Device I2CDisplay;
// GPIO
#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_UV_EN          15
#define GPIO_INTR           0
#define ESP_INTR_FLAG_DEFAULT 0
// adc 
#define DEFAULT_VREF        1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES       1          //Multisampling

#define UV_const        1019
#define DELAY_SECOND    5
static bool onenet_initialised = false;
static TaskHandle_t xOneNetTask = NULL;

esp_err_t res;

//bmp280 & max44009
bmp280_params_t params_b;
max44009_params_t params_m;
bmp280_t dev_b;
max44009_t dev_m;
uint8_t display_status = 0;
uint8_t button_tmp = 0;

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
void max44009_read();
void ml8511_read();
void display_task();
static void check_efuse();
static void print_char_val_type(esp_adc_cal_value_t val_type);
uint32_t analog_read(adc_channel_t channel);
void init_adc();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

SemaphoreHandle_t xSemaphore = NULL;
TaskHandle_t xHandle;
// interrupt service routine, called when the button is pressed
void IRAM_ATTR button_isr_handler(void* arg) {
    // notify the button task
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}
// task that will react to button clicks
void button_task(void* arg) {
	
	// infinite loop
	for(;;) {
		// wait for the notification from the ISR
		if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
			printf("Button pressed!\n");
            button_tmp++;
            if (button_tmp >= 2) {
                button_tmp = 0;
                display_status++;
                if (display_status >= 3) display_status = 0;
                vTaskDelete( xHandle );
                xTaskCreate(&display_task, "display_task", 8192, NULL, 5, &xHandle);
            }
		}
	}
}

bool DefaultBusInit( void ) {
    assert( SSD1306_I2CMasterInitDefault( ) == true );
    assert( SSD1306_I2CMasterAttachDisplayDefault( &I2CDisplay, I2CDisplayWidth, I2CDisplayHeight, I2CDisplayAddress, I2CResetPin ) == true );
    return true;
}

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
    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_conn_init();
    init_gpio();
    init_adc();
    
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();

    if ( DefaultBusInit( ) == true ) {
        printf( "I2C Display Bus Init lookin good...\n" );
        xTaskCreate(&display_task, "display_task", 8192, NULL, 5, &xHandle);
    } else printf( "Failed to init display ...\n" );

     // start the task that will handle the button
	xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
	// install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	// attach the interrupt service routine
	gpio_isr_handler_add(GPIO_INTR, button_isr_handler, NULL);
}

void display_task() {
    
    while (1) {
        printf("display_status = %d\n", display_status);
        if (display_status < 2) {
            bmp280_read();
            max44009_read();
            ml8511_read();

            SSD1306_Clear( &I2CDisplay, SSD_COLOR_BLACK );
            SSD1306_SetFont( &I2CDisplay, &Font_droid_sans_mono_7x13);
            
            char str[20];
            sprintf(str, "T:%.2fC", data[1]);
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_NorthWest, str, SSD_COLOR_WHITE );
            sprintf(str, "UV:%.2f", data[4]);
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_NorthEast, str, SSD_COLOR_WHITE );
            sprintf(str, "H:%.2f%%", data[2]);
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_SouthWest, str, SSD_COLOR_WHITE );
            sprintf(str, "I:%.2f", data[0]);
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_SouthEast, str, SSD_COLOR_WHITE );
            SSD1306_Update( &I2CDisplay );
            if (display_status == 0) {
                vTaskDelay(DELAY_SECOND * 1000 / portTICK_PERIOD_MS);
            } else {
                vTaskDelay(1 * 1000 / portTICK_PERIOD_MS);
            }
        } else {
            /** openweather API response 
             * {"coord":{"lon":139,"lat":35},
                "sys":{"country":"JP","sunrise":1369769524,"sunset":1369821049},
                "weather":[{"id":804,"main":"clouds","description":"overcast clouds","icon":"04n"}],
                "main":{"temp":289.5,"humidity":89,"pressure":1013,"temp_min":287.04,"temp_max":292.04},
                "wind":{"speed":7.31,"deg":187.002},
                "rain":{"3h":0},
                "clouds":{"all":92},
                "dt":1369824698,
                "id":1851632,
                "name":"Shuzenji",
                "cod":200}
             */
            app_rest_fetch();
            // Using cJSON, parse retrieved JSON string into human readable weather data
            cJSON *root = cJSON_Parse(http_json_message);

            cJSON *_main = cJSON_GetObjectItem(root,"main");
            cJSON *_weather = cJSON_GetObjectItem(root,"weather");
            cJSON *_zero = cJSON_GetArrayItem(_weather, 0);

            char *city = cJSON_GetObjectItem(root,"name")->valuestring;
            double temp = cJSON_GetObjectItem(_main, "temp")->valuedouble;
            int humidity = cJSON_GetObjectItem(_main, "humidity")->valueint;
            char *description = cJSON_GetObjectItem(_zero, "description")->valuestring;

            char city_buf[32];
            char temp_buf[32];
            char hum_buf[32];

            // display parsed data on terminal for debugging purposes. OPTIONAL
            printf("\nCity: %s", city);
            printf("\nTemperature: %.0lf C", temp);
            printf("\nHumidity: %d %%", humidity);
            printf("\nDescription: %s\n", description);

            // format parsed data into 'pretty' strings to display on LCD
            if (strlen(description) >= 10) {
                sprintf(city_buf, "SH");
            } else {
                sprintf(city_buf, "%s", city);
            }
            sprintf(temp_buf, "T:%.0lf C", temp);
            sprintf(hum_buf, "H:%d %%", humidity);

            SSD1306_Clear( &I2CDisplay, SSD_COLOR_BLACK );
            SSD1306_SetFont( &I2CDisplay, &Font_droid_sans_mono_7x13);
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_NorthWest, city_buf, SSD_COLOR_WHITE );
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_NorthEast, description, SSD_COLOR_WHITE );
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_SouthWest, temp_buf, SSD_COLOR_WHITE );
            SSD1306_FontDrawAnchoredString( &I2CDisplay, TextAnchor_SouthEast, hum_buf, SSD_COLOR_WHITE );
            SSD1306_Update( &I2CDisplay );
            vTaskDelay(15 * 60 * 1000 / portTICK_PERIOD_MS);
        }
    }   

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
 * GPIO_INTR       | button
 * GPIO_UV_EN      | ml8511 enable
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

    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_INTR);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
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
    if (bmp280_force_measurement(&dev_b) != ESP_OK)
    {
        printf("Force measurement failed\n");
    }
    // if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
    if (bmp280_read_float(&dev_b, &data[1], &data[3], &data[2]) != ESP_OK)
    {
        printf("Temperature/pressure reading failed\n");
    }

    printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[1], data[2], data[3]);
    // printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%\n", pressure, temperature, humidity);
}

// task  will react to button clicks
void max44009_read() 
{
    float lux_f = 0;
    uint8_t lux_raw;
    
    if (max44009_read_float(&dev_m, &lux_f, &lux_raw) != ESP_OK)
    {
        printf("Illuminance reading failed\n");
    }
    printf("Lux: %.3f\n", lux_f);
    data[0] = lux_f;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ml8511_read()
{
    int uvLevel;//, refLevel;
    float outputVoltage, uvIntensity;

    gpio_set_level(GPIO_UV_EN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uvLevel = analog_read(channel2);
    // printf("uvLevel = %d\n", uvLevel);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_UV_EN, 0);

    // outputVoltage = 3.3 / refLevel * uvLevel;
    if (uvLevel <= UV_const) uvLevel = UV_const;
    outputVoltage =  (float)uvLevel / 1000;
    // printf("outputVoltage = %.2f\n", outputVoltage);
    uvIntensity = mapfloat(outputVoltage, UV_const / 1000.0, 2.9, 0.0, 15.0);
    
    data[4] = uvIntensity;
    printf("UV Intensity: %.2f mw/cm^2\n", uvIntensity);
}

void onenet_task(void *param)
{
    mqtt_client* client = (mqtt_client *)param;
    char buf[128];
    memset(buf, 0, sizeof(buf));

   while(1) {
       vTaskDelay((unsigned long long)ONENET_PUB_INTERVAL* 1000 / portTICK_RATE_MS);
       if (display_status == 2) {   // 需要再读一下传感器数值
            bmp280_read();
            max44009_read();
            ml8511_read();
       }
        for (int i = 0; i < 5; ++i) {
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