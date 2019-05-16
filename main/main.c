#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include <esp_system.h>
#include <bmp280.h>
#include <max44009.h>
#include "driver/uart.h"
#include <string.h>
#include <me3616.h>
//adc
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "rom/ets_sys.h"
// GPIO
#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_INTR_IO            2  // INTR pin for max44009
#define ESP_INTR_FLAG_DEFAULT   0   
#define GPIO_LED_CONTROL        25  // LED control pin for gp2y1014au
#define GPIO_UV_EN              15
// UART
#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)
// adc 
#define DEFAULT_VREF        1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES       1          //Multisampling
// gp2y time define in microsecoend !
#define GP2Y_SAMPLE_TIME    280
#define GP2Y_DELTA_TIME     40
#define GP2Y_SLEEP_TIME     9680

#define UV_const        1025
#define UPDATE_TIME     5
// #define ME3616_GPS_MODE

extern me3616_obj_t obj[ME3616_OBJ_NUM];
extern me3616_event_t me3616;
extern bool flag_ok ;
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

void uart_sendstring(uint8_t UART_NUM, char* str)
{
    uart_write_bytes(UART_NUM, (const char *) str, strlen(str));
}
char is_message_in_str(const char* data, const char* str)
{
    return (strstr(data, str) - data) > 0 ? 1 : 0;
}
int count_comma(const char* str, int n);
void mystrcpy(const char *src, char* dst, int n);
void mystrcpy2(const char *src, char* dst, int n);
void float2char(float slope, char* buffer);

void init_gpio();
void init_uart();
void init_bme280();
void init_max44009();
void me3616_registered_to_onenet();
void bmp280_read();
void max44009_read();
void gp2y1014au0f_read();
void ml8511_read();
void me3616_getevent(const char * data);
void me3616_response(const char * data);
void uart_forward();
void me3616_upload();
static void check_efuse();
static void print_char_val_type(esp_adc_cal_value_t val_type);
uint32_t analog_read(adc_channel_t channel);
void init_adc();

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);


void app_main()
{
    init_gpio();
    init_adc();
    init_uart();
    xTaskCreatePinnedToCore(uart_forward, "uart_forward", 1024 *8, NULL, 10, NULL,1);
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();
    // initiate me3616 after start uart_forward task ()
    me3616_power_on();
    me3616_registered_to_onenet();
    xTaskCreatePinnedToCore(me3616_upload, "me3616_upload", configMINIMAL_STACK_SIZE * 8, NULL, 9, NULL, 1);
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
    
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << GPIO_PWR_ME3616)|(1ULL << GPIO_RESET_ME3616)|(1ULL << GPIO_UV_EN));
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << GPIO_LED_CONTROL));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

void init_uart()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    // uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
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
}

void init_max44009()
{
    max44009_init_auto_params(&params_m);   
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
        printf("Max44009 reading failed\n");
    }
    printf("init Lux: %.3f\n", lux);
}

void me3616_registered_to_onenet()
{  
    printf("Register to onenet...\r\n");
    // wait until internet connect success
    while (!me3616.flag_ip){
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        // printf("flag_ip = %d\n", me3616.flag_ip);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
#ifdef ME3616_GPS_MODE
    uart_sendstring(UART_NUM_1, "AT+ZGMODE=2\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+ZGNMEA=2\r\n");  // Only RMC is needed
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+ZGRUN=1\r\n");  // run 1 time (success)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    int gps_count = 0;
    while (!me3616.flag_gps){
        if (gps_count == 18) {
            uart_sendstring(UART_NUM_1, "AT+ZGRUN=0\r\n"); 
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        }
        vTaskDelay(10 * 1000 / portTICK_PERIOD_MS); 
        gps_count++;
    }
#endif
    // 提示符
    uart_sendstring(UART_NUM_1, "AT\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // 查询模块识别信息
    uart_sendstring(UART_NUM_1, "ATI\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // 查询IMEI号
    uart_sendstring(UART_NUM_1, "AT+CGSN=1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // 查询IMSI号
    uart_sendstring(UART_NUM_1, "AT+CIMI\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // 查询信号强度
    uart_sendstring(UART_NUM_1, "AT+CSQ\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // 查询网络附着状态
    uart_sendstring(UART_NUM_1, "AT+CEREG?\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // 创建onenet平台
    uart_sendstring(UART_NUM_1, "AT+MIPLCREATE\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 新增object id:3303(temperature)
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3301,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3303,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3304,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3323,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3300,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3325,1,\"1\",3,1\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
// #ifdef ME3616_GPS_MODE
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3336,1,\"1\",2,0\r\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
// #endif
    // 注册onenet 平台  这一步比较费时间，要多delay
    uart_sendstring(UART_NUM_1, "AT+MIPLOPEN=0,3600\r\n");
    while (!me3616.flag_miplopen){vTaskDelay(100 / portTICK_PERIOD_MS);}
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
}
void bmp280_read()
{
    // float pressure=0, temperature=0, humidity=0;
    float data[3] = {0};
    if (bmp280_force_measurement(&dev_b) != ESP_OK)
    {
        printf("Force measurement failed\n");
    }
    // if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
    if (bmp280_read_float(&dev_b, &data[0], &data[2], &data[1]) != ESP_OK)
    {
        printf("Temperature/pressure reading failed\n");
    }

    printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[0], data[1], data[2]);
    if (me3616.discover_count >= ME3616_OBJ_NUM) {
        /**
         * obj[1]: temperature;
         * obj[2]: humidity;
         * obj[3]: pressure;
         */
        for (int i = 0; i < 3; ++i) {
            // init max & min otherwise min will always be 0;
            if (obj[i+1].max == 0 && obj[i+1].min == 0) {
                obj[i+1].max = data[i];
                obj[i+1].min = data[i];
            }
            // obj[i+1].value = data[i];
            update_value(&obj[i+1], data[i]);
        }
    }
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
    update_value(&obj[0], lux_f);
}


/**
 * return index next to the nth ',' of str
 */
int count_comma(const char* str, int n)
{
    int count = 0;
    size_t i;
    for(i = 0; str[i] != '\0'; i++)
    {
        if (str[i] == ',')   ++count;
        if (count == n) {
            break;
        }
    }
    if (count == 0) return -1;
    else ++i;

    return i;
}

/**
 * copy str start from the first ',' to the next ',' 
 */
void mystrcpy(const char *src, char* dst, int n)
{
    int l = count_comma(src, n);
    size_t i = 0;
    // printf("l = %d\n", l);
	++l;
    while (src[l] != '\0'){
        if (src[l] == ',') break;
        dst[i] = src[l];
        ++i;
        ++l;
    }
    // printf("i = %u\n", i);
    dst[i] = '\0';
}

void mystrcpy2(const char *src, char* dst, int n)
{
    int l = count_comma(src, n);
    size_t i = 0;
    // printf("l = %d\n", l);
    while (src[l] != '\0'){
        if (src[l] == ',') break;
        dst[i] = src[l];
        ++i;
        ++l;
    }
    // printf("i = %u\n", i);
    dst[i] = '\0';
}

/**
 * keep 2 mantissa
 */
void float2char(float slope, char* buffer)  //浮点型数，存储的字符数组，字符数组的长度
{
    int temp;
    int8_t i = 0, j = 0, k = 0;
    // if (slope < 0) {
    //     buffer[0] = '-';
    //     slope = -slope;
    // }
    temp = (int)slope;//取整数部分
    for(i = 0; temp != 0; i++)//计算整数部分的位数
        temp /= 10;
    temp =(int)slope;
    if (i > 0) {
       i--;
    }
    // printf("i = %d", i);
    for(j = i; j >= 0; j--)//将整数部分转换成字符串型
    {
        buffer[j] = temp % 10 + '0';
        temp /= 10;
    }
    buffer[i + 1] = '.';

    slope -=(int)slope;
	i = i + 2;
    for(; k < 2; ++k)//将小数部分转换成字符串型
    {
        slope *= 10;
        buffer[i + k] = (int)slope + '0';
        slope -= (int)slope;
    }
    buffer[i + k] = '\0';
}

void me3616_getevent(const char * data)
{
    // char tmp[10];
    // char tmp2[10];
    char cmd[80];
    char objid[12];
    char msgid[12];
    char resourceid[10];
    if (is_message_in_str(data, "OK")) {
        flag_ok = 1;
    }
    
    if (is_message_in_str(data, "MIPLOBSERVE:")) {
        //+MIPLOBSERVE: 0, 80856, 1, 3303, 0, -1
        mystrcpy(data, objid, 3);// get object id from data (3: obj_id, 1: msgid)
        // printf("obj_id = |%s|", tmp);
        for(size_t i = 0; i < ME3616_OBJ_NUM; i++) {
            // printf("tmp = |%s|", tmp);
            // printf("obj_id = |%s|", obj[i].id);
            if (!strcmp(objid, obj[i].id)&& !obj[i].observe) {
                // me3616 object operation
                obj[i].observe = 1;
                mystrcpy(data, obj[i].msgid_observe, 1);// copy msgid to object
                // printf("msgid = %s",obj[i].msgid_observe );
                me3616_onenet_miplobserve_rsp(cmd, obj[i].msgid_observe);
                uart_sendstring(UART_NUM_1, cmd);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            }
        }
    } else if (is_message_in_str(data, "MIPLDISCOVER:")) {
        // +MIPLDISCOVER: 0, 15321, 3303
        // printf("in DISCOVER\n");
        mystrcpy(data, objid, 2);// get object id from data (2: obj_id, 1: msgid)
        objid[4] = '\0'; // manually set '\0'
        mystrcpy(data, msgid, 1);// copy msgid to object
        // printf("tmp = %s\n",tmp);
        for(size_t i = 0; i < ME3616_OBJ_NUM; i++) {
            // printf("in for loop\n");
            if (!strcmp(objid, obj[i].id)){//&& !obj[i].discover) {
                // me3616 object operation
                // printf("in if\n");
                obj[i].discover = 1;    
                if (i == 6) {// location is different
                    me3616_onenet_mipldiscover_rsp(cmd, msgid, "\"5514;5515\"");
                } else {
                    me3616_onenet_mipldiscover_rsp(cmd, msgid, "\"5700;5601;5602;5605\"");
                }
                uart_sendstring(UART_NUM_1, cmd);
                me3616.discover_count++;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
            }
        }
    } else if (is_message_in_str(data, "+MIPLREAD:")) {
        // 现在我返回的类型似乎都是float 
        // +MIPLREAD: 0, 65313, 3303, 0, 5700
        // AT+MIPLREADRSP=0,65313,1,3303,0,5700,4,4,20.123,0,0
        int id = -1;
        int ret = 0;
        int adc_read = 0;
        float value[6];
        mystrcpy(data, msgid, 1);
        mystrcpy(data, objid, 2);// get object id
        mystrcpy(data, resourceid, 4);
        resourceid[4] = '\0';

        if (!strcmp(objid, "3301")) id = 0;
        else if (!strcmp(objid, "3303")) id = 1;
        else if (!strcmp(objid, "3304")) id = 2;
        else if (!strcmp(objid, "3323")) id = 3;
        else if (!strcmp(objid, "3300")) id = 4;
        else if (!strcmp(objid, "3325")) id = 5;
        
        if (id == 1 || id == 2 || id == 3) {    //bme280
            bmp280_force_measurement(&dev_b);
            bmp280_read_float(&dev_b, &value[1], &value[3], &value[2]);
            printf("READRSP: temp: %.2f, humi: %.2f, pres:%.2f\n", value[1], value[2], value[3]);
            
        } else if (id == 0) {   //max44009
            // for reason that i use the intrupt way, it need to change mode to do this
            // so now just do nothing;
            value[0] = obj[0].value;//nothing
        } else if (id == 4) {   //ml8511
            gpio_set_level(GPIO_UV_EN, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            adc_read = analog_read(channel2);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_UV_EN, 0);

            if (adc_read <= 990) adc_read = 990;
            value[4] = mapfloat((float)adc_read / 1000, 0.99, 2.9, 0.0, 15.0);
            printf("READRSP：uv intensity: %.2f mw/cm^2\n", value[4]);
        } else if (id == 5) {   //gp2y
            gpio_set_level(GPIO_LED_CONTROL, 0);
            ets_delay_us(GP2Y_SAMPLE_TIME); //delay microsecond 
            adc_read = analog_read(channel1);
            ets_delay_us(GP2Y_DELTA_TIME);
            gpio_set_level(GPIO_LED_CONTROL, 1);
            
            value[5] = 0.17 * adc_read / 1000 - 0.1;
            if (value[5] < 0) value[5] = 0;
            // update value in object
            printf("READRSP：dust density: %.2f mg/m3\n", value[5]);
        }
        // update value and max & min
        for (int i = 0; i < ME3616_OBJ_NUM - 1; ++i){
            // obj[i].value = value[i];
            ret = update_value(&obj[i], value[i]);
        }
        // make read response first
        me3616_onenet_miplread_rsp_float(cmd, msgid, objid, resourceid, obj[id].value, 0);
        uart_sendstring(UART_NUM_1, cmd);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // then send notify of max & min
        for (int i = 0; i < ME3616_OBJ_NUM - 1; ++i){
            if (ret == 1) { // max
                me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                obj[i].id, 5602, value[i], 0);
                uart_sendstring(UART_NUM_1, cmd);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            } else if (ret == 2) { // min
                me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                obj[i].id, 5601, value[i], 0);
                uart_sendstring(UART_NUM_1, cmd);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
    } else if (is_message_in_str(data, "+MIPLEXECUTE:")) {
        // +MIPLEXECUTE:0,22308,3303,0,5605,5, "reset"
        // AT+MIPLEXECUTERSP=0,22308,2
        int id = -1;
        mystrcpy(data, msgid, 1);
        mystrcpy(data, objid, 2);// get object id
        mystrcpy(data, resourceid, 4);

        if (!strcmp(objid, "3301")) id = 0;
        else if (!strcmp(objid, "3303")) id = 1;
        else if (!strcmp(objid, "3304")) id = 2;
        else if (!strcmp(objid, "3323")) id = 3;
        else if (!strcmp(objid, "3300")) id = 4;
        else if (!strcmp(objid, "3325")) id = 5;

        if(!strcmp(resourceid, "5605")) {
            printf("clear max & min in id: %s\n", objid);
            obj[id].max = obj[id].value;
            obj[id].min = obj[id].value;
            me3616_onenet_miplexecute_rsp(cmd, msgid, 2);
            uart_sendstring(UART_NUM_1, cmd);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            // update 5601 & 5602 resource
            me3616_onenet_miplnotify_float(cmd, obj[id].msgid_observe,
                        obj[id].id, 5602, obj[id].max, 0);
            uart_sendstring(UART_NUM_1, cmd);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            me3616_onenet_miplnotify_float(cmd, obj[id].msgid_observe,
            obj[id].id, 5601, obj[id].min, 0);
            uart_sendstring(UART_NUM_1, cmd);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    } else if (is_message_in_str(data, "+MIPLEVENT: 0, 6")) {
        // me3616.event = ME3616_REG_SUCCESS;
        me3616.flag_miplopen = 1;
        return;
    } else if (is_message_in_str(data, "+IP:")) {
        // me3616.event = ME3616_IP_CONNECTED;
        me3616.flag_ip = 1;
        return;
        // printf("event: get_ip\n");
    } else if (is_message_in_str(data, "GNRMC")) {
        // here we use objid msgid as 2 temp char[]
        // $GNRMC,064914.36,V,,,,,,,120419,,,N,V*1D
        // $GNRMC,064915.36,A,3101.61831,N,12126.26758,E,0.000,,120419,,,A,V*17
        float lat=0, lon=0;
        int itmp=0;
        mystrcpy2(data, objid, 2); // get valid information
        if (!strcmp(objid, "A")) {
            mystrcpy2(data, objid, 3);
            mystrcpy2(data, msgid, 5);
            lat = atof(objid);
            itmp = lat / 100;
            // printf("itmp=%d\n", itmp);
            obj[6].max = itmp + (lat - itmp*100)/60;
            lon = atof(msgid);
            itmp = lon / 100;
            // printf("itmp=%d\n", itmp);
            obj[6].min = itmp + (lon - itmp*100)/60;
            // printf("sLatitude: %s, sLongitude: %s\n", objid, msgid);
            printf("Longitude: %.8f, Latitude: %.8f\n", obj[6].max, obj[6].min);
            me3616.flag_gps = 1;
        }
        return;
        // printf("event: get_ip\n");
    } else {
        // me3616.event = ME3616_NORMAL;    // set to ME3616_NORMAL
        return;
    }
    return;   
}

/**
 * uart0:   pc  -- esp32
 * uart1: esp32 -- me3616
 */
void uart_forward()
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data0 = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *data1 = (uint8_t *) malloc(BUF_SIZE);
    const char *u_esp32 = {"\nesp32: "};
    // const char *u_me3616 = {"me3616: "};
    printf("init\n");
    while (1) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        // Read data from the UART0
        int len0 = uart_read_bytes(UART_NUM_0, data0, BUF_SIZE, 20 / portTICK_RATE_MS);
        int len1 = uart_read_bytes(UART_NUM_1, data1, BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len0 > 0) {//receve from pc
            uart_write_bytes(UART_NUM_0, u_esp32, 8);
            uart_write_bytes(UART_NUM_0, (const char *) data0, len0);    //display back
            uart_write_bytes(UART_NUM_1, (const char *) data0, len0);    //send to me3616
        }
        if (len1 > 0) {//receive from me3616
            // printf("uart1[R]: %d\n", len1);
            // deal with response from me3616
            // printf("len1 = %d\n",len1);
            uart_write_bytes(UART_NUM_0, (const char *) data1, len1);
            // printf("me3616_getevent\n");
            // once receive data from me3616, process the data;
            me3616_getevent((const char *) data1);
            // printf("me3616_response\n");
            // me3616_response((const char *) data1);
            // printf("len1 end\n");
        }
    }
}
void gp2y1014au0f_read()
{
    int voltage;
    float dustDensity;

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
    // update value in object
    // obj[5].value = dustDensity;
    // init max & min otherwise min will always be 0;
    if (obj[5].max == 0 && obj[5].min == 0) {
        obj[5].max = dustDensity;
        obj[5].min = dustDensity;
    }
    update_value(&obj[5], dustDensity);
    printf("Dust Density: %.2f mg/m3\n", dustDensity);
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
    // update value in object
    // obj[4].value = uvIntensity;
    if (obj[4].max == 0 && obj[4].min == 0) {
        obj[4].max = uvIntensity;
        obj[4].min = uvIntensity;
    }
    update_value(&obj[4], uvIntensity);
    printf("UV Intensity: %.2f mw/cm^2\n", uvIntensity);
}

void me3616_upload()
{
    char cmd[50];
    char buf[10];
    int max_count = 0;
    // float num_test = 13.33;
    //me3616_event_t* me3616 = get_me3616();
    printf("nbiot_upload task start!\n");
    while(1){
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        max_count++;
    #ifdef ME3616_GPS_MODE
        if (me3616.flag_gps == 1) {
            me3616_onenet_miplnotify_gps(cmd, obj[6].msgid_observe,
                        obj[6].id, 5515, obj[6].max, 0);//max place longitude
            uart_sendstring(UART_NUM_1, cmd);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            me3616_onenet_miplnotify_gps(cmd, obj[6].msgid_observe,
                        obj[6].id, 5514, obj[6].min, 0);//min place latitude
            uart_sendstring(UART_NUM_1, cmd);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            me3616.flag_gps = 0;
        }
    #endif
        if (!me3616.upload_en && me3616.discover_count == ME3616_OBJ_NUM) {
            me3616.upload_en = 1;
            printf("init success & upload enable\n");
        }
        if (me3616.flag_miplopen && me3616.upload_en) {
            max44009_read();
            bmp280_read();
            ml8511_read();
            gp2y1014au0f_read();
            for(size_t i = 0; i < ME3616_OBJ_NUM - 1; i++)
            {
                // num_test += 1;
                // AT+MIPLNOTIFY=0,114453,3303,0,5700,4,4,25.1,0,0
                float2char(obj[i].value, buf);
                me3616_onenet_miplnotify(cmd, obj[i].msgid_observe, obj[i].id,
                 5700, 4, buf, 0);
                uart_sendstring(UART_NUM_1, cmd);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            if (max_count == 10) {
                max_count = 0; // reset counter
                for (int i = 0; i < ME3616_OBJ_NUM - 1; ++i) { 
                    if (obj[i].max_flag) { // max
                        obj[i].max_flag = 0;
                        me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                        obj[i].id, 5602, obj[i].max, 0);
                        uart_sendstring(UART_NUM_1, cmd);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                    if (obj[i].min_flag) { // min
                        obj[i].min_flag = 0;
                        me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                        obj[i].id, 5601, obj[i].min, 0);
                        uart_sendstring(UART_NUM_1, cmd);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                }
            }
        }
        vTaskDelay(UPDATE_TIME * 60 * 1000 / portTICK_PERIOD_MS);
    }
}