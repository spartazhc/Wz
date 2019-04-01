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

#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_INTR_IO    GPIO_NUM_5

#define ESP_INTR_FLAG_DEFAULT   0
// UART
#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

extern me3616_obj_t obj[ME3616_OBJ_NUM];
extern me3616_event_t me3616;
extern bool flag_ok ;
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
void float2char(float slope, char* buffer);

void init_gpio();
void init_uart();
void init_bme280();
void init_max44009();
void init_me3616();
void bmp280_read();
void max44009_task();
void me3616_getevent(const char * data);
void me3616_response(const char * data);
void uart_forward();
void me3616_upload();
// void max44009_read(void *pvParameters);
// void max44009_th_test(void *pvParameters);

void me3616_test(){
    int ret = 0;
    me3616_power_on();
    printf("Doiot System Start.\r\n");
    // wait until internet connect success
    while (!me3616.flag_ip){
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 提示符
    ret = me3616_send_cmd("AT\r\n", flag_ok, 300);
    printf("**ret = %d\n", ret);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询模块识别信息
    ret = me3616_send_cmd("ATI\r\n", flag_ok, 300);
    printf("**ret = %d\n", ret);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询IMEI号
    ret = me3616_send_cmd("AT+CGSN=1\r\n", flag_ok, 300);
    printf("**ret = %d\n", ret);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询IMSI号
    me3616_send_cmd("AT+CIMI\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询信号强度
    me3616_send_cmd("AT+CSQ\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询网络附着状态
    me3616_send_cmd("AT+CEREG?\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 创建onenet平台
    me3616_send_cmd("AT+MIPLCREATE\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 新增object id:3303(temperature)
    me3616_send_cmd("AT+MIPLADDOBJ=0,3301,1,\"1\",3,0\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    me3616_send_cmd("AT+MIPLADDOBJ=0,3303,1,\"1\",3,0\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    me3616_send_cmd("AT+MIPLADDOBJ=0,3304,1,\"1\",3,0\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    me3616_send_cmd("AT+MIPLADDOBJ=0,3323,1,\"1\",3,0\r\n", flag_ok, 300);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 注册onenet 平台  这一步比较费时间，要多delay
    me3616_send_cmd("AT+MIPLOPEN=0,3600\r\n", flag_ok, 300);
    while (!me3616.flag_miplopen){vTaskDelay(100 / portTICK_PERIOD_MS);}
    // vTaskDelay(100 / portTICK_PERIOD_MS);
}


void app_main()
{
    // //me3616_event_t* me3616 = get_me3616();

    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();
    init_gpio();
    init_uart();
    xTaskCreatePinnedToCore(uart_forward, "uart_forward", 1024 *8, NULL, 10, NULL,1);
    // me3616_test();
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();
    // initiate me3616 after start uart_forward task ()
    init_me3616();
    xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(max44009_task, "max44009_task", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(me3616_upload, "me3616_upload", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, 1);
    // install ISR service with default configuration
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	
	// attach the interrupt service routine
	gpio_isr_handler_add(GPIO_INTR_IO, max44009_isr_handler, NULL);
}


void init_gpio()
{
    //max44009 intr pin
    gpio_config_t io_conf;
    //enable interrupt negedge
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
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << GPIO_PWR_ME3616)|(1ULL << GPIO_RESET_ME3616));
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
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

void init_me3616()
{   
    //me3616_event_t* me3616 = get_me3616();
    // power me3616
    gpio_set_level(GPIO_PWR_ME3616, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_PWR_ME3616, 0);
    printf("Doiot System Start.\r\n");
    // wait until internet connect success
    while (!me3616.flag_ip){
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        // printf("flag_ip = %d\n", me3616.flag_ip);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 提示符
    uart_sendstring(UART_NUM_1, "AT\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询模块识别信息
    uart_sendstring(UART_NUM_1, "ATI\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询IMEI号
    uart_sendstring(UART_NUM_1, "AT+CGSN=1\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询IMSI号
    uart_sendstring(UART_NUM_1, "AT+CIMI\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询信号强度
    uart_sendstring(UART_NUM_1, "AT+CSQ\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 查询网络附着状态
    uart_sendstring(UART_NUM_1, "AT+CEREG?\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 创建onenet平台
    uart_sendstring(UART_NUM_1, "AT+MIPLCREATE\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // 新增object id:3303(temperature)
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3301,1,\"1\",3,0\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3303,1,\"1\",3,0\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3304,1,\"1\",3,0\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_sendstring(UART_NUM_1, "AT+MIPLADDOBJ=0,3323,1,\"1\",3,0\r\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
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
    // int ret = 0;
    while (1)
    {
        vTaskDelay(20*1000  / portTICK_PERIOD_MS);
        if (bmp280_force_measurement(&dev_b) != ESP_OK)
        {
            printf("Force measurement failed\n");
            continue;
        }
        // if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
        if (bmp280_read_float(&dev_b, &data[0], &data[2], &data[1]) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Temp: %.2f C, Hum: %.2f%%, Pres: %.2f Pa\n", data[0], data[1], data[2]);
        // printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%\n", pressure, temperature, humidity);
        if (me3616.discover_count >= 4) {
            /**
             * obj[1]: temperature;
             * obj[2]: humidity;
             * obj[3]: pressure;
             */
            for (int i = 0; i < 3; ++i) {
                obj[i+1].value = data[i];
                update_value(obj[i+1], data[i]);
            }
        }
    }
}

// task  will react to button clicks
void max44009_task() 
{
	uint8_t intr_status;
    float lux_f = 0;
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
                if (me3616.upload_en) {
                    obj[0].value = lux_f;
                }
                if (max44009_set_threshold_etc(&dev_m, &params_m, lux_f, lux_raw) != ESP_OK)
                {
                    printf("Threshold setting failed\n");
                    continue;
                }
            }
		}
	}
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
    char objid[10];
    char msgid[10];
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
                me3616_onenet_mipldiscover_rsp(cmd, msgid, "\"5700;5601;5602\"");
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
        mystrcpy(data, msgid, 1);
        mystrcpy(data, objid, 2);// get object id
        mystrcpy(data, resourceid, 4);
        resourceid[4] = '\0';
        // if object is in bme280
        if (!strcmp(objid, "3303")) id = 1;
        if (!strcmp(objid, "3304")) id = 2;
        if (!strcmp(objid, "3323")) id = 3;
        if (id == 1 || id == 2 || id == 3) {
            float data[3];
            bmp280_force_measurement(&dev_b);
            bmp280_read_float(&dev_b, &data[0], &data[2], &data[1]);
            printf("READRSP: temp: %.2f, humi: %.2f, pres:%.2f\n", data[0], data[1], data[2]);
            for (int i = 1; i < 4; ++i){
                obj[i].value = data[i-1];
                ret = update_value(obj[i], data[i-1]);
                if (ret == 1) { // max
                    me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                    obj[i].id, 5602, data[i-1], 0);
                    uart_sendstring(UART_NUM_1, cmd);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                } else if (ret == 2) { // min
                    me3616_onenet_miplnotify_float(cmd, obj[i].msgid_observe,
                    obj[i].id, 5601, data[i-1], 0);
                    uart_sendstring(UART_NUM_1, cmd);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                ret = 0; // reset ret;
            }
        } 
        me3616_onenet_miplread_rsp_float(cmd, msgid, objid, resourceid, obj[id].value, 0);
        uart_sendstring(UART_NUM_1, cmd);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } else if (is_message_in_str(data, "+MIPLEVENT: 0, 6")) {
        // me3616.event = ME3616_REG_SUCCESS;
        me3616.flag_miplopen = 1;
        return;
    } else if (is_message_in_str(data, "+IP:")) {
        // me3616.event = ME3616_IP_CONNECTED;
        me3616.flag_ip = 1;
        return;
        // printf("event: get_ip\n");
    } else {
        // me3616.event = ME3616_NORMAL;    // set to ME3616_NORMAL
        return;
    }
    return;   
}
//+MIPLOBSERVE: 0, 80856, 1, 3303, 0, -1
// +MIPLDISCOVER: 0, 15321, 3303
// void me3616_response(const char* data)
// {
//     // const char* miplobserve = "MIPLOBSERVE";
//     char cmd[80];
//     //me3616_event_t* me3616 = get_me3616();

//     switch (me3616.event)
//     {
//         case ME3616_NORMAL:
//             break;
//         // when RSP, only msgid is needed
//         case ME3616_OBSERVE:
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             // strcpy(cmd, "AT+MIPLOBSERVERSP=0,");
//             // strcat(cmd, obj[me3616.cur_obj].msgid_observe);
//             // strcat(cmd, ",1\r\n");
//             me3616_onenet_miplobserve_rsp(cmd, obj[me3616.cur_obj].msgid_observe);
//             uart_sendstring(UART_NUM_1, cmd);
//             me3616.event = ME3616_NORMAL;
//             // me3616.cur_obj = -1;
//             me3616.observe_count++;
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             break;
//             // AT+MIPLOBSERVERSP=0, ,1
//             // AT+MIPLDISCOVERRSP=0, ,1,14,"5700;5601;5602"
//         case ME3616_DISCOVER:
//             // printf("in ME3616_DISCOVER\n");
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             me3616_onenet_mipldiscover_rsp(cmd, obj[me3616.cur_obj].msgid_discover, "\"5700;5601;5602\"");
//             // strcpy(cmd, "AT+MIPLDISCOVERRSP=0,");
//             // strcat(cmd, obj[me3616.cur_obj].msgid_discover);
//             // strcat(cmd, ",1,14,\"5700;5601;5602\"\r\n"); // specific attribute for object
//             // printf("make cmd: %s\n",cmd);
//             uart_sendstring(UART_NUM_1, cmd);
//             printf("cmd is sent\n");
//             me3616.event = ME3616_NORMAL;
//             // printf("me3616.event = %d",me3616.event);
//             // me3616.cur_obj = -1;
//             me3616.discover_count++;
//             // printf("discover_count = %d",me3616.discover_count);
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             // printf("hello~\n");
//             break;
//         case ME3616_IP_CONNECTED:
//             me3616.flag_ip = 1;
//             break;
//         case ME3616_REG_SUCCESS:
//             me3616.flag_miplopen = 1;
//             break;
//         default:
//             break;
//     }
// }

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
        vTaskDelay(1 * 60 * 1000 / portTICK_PERIOD_MS);
        max_count++;
        if (!me3616.upload_en && me3616.discover_count == 4) {
            me3616.upload_en = 1;
            printf("init success & upload enable\n");

        }
        if (me3616.flag_miplopen && me3616.upload_en) {
            for(size_t i = 0; i < ME3616_OBJ_NUM; i++)
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
                for (int i = 1; i < 4; ++i) { //i=1,2,3
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

    }
}