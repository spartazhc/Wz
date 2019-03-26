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
#include <doiot.h>

#define SDA_GPIO 18
#define SCL_GPIO 19
#define GPIO_INTR_IO    GPIO_NUM_5
#define GPIO_PWR_DOIOT  GPIO_NUM_4
#define ESP_INTR_FLAG_DEFAULT   0
// UART
#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)

extern doiot_obj_t obj[DOIOT_OBJ_NUM];
extern doiot_event_t doiot;
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
void init_doiot();
void bmp280_read();
void max44009_task();
void doiot_getevent(const char * data);
void doiot_response(const char * data);
void uart_forward();
void doiot_upload();
// void max44009_read(void *pvParameters);
// void max44009_th_test(void *pvParameters);



void app_main()
{
    // //doiot_event_t* doiot = get_doiot();

    // create the binary semaphore
	xSemaphore = xSemaphoreCreateBinary();
    init_gpio();
    init_uart();
    xTaskCreate(uart_forward, "uart_forward", 1024, NULL, 10, NULL);
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
    init_max44009();
    // initiate doiot after start uart_forward task ()
    init_doiot();
    xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(max44009_task, "max44009_task", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(doiot_upload, "doiot_upload", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL, 1);
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
    io_conf.pin_bit_mask = (1ULL << GPIO_PWR_DOIOT);
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

void init_doiot()
{   
    //doiot_event_t* doiot = get_doiot();
    // power doiot
    gpio_set_level(GPIO_PWR_DOIOT, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_PWR_DOIOT, 0);
    printf("Doiot System Start.\r\n");
    // wait until internet connect success
    while (!doiot.flag_ip){
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        // printf("flag_ip = %d\n", doiot.flag_ip);
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
    while (!doiot.flag_miplopen){vTaskDelay(100 / portTICK_PERIOD_MS);}
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
}
void bmp280_read()
{
    float pressure=0, temperature=0, humidity=0;
    while (1)
    {
        vTaskDelay(20*1000  / portTICK_PERIOD_MS);
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
        // if (doiot.discover_count >= 2) {
            obj[1].value = temperature;
            obj[2].value = humidity;
            obj[3].value = pressure;
        // }
        
        
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
                // if (doiot.discover_count >= 2) {
                obj[0].value = lux_f;
                    // doiot.upload_en = 1;
                // }
                if (max44009_set_threshold_etc(&dev_m, &params_m, lux_f, lux_raw) != ESP_OK)
                {
                    printf("Threshold setting failed\n");
                    continue;
                }
            }
		}
	}
}
// void max44009_read(void *pvParameters)
// {
//     float lux;
//     uint8_t lux_raw;
//     while(1)
//     {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
        
//         if (max44009_read_float(&dev_m, &lux, &lux_raw) != ESP_OK)
//         {
//             printf("Temperature/pressure reading failed\n");
//             continue;
//         }
//         printf("Lux: %.3f\n", lux);
//     }
// }

// void max44009_th_test(void *pvParameters)
// {
//     float lux;
//     uint8_t lux_raw;
//     while(1)
//     {
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//         if (max44009_read_float(&dev_m, &lux, &lux_raw) != ESP_OK)
//         {
//             printf("Temperature/pressure reading failed\n");
//         }
//         printf("init Lux: %.3f\n", lux);
//         if (max44009_set_threshold_etc(&dev_m, &params_m, lux, lux_raw) != ESP_OK)
//         {
//             printf("Threshold setting failed\n");
//         }

//         max44009_read_regs(&dev_m);
//     }
    
// }



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

	++l;
    while (src[l] != '\0'){
        if (src[l] == ',') break;
        dst[i] = src[l];
        ++i;
        ++l;
    }
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

void doiot_getevent(const char * data)
{
    char tmp[10];
    //doiot_event_t* doiot = get_doiot();
    if (is_message_in_str(data, "MIPLDISCOVER")) {
        // +MIPLDISCOVER: 0, 15321, 3303
        mystrcpy(data, tmp, 2);// get object id from data (2: obj_id, 1: msgid)
        //ok
        for(size_t i = 0; i < DOIOT_OBJ_NUM; i++) {
            if (!strcmp(tmp, obj[i].id)){ //&& !obj[i].discover) {
                // doiot object operation
                obj[i].discover = 1;
                mystrcpy(data, obj[i].msgid_discover, 1);// copy msgid to object
                // doiot event operation
                doiot.cur_obj = i; // save cur obj index
                doiot.event = DOIOT_DISCOVER;
                break;
            }
        }
    } else if (is_message_in_str(data, "MIPLOBSERVE")) {
        //+MIPLOBSERVE: 0, 80856, 1, 3303, 0, -1
        mystrcpy(data, tmp, 3);// get object id from data (3: obj_id, 1: msgid)
        for(size_t i = 0; i < DOIOT_OBJ_NUM; i++) {
            if (!strcmp(tmp, obj[i].id) ){//&& !obj[i].observe) {
                // doiot object operation
                obj[i].observe = 1;
                mystrcpy(data, obj[i].msgid_observe, 1);// copy msgid to object
                // doiot event operation
                doiot.cur_obj = i; // save cur obj index
                doiot.event = DOIOT_OBSERVE;
                break;
            }
        }
    } else if (is_message_in_str(data, "+MIPLEVENT: 0, 6")) {
        doiot.event = DOIOT_REG_SUCCESS;
        return;
    } else if (is_message_in_str(data, "+IP:")) {
        doiot.event = DOIOT_IP_CONNECTED;
        return;
        // printf("event: get_ip\n");
    } else {
        doiot.event = DOIOT_NORMAL;    // set to DOIOT_NORMAL
        return;
    }
    return;   
}
//+MIPLOBSERVE: 0, 80856, 1, 3303, 0, -1
// +MIPLDISCOVER: 0, 15321, 3303
void doiot_response(const char* data)
{
    // const char* miplobserve = "MIPLOBSERVE";
    char cmd[80];
    //doiot_event_t* doiot = get_doiot();

    switch (doiot.event)
    {
        case DOIOT_NORMAL:
            break;
        // when RSP, only msgid is needed
        case DOIOT_OBSERVE:
            strcpy(cmd, "AT+MIPLOBSERVERSP=0,");
            strcat(cmd, obj[doiot.cur_obj].msgid_observe);
            strcat(cmd, ",1\r\n");
            uart_sendstring(UART_NUM_1, cmd);
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // uart_sendstring(UART_NUM_0, cmd);
            doiot.event = DOIOT_NORMAL;
            // doiot.cur_obj = -1;
            // doiot.observe_count++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
            // AT+MIPLOBSERVERSP=0, ,1
            // AT+MIPLDISCOVERRSP=0, ,1,14,"5700;5601;5602"
        case DOIOT_DISCOVER:
            strcpy(cmd, "AT+MIPLDISCOVERRSP=0,");
            strcat(cmd, obj[doiot.cur_obj].msgid_discover);
            strcat(cmd, ",1,14,\"5700;5601;5602\"\r\n"); // specific attribute for object
            // uart_sendstring(UART_NUM_0, cmd);
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            uart_sendstring(UART_NUM_1, cmd);
            doiot.event = DOIOT_NORMAL;
            // doiot.cur_obj = -1;
            // doiot.discover_count++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        case DOIOT_IP_CONNECTED:
            doiot.flag_ip = 1;
            break;
        case DOIOT_REG_SUCCESS:
            doiot.flag_miplopen = 1;
            break;
        default:
            break;
    }
    return;
}

/**
 * uart0:   pc  -- esp32
 * uart1: esp32 -- doiot
 */
void uart_forward()
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data0 = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *data1 = (uint8_t *) malloc(BUF_SIZE);
    const char *u_esp32 = {"\nesp32: "};
    // const char *u_doiot = {"doiot: "};
    printf("init\n");
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // Read data from the UART0
        int len0 = uart_read_bytes(UART_NUM_0, data0, BUF_SIZE, 20 / portTICK_RATE_MS);
        int len1 = uart_read_bytes(UART_NUM_1, data1, BUF_SIZE, 20 / portTICK_RATE_MS);

        if (len0 > 0) {//receve from pc
            uart_write_bytes(UART_NUM_0, u_esp32, 8);
            uart_write_bytes(UART_NUM_0, (const char *) data0, len0);    //display back
            // if (is_message_in_str((const char *)data0, "shut")) {
            //     uart_sendstring(UART_NUM_1, "AT+MIPLCLOSE=0\r\n"); // MIPLCLOSE first
            //     gpio_set_level(GPIO_PWR_DOIOT, 1);
            //     vTaskDelay(5000 / portTICK_PERIOD_MS);
            //     gpio_set_level(GPIO_PWR_DOIOT, 0);
            // } else if (is_message_in_str((const char *)data0, "close")) {
            //     uart_sendstring(UART_NUM_1, "AT+MIPDISCOVER=0\r\n");
            //     // free(doiot);
            //     // for (size_t i = 0; i < DOIOT_OBJ_NUM; i++) {
            //     //     free(obj[i]);
            //     // }
                
            //     // init_doiot();
            // }
            uart_write_bytes(UART_NUM_1, (const char *) data0, len0);    //send to doiot
        }
        if (len1 > 0) {//receive from doiot
            // printf("uart1[R]: %d\n", len1);
            // deal with response from doiot
            //
            // uart_write_bytes(UART_NUM_0, u_doiot, 7);
            uart_write_bytes(UART_NUM_0, (const char *) data1, len1);

            // once receive data from doiot, process the data;
            doiot_getevent((const char *) data1);
            doiot_response((const char *) data1);
        }
    }
}

void doiot_upload()
{
    char cmd[50];
    char buf[10];
    //doiot_event_t* doiot = get_doiot();
    printf("nbiot_upload task start!\n");
    while(1){
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        vTaskDelay(1 * 60 * 1000 / portTICK_PERIOD_MS);
        if (!doiot.upload_en && doiot.discover_count == 4) {
            doiot.upload_en = 1;
        }
        
        if (doiot.flag_miplopen && doiot.upload_en) {
            
            for(size_t i = 0; i < DOIOT_OBJ_NUM; i++)
            {
                // AT+MIPLNOTIFY=0,114453,3303,0,5700,4,4,25.1,0,0
                strcpy(cmd, "AT+MIPLNOTIFY=0,");
                strcat(cmd, obj[i].msgid_observe);
                strcat(cmd, ",");
                strcat(cmd, obj[i].id);
                strcat(cmd, ",0,5700,4,4,"); // maybe max/min need manually upload
                // float2char(num_test, buf);
                // printf("num = %.2f, buf = %s \n", num_test, buf);
                float2char(obj[i].value, buf);
                strcat(cmd, buf);
                strcat(cmd, ",0,0\r\n"); // next time try config index bit
                printf("%s", cmd);
                uart_sendstring(UART_NUM_1, cmd);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            
        }
    }
}