#include "me3616.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
// #include <freertos/task.h>
#include <stdbool.h>

// const char* obj_list[ME3616_OBJ_NUM] = {"3301", "3303", "3304", "3323"};
// me3616_event_t* me3616 = NULL;
#define GPIO_PWR_ME3616     4
#define GPIO_RESET_ME3616   0
me3616_event_t me3616 = {0, 0, 0, 0};
me3616_obj_t obj[ME3616_OBJ_NUM] = {
    {"3301", 0, 0, 0, 0, 0, 0, 0, ""},
    {"3303", 0, 0, 0, 0, 0, 0, 0, ""},
    {"3304", 0, 0, 0, 0, 0, 0, 0, ""},
    {"3323", 0, 0, 0, 0, 0, 0, 0, ""}
};
bool flag_ok = 0;

int me3616_send_cmd(char* str, bool flag_ok, int waitms)
{
    uart_write_bytes(UART_NUM_1, (const char *) str, strlen(str));
    for (int t = 0; t < waitms; t += 50){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (flag_ok) {
            flag_ok = 0;
            return 0;
        }
    }
    return -1;
}


void me3616_power_on(void)
{
    gpio_set_level(GPIO_PWR_ME3616, 1);      /* set PWR high to poweron */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_PWR_ME3616, 0);      /* set PWR low */
}


void me3616_hardware_reset(void)
{
    gpio_set_level(GPIO_RESET_ME3616, 1);      /* set PWR high to poweron */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_RESET_ME3616, 0);      /* set PWR low */
}

// ? is there a wakeup pin?
// void me3616_wake_up(void) 
// {
// 	GPIO_SetBits(GPIOC, GPIO_Pin_6);	/* 拉高WAKE_UP引脚，唤醒模组 */
// 	delay_ms(100);
// 	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
// }

int me3616_sleep_config(int mode)
{
	int res;
	if (mode) {
		res = me3616_send_cmd("AT+ZSLR=1", flag_ok, 300);
		printf("AT+ZSLR=1\r\n");
	} else {
		res = me3616_send_cmd("AT+ZSLR=0", flag_ok, 300);
		printf("AT+ZSLR=0\r\n");
	}
	return res;
}

char* me3616_onenet_miplobserve_rsp(char* dst, const char* msgid){
    sprintf(dst, "AT+MIPLOBSERVERSP=0,%s,1\r\n", msgid);
    return dst;
}

char* me3616_onenet_mipldiscover_rsp(char* dst, const char* msgid, const char *valuestring){
    sprintf(dst, "AT+MIPLDISCOVERRSP=0,%s,1,%d,%s\r\n", msgid, 
                            (int)strlen(valuestring)-2, valuestring);
    return dst;
}
// AT+MIPLNOTIFY=0,114453,3303,0,5700,4,4,25.1,0,0
// only one instance so fixed
char* me3616_onenet_miplnotify(char* dst, const char* msgid, const char* objectid,
    int resourceid, int valuetype, const char *value, int index){
	int len = 4; // default 4
	if (valuetype == 1) len = (int) strlen(value);
    sprintf(dst, "AT+MIPLNOTIFY=0,%s,%s,0,%d,%d,%d,%s,%d,0\r\n", msgid, objectid,
        resourceid, valuetype, len, value, index);
    return dst;
}

char* me3616_onenet_miplnotify_float(char* dst, const char* msgid, const char* objectid,
    int resourceid, float value, int index){
    sprintf(dst, "AT+MIPLNOTIFY=0,%s,%s,0,%d,%d,%d,%.2f,%d,0\r\n", msgid, objectid,
        resourceid, 4, 4, value, index);
    return dst;
}


char* me3616_onenet_miplupdate(char* dst, int mode){
    sprintf(dst, "AT+MIPLUPDATE=0,3600,%d\r\n", mode);
    return dst;
}

// +MIPLREAD: 0, 65313, 3303, 0, 5700
// AT+MIPLREADRSP=0,65313,1,3303,0,5700,4,4,20.123,0,0
char* me3616_onenet_miplread_rsp(char* dst, const char* msgid, const char* objectid, 
                        const char* resourceid, int valuetype,
                        const char *value, int index)
{
	int len = 4; // default 4
	if (valuetype == 1) len = (int) strlen(value);
	sprintf(dst, "AT+MIPLREADRSP=0,%s,%d,%s,%d,%s,%d,%d,%s,%d,%d,\r\n", msgid, 1, objectid, 0, resourceid, valuetype, len, value, index,0);

	return dst;
}

char* me3616_onenet_miplread_rsp_float(char* dst, const char* msgid, const char* objectid, 
                        const char* resourceid, float value, int index)
{
	sprintf(dst, "AT+MIPLREADRSP=0,%s,%d,%s,%d,%s,%d,%d,%.2f,%d,%d,\r\n", msgid, 1, objectid, 0, resourceid, 4, 4, value, index,0);

	return dst;
}

// me3616_onenet_miplread_rsp(cmd, msgid, "3303", 5700,4,"3.3",0);

/**return   0：no update;
 *          1: max update;
 *          2: min update;
 */
int update_value(me3616_obj_t obj, float new_value){
    if (new_value > obj.max) {
        obj.max = new_value;
        return 1;
    }
    if (new_value < obj.min) {
        obj.min = new_value;
        return 2;
    }
    return 0;
}

