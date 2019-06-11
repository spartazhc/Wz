#include "esp_http_client.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

// char buffer that will contain JSON string that will be fetched
extern char http_json_message[1024*4];

// function that clears JSON buffer
void app_http_json_flush();

// event handler for http events
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
