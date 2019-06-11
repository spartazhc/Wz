#include "app_rest.h"
#include "app_http.h"


// function that fetches REST resource
void app_rest_fetch()
{
    // flush json buffer
    app_http_json_flush();

    // form valid API URI by concatenating base URI with api key
    char URI[256] = "";
    strcat(URI, API_URI);
    strcat(URI, API_KEY);

    esp_http_client_config_t config = {
        .url = URI,
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        // ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
        //         esp_http_client_get_status_code(client),
        //         esp_http_client_get_content_length(client));
    } else {
        // ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}