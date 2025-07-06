#include "waveshare_rgb_lcd_port.h"
#include "ui.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "cJSON.h"

#define PORT 4210
// WiFi Configuration - Update these with your network credentials
#define WIFI_SSID "HuNor"
#define WIFI_PASSWORD "1234554321"

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const int WIFI_MAXIMUM_RETRY = 5;

// Add these extern declarations to access UI elements
extern lv_obj_t * ui_SOC;
extern lv_obj_t * ui_Range;
extern lv_obj_t * ui_SOCbar;
extern lv_obj_t * ui_battw;
extern lv_obj_t * ui_battamps;
extern lv_obj_t * ui_celltemp;
extern lv_obj_t * ui_volts;
extern lv_obj_t * ui_labelbattw;
extern lv_obj_t * ui_labelbattamps;
extern lv_obj_t * ui_labelcelltemp;
extern lv_obj_t * ui_labelvolt;
extern lv_obj_t * ui_wifi;

// Data processing function - now updates UI elements
void process_received_data(float soc, float range, float temp, float amp, float volt, float watt) {
    ESP_LOGI(TAG, "Received data - SOC: %.1f%%, Range: %.1f km, Temp: %.1f°C, Amp: %.1f A, Volt: %.1f V, Watt: %.1f W", 
             soc, range, temp, amp, volt, watt);
    
    // Update UI elements - must be done with LVGL lock
    if (lvgl_port_lock(-1)) {
        // Update SOC percentage and bar
        char soc_text[16];
        snprintf(soc_text, sizeof(soc_text), "%.1f%%", soc);
        lv_label_set_text(ui_SOC, soc_text);
        lv_bar_set_value(ui_SOCbar, (int)soc, LV_ANIM_ON);
        
        // Update Range
        char range_text[16];
        snprintf(range_text, sizeof(range_text), "%.1f km", range);
        lv_label_set_text(ui_Range, range_text);       
    
        // Update battery watts
        char watt_text[16];
        snprintf(watt_text, sizeof(watt_text), "%.1f", watt);
        lv_label_set_text(ui_labelbattw, watt_text);
        // Update arc value (assuming max 50kW for scaling)
        lv_arc_set_value(ui_battw, (int)(watt * 42 / 50));
        // Update battery amps
        char amp_text[16];
        snprintf(amp_text, sizeof(amp_text), "%.1f", amp);
        lv_label_set_text(ui_labelbattamps, amp_text);
        // Update arc value (assuming max 200A for scaling)
        lv_arc_set_value(ui_battamps, (int)(amp * 42 / 150));
        // Update cell temperature
        char temp_text[16];
        snprintf(temp_text, sizeof(temp_text), "%.1f°C", temp);
        lv_label_set_text(ui_labelcelltemp, temp_text);
        // Update arc value (assuming max 60°C for scaling)
        lv_arc_set_value(ui_celltemp, (int)(temp * 42 / 60));
        // Update voltage
        char volt_text[16];
        snprintf(volt_text, sizeof(volt_text), "%.1f", volt);
        lv_label_set_text(ui_labelvolt, volt_text);
        // Update arc value (assuming max 500V for scaling)
        lv_arc_set_value(ui_volts, (int)(volt * 42 / 500));
      
        // WiFi status is now updated by the periodic task
        lv_label_set_text(ui_wifi, "Connected");
        lvgl_port_unlock();
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
               
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG, "Failed to connect to WiFi");
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Wait until either the connection is established (WIFI_CONNECTED_BIT) or connection failed */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* Clean up event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "UDP server listening on port %d", PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            rx_buffer[len] = 0; // Null-terminate
            ESP_LOGI(TAG, "Received %d bytes from %s:%d", len, 
                     inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            
            cJSON *json = cJSON_Parse(rx_buffer);
            if (json) {
                cJSON *soc_item = cJSON_GetObjectItem(json, "soc");
                cJSON *range_item = cJSON_GetObjectItem(json, "range");
                cJSON *temp_item = cJSON_GetObjectItem(json, "celltemp");
                cJSON *battA_item = cJSON_GetObjectItem(json, "battA");
                cJSON *battV_item = cJSON_GetObjectItem(json, "battV");
                cJSON *battW_item = cJSON_GetObjectItem(json, "battW");
                
                // Check if all required fields exist
                if (soc_item && range_item && temp_item && battA_item && battV_item && battW_item) {
                    float soc = soc_item->valuedouble;
                    float range = range_item->valuedouble;
                    float temp = temp_item->valuedouble;
                    float battA = battA_item->valuedouble;
                    float battV = battV_item->valuedouble;
                    float battW = battW_item->valuedouble;
                    
                    process_received_data(soc, range, temp, battA, battV, battW);
                } else {
                    ESP_LOGW(TAG, "JSON missing required fields");
                }
                cJSON_Delete(json);
            } else {
                ESP_LOGW(TAG, "Failed to parse JSON: %s", rx_buffer);
            }
        } else if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}

void app_main()
{
     waveshare_esp32_s3_rgb_lcd_init(); // Initialize the Waveshare ESP32-S3 RGB LCD 
    // wavesahre_rgb_lcd_bl_on();  //Turn on the screen backlight 
    // wavesahre_rgb_lcd_bl_off(); //Turn off the screen backlight 
     // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Start UDP server task
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Application started successfully");

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        // lv_demo_stress();
        ui_init();
        
        // Initialize default values
        lv_label_set_text(ui_SOC, "0.0%");
        lv_label_set_text(ui_Range, "0.0 km");
        lv_label_set_text(ui_labelbattw, "0.0");
        lv_label_set_text(ui_labelbattamps, "0.0");
        lv_label_set_text(ui_labelcelltemp, "0.0°C");
        lv_label_set_text(ui_labelvolt, "0.0");
        lv_label_set_text(ui_wifi, "Connecting...");
        
        // lv_demo_benchmark();
        // lv_demo_music();
        // example_lvgl_demo_ui();
        // Release the mutex
        lvgl_port_unlock();
    }
}