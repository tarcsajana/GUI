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
#include <freertos/queue.h>
#include "cJSON.h"

#define PORT 4210
// WiFi Configuration - Update these with your network credentials
#define WIFI_SSID "HuNor"
#define WIFI_PASSWORD "1234554321"

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Chart data storage
#define CHART_DATA_POINTS 60
static float kw_data[CHART_DATA_POINTS];
static float soc_data[CHART_DATA_POINTS];
static int data_index = 0;
static SemaphoreHandle_t chart_mutex = NULL;

// Data queue for thread-safe communication
typedef struct {
    float soc;
    float range;
    float temp;
    float amp;
    float volt;
    float watt;
} sensor_data_t;

static QueueHandle_t sensor_data_queue = NULL;

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

// Screen2 chart elements
extern lv_obj_t * ui_Kwchart;
extern lv_obj_t * uic_Kwchart;

// Safe chart update function with proper error checking
void update_chart_data_safe(float kw_value, float soc_value) {
    // Take mutex to protect chart data
    if (chart_mutex != NULL && xSemaphoreTake(chart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Store data in circular buffer
        kw_data[data_index] = kw_value;
        soc_data[data_index] = soc_value;
        
        // Move to next index (circular buffer)
        data_index = (data_index + 1) % CHART_DATA_POINTS;
        
        xSemaphoreGive(chart_mutex);
    }
}

// Separate function to update chart display - called from LVGL task only
void update_chart_display(void) {
    // Check if chart exists and is valid
    if (ui_Kwchart == NULL) {
        ESP_LOGW("CHART", "Chart object is NULL");
        return;
    }
    
    // Take mutex to protect chart data
    if (chart_mutex != NULL && xSemaphoreTake(chart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Get chart series with proper error checking
        lv_chart_series_t * kw_series = lv_chart_get_series_next(ui_Kwchart, NULL);
        if (kw_series == NULL) {
            ESP_LOGW("CHART", "kW series is NULL");
            xSemaphoreGive(chart_mutex);
            return;
        }
        
        lv_chart_series_t * soc_series = lv_chart_get_series_next(ui_Kwchart, kw_series);
        if (soc_series == NULL) {
            ESP_LOGW("CHART", "SOC series is NULL");
            xSemaphoreGive(chart_mutex);
            return;
        }
        
        // Clear existing data
        lv_chart_set_all_value(ui_Kwchart, kw_series, LV_CHART_POINT_NONE);
        lv_chart_set_all_value(ui_Kwchart, soc_series, LV_CHART_POINT_NONE);
        
        // Add data points in order (oldest to newest)
        for (int i = 0; i < CHART_DATA_POINTS; i++) {
            int idx = (data_index + i) % CHART_DATA_POINTS;
            
            // Validate data before adding
            float kw_val = kw_data[idx];
            float soc_val = soc_data[idx];
            
            // Clamp values to reasonable ranges
            if (kw_val < -60.0f) kw_val = -60.0f;
            if (kw_val > 42.0f) kw_val = 42.0f;
            if (soc_val < 0.0f) soc_val = 0.0f;
            if (soc_val > 100.0f) soc_val = 100.0f;
            
            // Add kW data (series 1)
            lv_chart_set_next_value(ui_Kwchart, kw_series, (lv_coord_t)kw_val);
            
            // Add SOC data (series 2)
            lv_chart_set_next_value(ui_Kwchart, soc_series, (lv_coord_t)soc_val);
        }
        
        // Refresh the chart
        lv_chart_refresh(ui_Kwchart);
        
        xSemaphoreGive(chart_mutex);
    }
}

// UI update task - runs in LVGL context
void ui_update_task(void *pvParameters) {
    sensor_data_t data;
    
    while (1) {
        // Wait for data from queue
        if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("UI", "Updating UI with received data");
            
            // Convert watts to kW
            float watt_kw = data.watt / 1000.0f;
            
            // Update chart data (thread-safe)
            //update_chart_data_safe(watt_kw, data.soc);
            
            // Update UI elements - must be done with LVGL lock
            if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
                // Validate UI objects before using them
                if (ui_SOC != NULL) {
                    char soc_text[16];
                    snprintf(soc_text, sizeof(soc_text), "%.1f%%", data.soc);
                    lv_label_set_text(ui_SOC, soc_text);
                }
                
                if (ui_SOCbar != NULL) {
    int soc_val = (int)data.soc;
    soc_val = (soc_val < 0) ? 0 : (soc_val > 100) ? 100 : soc_val;
    lv_bar_set_value(ui_SOCbar, soc_val, LV_ANIM_ON);

    // Set color based on SOC range
    lv_color_t bar_color;

    if (soc_val <= 10) {
        bar_color = lv_color_hex(0xFF0000); // Red
    } else if (soc_val <= 25) {
        bar_color = lv_color_hex(0xFFA500); // Orange
    } else if (soc_val > 90) {
        bar_color = lv_color_hex(0x00FF00); // Green
    } else {
        bar_color = lv_color_hex(0x3366CC); // Default/original color (adjust if needed)
    }

    lv_obj_set_style_bg_color(ui_SOCbar, bar_color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
}
                
                if (ui_Range != NULL) {
                    char range_text[16];
                    snprintf(range_text, sizeof(range_text), "%.1f km", data.range);
                    lv_label_set_text(ui_Range, range_text);
                }
                
                if (ui_labelbattw != NULL) {
                    char watt_text[16];
                    snprintf(watt_text, sizeof(watt_text), "%.1f", watt_kw);
                    lv_label_set_text(ui_labelbattw, watt_text);
                }
                
                if (ui_battw != NULL) {
    // Clamp watt value between -60,000 and +42,000 W
    float clamped_watt = (data.watt < -60000.0f) ? -60000.0f : (data.watt > 42000.0f) ? 42000.0f : data.watt;

    // Update arc value directly using LVGL's arc value range (-60 to +42 kW)
    int arc_value = (int)(clamped_watt / 1000.0f);  // From watts to kW
    lv_arc_set_value(ui_battw, arc_value);

    // Set color based on sign
    if (arc_value >= 0) {
        lv_obj_set_style_arc_color(ui_battw, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT); // Green
    } else {
        lv_obj_set_style_arc_color(ui_battw, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT); // Red
    }
}
                
                if (ui_labelbattamps != NULL) {
                    char amp_text[16];
                    snprintf(amp_text, sizeof(amp_text), "%.1f", data.amp);
                    lv_label_set_text(ui_labelbattamps, amp_text);
                }
                
                if (ui_battamps != NULL) {
                    // Update arc value for amps (-150A to +150A range)
                    int amp_arc_value = (int)data.amp;
                    amp_arc_value = (amp_arc_value < 0) ? 0 : (amp_arc_value > 100) ? 100 : amp_arc_value;
                    lv_arc_set_value(ui_battamps, amp_arc_value);
                }
                
                if (ui_labelcelltemp != NULL) {
                    char temp_text[16];
                    snprintf(temp_text, sizeof(temp_text), "%.1f°C", data.temp);
                    lv_label_set_text(ui_labelcelltemp, temp_text);
                }
                
                if (ui_celltemp != NULL) {
    // Clamp temperature to safe display range
    int temp_arc_value = (int)data.temp;
    temp_arc_value = (temp_arc_value < -10) ? -10 : (temp_arc_value > 100) ? 100 : temp_arc_value;

    // Set arc value (e.g., shift -10 to 0 range into 0–100 for arc if needed)
    lv_arc_set_value(ui_celltemp, temp_arc_value);  // Optional: map range depending on your arc min/max

    // Set arc color based on temperature range
    lv_color_t arc_color;
    if (temp_arc_value <= 10) {
        arc_color = lv_color_hex(0x0000FF); // Blue
    } else if (temp_arc_value <= 20) {
        arc_color = lv_color_hex(0x90EE90); // Light Green
    } else if (temp_arc_value <= 35) {
        arc_color = lv_color_hex(0x00FF00); // Green
    } else {
        arc_color = lv_color_hex(0xFF0000); // Red
    }

    lv_obj_set_style_arc_color(ui_celltemp, arc_color, LV_PART_INDICATOR | LV_STATE_DEFAULT);
}
                
                if (ui_labelvolt != NULL) {
                    char volt_text[16];
                    snprintf(volt_text, sizeof(volt_text), "%.1f", data.volt);
                    lv_label_set_text(ui_labelvolt, volt_text);
                }
                
                if (ui_volts != NULL) {
                    // Update arc value for voltage (0V to 370V range)
                    int volt_arc_value = (int)data.volt;
                    volt_arc_value = (volt_arc_value < 0) ? 0 : (volt_arc_value > 100) ? 100 : volt_arc_value;
                    lv_arc_set_value(ui_volts, volt_arc_value);
                }
                
                if (ui_wifi != NULL) {
                    lv_label_set_text(ui_wifi, "Connected");
                }
                
                // Update chart display
                //update_chart_display();
                
                lvgl_port_unlock();
            } else {
                ESP_LOGW("UI", "Failed to acquire LVGL lock");
            }
        }
    }
}

// Simplified data processing function - now just queues data
void process_received_data(float soc, float range, float temp, float amp, float volt, float watt) {
    ESP_LOGI("DATA", "Received data - SOC: %.1f%%, Range: %.1f km, Temp: %.1f°C, Amp: %.1f A, Volt: %.1f V, Watt: %.1f W", 
             soc, range, temp, amp, volt, watt);
    
    // Create data structure
    sensor_data_t data = {
        .soc = soc,
        .range = range,
        .temp = temp,
        .amp = amp,
        .volt = volt,
        .watt = watt
    };
    
    // Send to queue (non-blocking)
    if (sensor_data_queue != NULL) {
        if (xQueueSend(sensor_data_queue, &data, 0) != pdTRUE) {
            ESP_LOGW("DATA", "Failed to queue sensor data");
        }
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
            ESP_LOGI("WIFI", "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI("WIFI", "Failed to connect to WiFi");
        }
        ESP_LOGI("WIFI", "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
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

    ESP_LOGI("WIFI", "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI", "Connected to AP SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("WIFI", "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE("WIFI", "UNEXPECTED EVENT");
    }

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
        ESP_LOGE("UDP", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE("UDP", "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI("UDP", "UDP server listening on port %d", PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            rx_buffer[len] = 0; // Null-terminate
            ESP_LOGI("UDP", "Received %d bytes from %s:%d", len, 
                     inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            
            cJSON *json = cJSON_Parse(rx_buffer);
            if (json) {
                cJSON *soc_item = cJSON_GetObjectItem(json, "soc");
                cJSON *range_item = cJSON_GetObjectItem(json, "range");
                cJSON *temp_item = cJSON_GetObjectItem(json, "celltemp");
                cJSON *battA_item = cJSON_GetObjectItem(json, "battA");
                cJSON *battV_item = cJSON_GetObjectItem(json, "battV");
                cJSON *battW_item = cJSON_GetObjectItem(json, "battW");
                
                // Check if all required fields exist and are numbers
                if (cJSON_IsNumber(soc_item) && cJSON_IsNumber(range_item) && 
                    cJSON_IsNumber(temp_item) && cJSON_IsNumber(battA_item) && 
                    cJSON_IsNumber(battV_item) && cJSON_IsNumber(battW_item)) {
                    
                    float soc = (float)soc_item->valuedouble;
                    float range = (float)range_item->valuedouble;
                    float temp = (float)temp_item->valuedouble;
                    float battA = (float)battA_item->valuedouble;
                    float battV = (float)battV_item->valuedouble;
                    float battW = (float)battW_item->valuedouble;
                    
                    process_received_data(soc, range, temp, battA, battV, battW);
                } else {
                    ESP_LOGW("UDP", "JSON missing required fields or invalid types");
                }
                cJSON_Delete(json);
            } else {
                ESP_LOGW("UDP", "Failed to parse JSON: %s", rx_buffer);
            }
        } else if (len < 0) {
            ESP_LOGE("UDP", "recvfrom failed: errno %d", errno);
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}

void app_main() {
    // Initialize chart data arrays
    for (int i = 0; i < CHART_DATA_POINTS; i++) {
        kw_data[i] = 0.0f;
        soc_data[i] = 0.0f;
    }
    
    // Create synchronization objects
    chart_mutex = xSemaphoreCreateMutex();
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));
    
    if (chart_mutex == NULL || sensor_data_queue == NULL) {
        ESP_LOGE("MAIN", "Failed to create synchronization objects");
        return;
    }
    
    waveshare_esp32_s3_rgb_lcd_init();
    
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
    ESP_LOGI("MAIN", "Initializing WiFi...");
    wifi_init_sta();

    // Start UDP server task
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    
    // Start UI update task
    xTaskCreate(ui_update_task, "ui_update", 4096, NULL, 4, NULL);
    
    ESP_LOGI("MAIN", "Application started successfully");

    ESP_LOGI("MAIN", "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        ui_init();
        
        // Initialize default values with null checks
        if (ui_SOC != NULL) lv_label_set_text(ui_SOC, "0.0%");
        if (ui_Range != NULL) lv_label_set_text(ui_Range, "0 km");
        if (ui_labelbattw != NULL) lv_label_set_text(ui_labelbattw, "0.0");
        if (ui_labelbattamps != NULL) lv_label_set_text(ui_labelbattamps, "0.0");
        if (ui_labelcelltemp != NULL) lv_label_set_text(ui_labelcelltemp, "0.0°C");
        if (ui_labelvolt != NULL) lv_label_set_text(ui_labelvolt, "0.0");
        if (ui_wifi != NULL) lv_label_set_text(ui_wifi, "Connecting...");
        
        lvgl_port_unlock();
    }
}