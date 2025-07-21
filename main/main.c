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
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_system.h" // For esp_restart

// Function prototype for get_trip_statistics
void get_trip_statistics(float *total_energy, float *total_distance, float *avg_consumption);

// Global SOC variable for trip reset
static float soc = 100.0f; // Initialize to 100%, will be updated with real data

#define PORT 4210
// WiFi Configuration - Update these with your network credentials
#define WIFI_SSID "HunorCar"
#define WIFI_PASSWORD "Jucymucy8581"

#define NTP_SERVER "pool.ntp.org"
#define TIMEZONE_OFFSET 1 * 3600 
// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Chart data storage
#define CHART_DATA_POINTS 60
static float kw_data[CHART_DATA_POINTS];
static float soc_data[CHART_DATA_POINTS];
static int data_index = 0;
static bool chart_initialized = false;
static SemaphoreHandle_t chart_mutex = NULL;
static bool time_synchronized = false;
static char current_time_str[32] = "00:00:00";
static char current_date_str[32] = "2024-01-01";
// Chart series references - store them once during initialization
static lv_chart_series_t * kw_series = NULL;
static lv_chart_series_t * soc_series = NULL;
static bool first_data_received = false;
static TickType_t last_chart_update = 0;
static const TickType_t CHART_UPDATE_INTERVAL = pdMS_TO_TICKS(60000); // 60 seconds


// Data queue for thread-safe communication
typedef struct {
    float soc;
    float range;
    float temp;
    float amp;
    float volt;
    float watt;
    float consumption;  // Add this field
    float speed;        // Add this field
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
extern lv_obj_t * uic_Label9;
extern lv_obj_t * uic_Label8; 
// Screen2 chart elements
extern lv_obj_t * ui_Kwchart;
extern lv_obj_t * uic_Kwchart;
extern lv_obj_t * uic_clock_time;   // Time display label
extern lv_obj_t * uic_clock_date;   // Date display label
#define CONSUMPTION_SAMPLES 30  // Number of samples for rolling average
static float consumption_history[CONSUMPTION_SAMPLES];
static int consumption_index = 0;
static float total_energy_consumed = 0.0f;  // kWh
static float total_distance_traveled = 0.0f;  // km
static float last_timestamp = 0.0f;
static float current_consumption = 0.0f;  // kW/100km
static bool consumption_initialized = false;
static float soc_start = 100.0f; // Default to 100%, set on trip reset

// Add extern declaration for your consumption display UI element
extern lv_obj_t * uic_consumption_label;  // You'll need to add this to your UI
extern lv_obj_t * uic_avgconsumption;
extern lv_obj_t * uic_totaldistancetraveled;
extern lv_obj_t * uic_totalenergyconsumed;

// Function to calculate consumption based on current power and speed
float calculate_consumption(float power_kw, float speed_kmh) {
    // Only calculate when vehicle is moving (speed > 1 km/h to avoid division by zero)
    if (speed_kmh > 1.0f && power_kw < 0.0f) {  // Only count negative power (usage)
        return (-power_kw * 100.0f) / speed_kmh;  // kW/100km, make positive
    }
    return 0.0f;  // Return 0 if not moving or not using power
}

// Enhanced consumption tracking with trip statistics
void update_trip_statistics(float power_kw, float speed_kmh, float current_time) {
    if (last_timestamp > 0.0f && current_time > last_timestamp) {
        float time_delta = current_time - last_timestamp; // in seconds
        float distance_delta = (speed_kmh * time_delta) / 3600.0f; // km
        
        // Update total distance
        total_distance_traveled += distance_delta;
        
        // Update total energy consumed (only negative power, i.e., usage)
        if (power_kw < 0.0f) { // Only count energy used (driving)
            float energy_delta = (-power_kw * time_delta) / 3600.0f; // kWh, make positive
            total_energy_consumed += energy_delta;
        }
        // Optionally, track regen/charging energy separately:
        // else if (power_kw > 0.0f) {
        //     float regen_delta = (power_kw * time_delta) / 3600.0f;
        //     total_energy_regenerated += regen_delta;
        // }
    }
    
    last_timestamp = current_time;
}
// Function to update consumption with rolling average
void update_consumption_average(float instantaneous_consumption) {
    if (!consumption_initialized) {
        // Initialize array with first value
        for (int i = 0; i < CONSUMPTION_SAMPLES; i++) {
            consumption_history[i] = instantaneous_consumption;
        }
        consumption_initialized = true;
    }
    
    // Add new value to rolling buffer
    consumption_history[consumption_index] = instantaneous_consumption;
    consumption_index = (consumption_index + 1) % CONSUMPTION_SAMPLES;
    
    // Calculate average
    float sum = 0.0f;
    int valid_samples = 0;
    
    for (int i = 0; i < CONSUMPTION_SAMPLES; i++) {
        if (consumption_history[i] > 0.0f) {  // Only count valid consumption values
            sum += consumption_history[i];
            valid_samples++;
        }
    }
    
    if (valid_samples > 0) {
        current_consumption = sum / valid_samples;
    }
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI("TIME", "Time synchronization complete");
    time_synchronized = true;
}

// Initialize NTP client
void initialize_ntp(void)
{
    ESP_LOGI("TIME", "Initializing NTP client...");
    
    // Set timezone (Oslo is UTC+1)
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);  // Central European Time with DST
    tzset();
    
    // Initialize SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    
    ESP_LOGI("TIME", "NTP client initialized, waiting for time sync...");
}

// Update time strings
void update_time_strings(void)
{
    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Format time (24-hour format)
    strftime(current_time_str, sizeof(current_time_str), "%H:%M", &timeinfo);
    
    // Format date
    strftime(current_date_str, sizeof(current_date_str), "%Y-%m-%d", &timeinfo);
}

// Clock update task
void clock_update_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Update every second
    
    while (1) {
        if (time_synchronized) {
            update_time_strings();
            
            // Update UI with LVGL lock
            if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
                // Update time display
                if (uic_clock_time != NULL) {
                    lv_label_set_text(uic_clock_time, current_time_str);
                }
                
                // Update date display
                if (uic_clock_date != NULL) {
                    lv_label_set_text(uic_clock_date, current_date_str);
                }
                
                lvgl_port_unlock();
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Initialize chart series once - call this after ui_init()
void initialize_chart_series(void) {
    if (ui_Kwchart == NULL) {
        ESP_LOGW("CHART", "Chart object not available for initialization");
        return;
    }
    
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        ESP_LOGI("CHART", "Initializing chart series...");
        
        // First, set the chart configuration BEFORE adding series
        lv_chart_set_type(ui_Kwchart, LV_CHART_TYPE_LINE);
        lv_chart_set_point_count(ui_Kwchart, CHART_DATA_POINTS);
        lv_chart_set_range(ui_Kwchart, LV_CHART_AXIS_PRIMARY_Y, -60, 100);
        
        // Create new series
        kw_series = lv_chart_add_series(ui_Kwchart, lv_color_hex(0xFF0000), LV_CHART_AXIS_PRIMARY_Y);
        soc_series = lv_chart_add_series(ui_Kwchart, lv_color_hex(0x00FF00), LV_CHART_AXIS_PRIMARY_Y);
        
        if (kw_series && soc_series) {
            ESP_LOGI("CHART", "Series created successfully, clearing initial data...");
            
            // IMPORTANT: Clear all points immediately after series creation
            // This removes the default/initialization data that LVGL adds
            lv_chart_set_all_value(ui_Kwchart, kw_series, LV_CHART_POINT_NONE);
            lv_chart_set_all_value(ui_Kwchart, soc_series, LV_CHART_POINT_NONE);
            
            // Initialize our data arrays to known values
            for (int i = 0; i < CHART_DATA_POINTS; i++) {
                kw_data[i] = 0.0f;
                soc_data[i] = 0.0f;
            }
            data_index = 0;
            
            // Force a complete chart refresh to ensure clean state
            lv_chart_refresh(ui_Kwchart);
            lv_obj_invalidate(ui_Kwchart);
            
            chart_initialized = true;
            ESP_LOGI("CHART", "Chart series initialized and cleared successfully");
        } else {
            ESP_LOGE("CHART", "Failed to create chart series");
        }
        
        lvgl_port_unlock();
    }
}
// Enhanced clear function that should be called when first real data arrives
void clear_chart_artifacts(void) {
    if (!chart_initialized || ui_Kwchart == NULL || kw_series == NULL || soc_series == NULL) {
        return;
    }
    
    ESP_LOGI("CHART", "Clearing chart artifacts...");
    
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        // Method 1: Clear all points to "no data" state
        lv_chart_set_all_value(ui_Kwchart, kw_series, LV_CHART_POINT_NONE);
        lv_chart_set_all_value(ui_Kwchart, soc_series, LV_CHART_POINT_NONE);
        
        // Method 2: Alternative - remove and recreate series (more aggressive)
        // Uncomment these lines if Method 1 doesn't work:
        /*
        lv_chart_remove_series(ui_Kwchart, kw_series);
        lv_chart_remove_series(ui_Kwchart, soc_series);
        
        kw_series = lv_chart_add_series(ui_Kwchart, lv_color_hex(0xFF0000), LV_CHART_AXIS_PRIMARY_Y);
        soc_series = lv_chart_add_series(ui_Kwchart, lv_color_hex(0x00FF00), LV_CHART_AXIS_PRIMARY_Y);
        
        if (kw_series && soc_series) {
            lv_chart_set_all_value(ui_Kwchart, kw_series, LV_CHART_POINT_NONE);
            lv_chart_set_all_value(ui_Kwchart, soc_series, LV_CHART_POINT_NONE);
        }
        */
        
        // Reset data arrays
        for (int i = 0; i < CHART_DATA_POINTS; i++) {
            kw_data[i] = 0.0f;
            soc_data[i] = 0.0f;
        }
        data_index = 0;
        
        // Force complete refresh
        lv_chart_refresh(ui_Kwchart);
        lv_obj_invalidate(ui_Kwchart);
        
        lvgl_port_unlock();
    }
}
void update_chart_display(void) {
    if (!chart_initialized || ui_Kwchart == NULL || kw_series == NULL || soc_series == NULL) {
        ESP_LOGW("CHART", "Chart not properly initialized");
        return;
    }
    
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        // Take mutex to protect chart data
        if (chart_mutex != NULL && xSemaphoreTake(chart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            // Get the most recent data point (the one we just added)
            int current_idx = (data_index - 1 + CHART_DATA_POINTS) % CHART_DATA_POINTS;
            
            // Log the actual values we're about to display
            ESP_LOGI("CHART", "Displaying chart data: kW=%.1f, SOC=%.1f (index=%d)", 
                     kw_data[current_idx], soc_data[current_idx], current_idx);
            
            // Add only the latest point (recommended for real-time updates)
            lv_chart_set_next_value(ui_Kwchart, kw_series, (lv_coord_t)kw_data[current_idx]);
            lv_chart_set_next_value(ui_Kwchart, soc_series, (lv_coord_t)soc_data[current_idx]);
            
            // Force chart refresh
            lv_obj_invalidate(ui_Kwchart);
            
            xSemaphoreGive(chart_mutex);
            
            ESP_LOGI("CHART", "Chart updated with kW=%.1f, SOC=%.1f", 
                     kw_data[current_idx], soc_data[current_idx]);
        }
        
        lvgl_port_unlock();
    }
}
void update_chart_data_safe(float kw_value, float soc_value) {
    if (!chart_initialized) {
        ESP_LOGW("CHART", "Chart not initialized, skipping update");
        return;
    }
    
    ESP_LOGI("CHART", "update_chart_data_safe called with kW=%.1f, SOC=%.1f", kw_value, soc_value);
    
    // Clear artifacts on first real data
    if (!first_data_received) {
        clear_chart_artifacts();
        first_data_received = true;
        ESP_LOGI("CHART", "First data received, clearing artifacts");
    }
    
    // Check if enough time has passed since last chart update
    TickType_t current_time = xTaskGetTickCount();
    if (current_time - last_chart_update < CHART_UPDATE_INTERVAL) {
        ESP_LOGI("CHART", "Chart update skipped - waiting for interval (time since last: %lu ms)", 
                 (unsigned long)((current_time - last_chart_update) * portTICK_PERIOD_MS));
        return; // IMPORTANT: Return here, don't call update_chart_display()
    }
    
    // Take mutex to protect chart data
    if (chart_mutex != NULL && xSemaphoreTake(chart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Validate input values
        if (kw_value < -60.0f) kw_value = -60.0f;
        if (kw_value > 42.0f) kw_value = 42.0f;
        if (soc_value < 0.0f) soc_value = 0.0f;
        if (soc_value > 100.0f) soc_value = 100.0f;
        
        ESP_LOGI("CHART", "Storing data at index %d: kW=%.1f, SOC=%.1f", data_index, kw_value, soc_value);
        
        // Store data in circular buffer
        kw_data[data_index] = kw_value;
        soc_data[data_index] = soc_value;
        
        // Move to next index (circular buffer)
        data_index = (data_index + 1) % CHART_DATA_POINTS;
        
        // Update the last update time
        last_chart_update = current_time;
        
        xSemaphoreGive(chart_mutex);
        
        ESP_LOGI("CHART", "Chart data stored successfully: kW=%.1f, SOC=%.1f%%", kw_value, soc_value);
        
        // ONLY call update_chart_display() when we actually stored new data
        update_chart_display();
    } else {
        ESP_LOGE("CHART", "Failed to acquire chart mutex");
    }
}

void ui_update_task_with_consumption(void *pvParameters) {
    sensor_data_t data;

    while (1) {
        // Wait for data from queue
        if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("UI", "Updating UI with received data");
            
            // Convert watts to kW
            float watt_kw = data.watt / 1000.0f;
            
            // Update chart data (thread-safe)
            update_chart_data_safe(watt_kw, data.soc);
            
            // Update UI elements - must be done with LVGL lock
            if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
                // Validate UI objects before using them
                        if (ui_SOC != NULL) {
                            char soc_text[16];
                            snprintf(soc_text, sizeof(soc_text), "%.1f%%", data.soc);
                            lv_label_set_text(ui_SOC, soc_text);
                        }
                        if (uic_Label9 != NULL) {
                            char soc_text[16];
                            snprintf(soc_text, sizeof(soc_text), "%.1f%%", data.soc);
                            lv_label_set_text(uic_Label9, soc_text);
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
                    snprintf(watt_text, sizeof(watt_text), "%.1fkw", watt_kw);
                    lv_label_set_text(ui_labelbattw, watt_text);
                }
                if (uic_Label8 != NULL) {
                    char watt_text[16];
                    snprintf(watt_text, sizeof(watt_text), "%.1fkW", watt_kw);
                    lv_label_set_text(uic_Label8, watt_text);
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
                    snprintf(amp_text, sizeof(amp_text), "%.1fA", data.amp);
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
                    volt_arc_value = (volt_arc_value < 0) ? 0 : (volt_arc_value > 370) ? 370 : volt_arc_value;
                    lv_arc_set_value(ui_volts, volt_arc_value);
                }
                
                if (ui_wifi != NULL) {
                    lv_label_set_text(ui_wifi, "Connected");
                }

                if (uic_consumption_label != NULL) {
                    char consumption_text[32];
                    if (current_consumption > 0.0f) {
                        snprintf(consumption_text, sizeof(consumption_text), "%.1f kW/100km", current_consumption);
                    } else {
                        snprintf(consumption_text, sizeof(consumption_text), "-- kW/100km");
                    }
                    lv_label_set_text(uic_consumption_label, consumption_text);
                }
                // Update consumption label
                if (uic_avgconsumption) {
                    float total_energy, total_distance, avg_consumption;
                    get_trip_statistics(&total_energy, &total_distance, &avg_consumption);
                    char buf[32];
                    if (total_distance > 0.0f) {
                        snprintf(buf, sizeof(buf), "%.2f kWh/100km", avg_consumption);
                    } else {
                        snprintf(buf, sizeof(buf), "-- kWh/100km");
                    }
                    lv_label_set_text(uic_avgconsumption, buf);
                }
                // Update total distance traveled label
                if (uic_totaldistancetraveled) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%.1f km", total_distance_traveled);
                    lv_label_set_text(uic_totaldistancetraveled, buf);
                }
                // Update total energy consumed label
                if (uic_totalenergyconsumed) {
                    char buf[64];
                    // Integration-based value
                    snprintf(buf, sizeof(buf), "%.2f kWh", total_energy_consumed);
                    // SOC-based value
                    float energy_used_Wh = (soc_start - data.soc) / 100.0f * 14000.0f;
                    float energy_used_kWh = energy_used_Wh / 1000.0f;
                    char soc_buf[32];
                    snprintf(soc_buf, sizeof(soc_buf), "%.2f kWh", energy_used_kWh);
                    strncat(buf, soc_buf, sizeof(buf) - strlen(buf) - 1);
                    lv_label_set_text(uic_totalenergyconsumed, buf);
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

// Enhanced process_received_data function with trip statistics
void process_received_data_with_consumption(float soc_in, float range, float temp, float amp, float volt, float watt, float speed_kmh) {
    ESP_LOGI("DATA", "Received data - SOC: %.1f%%, Range: %.1f km, Temp: %.1f°C, Amp: %.1f A, Volt: %.1f V, Watt: %.1f W, Speed: %.1f km/h", 
             soc_in, range, temp, amp, volt, watt, speed_kmh);
    
    // Update global soc variable
    soc = soc_in;
    
    // Convert watts to kW
    float power_kw = watt / 1000.0f;
    
    // Get current time for trip statistics
    float current_time = (float)esp_timer_get_time() / 1000000.0f; // Convert to seconds
    
    // Update trip statistics
    update_trip_statistics(power_kw, speed_kmh, current_time);
    
    // Calculate instantaneous consumption
    float instantaneous_consumption = calculate_consumption(power_kw, speed_kmh);
    
    // Update rolling average
    if (instantaneous_consumption > 0.0f) {
        update_consumption_average(instantaneous_consumption);
    }
    
    // Create data structure
    sensor_data_t data = {
        .soc = soc_in,
        .range = range,
        .temp = temp,
        .amp = amp,
        .volt = volt,
        .watt = watt,
        .consumption = current_consumption,
        .speed = speed_kmh
    };
    
    // Send to queue (non-blocking)
    if (sensor_data_queue != NULL) {
        if (xQueueSend(sensor_data_queue, &data, 0) != pdTRUE) {
            ESP_LOGW("DATA", "Failed to queue sensor data");
        }
    }
}

// Function to get trip statistics (call this from UI or logging)
void get_trip_statistics(float *total_energy, float *total_distance, float *avg_consumption) {
    *total_energy = total_energy_consumed;
    *total_distance = total_distance_traveled;
    
    // Calculate average consumption for the entire trip
    if (total_distance_traveled > 0.0f) {
        *avg_consumption = (total_energy_consumed * 100.0f) / total_distance_traveled;
    } else {
        *avg_consumption = 0.0f;
    }
}

// Function to reset trip statistics
void reset_trip_statistics(void) {
    total_energy_consumed = 0.0f;
    total_distance_traveled = 0.0f;
    last_timestamp = 0.0f;
    soc_start = soc; // soc is your current SOC value
    ESP_LOGI("TRIP", "Trip statistics reset");
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

void wifi_init_sta_with_ntp(void) {
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
        
        // Initialize NTP after WiFi connection
        initialize_ntp();
        
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
                cJSON *speed_item = cJSON_GetObjectItem(json, "speed_kmh");  // Changed from "speed" to "speed_kmh"

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
                    float speed = 0.0f;

                    // Use speed if available, otherwise calculate from range changes
                    if (cJSON_IsNumber(speed_item)) {
                        speed = (float)speed_item->valuedouble;
                    }
                     process_received_data_with_consumption(soc, range, temp, battA, battV, battW, speed);
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

    // Initialize WiFi with NTP
    ESP_LOGI("MAIN", "Initializing WiFi...");
    wifi_init_sta_with_ntp();  // Use the modified function

    // Start UDP server task
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    
    // Start UI update task
    xTaskCreate(ui_update_task_with_consumption, "ui_update", 4096, NULL, 4, NULL);
    
    // Start clock update task
    xTaskCreate(clock_update_task, "clock_update", 2048, NULL, 3, NULL);
    
    ESP_LOGI("MAIN", "Application started successfully");

    ESP_LOGI("MAIN", "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        ui_init();
        initialize_chart_series();
        
        // Initialize default values with null checks
        if (ui_SOC != NULL) lv_label_set_text(ui_SOC, "0.0%");
        if (ui_Range != NULL) lv_label_set_text(ui_Range, "0 km");
        if (ui_labelbattw != NULL) lv_label_set_text(ui_labelbattw, "0.0");
        if (ui_labelbattamps != NULL) lv_label_set_text(ui_labelbattamps, "0.0");
        if (ui_labelcelltemp != NULL) lv_label_set_text(ui_labelcelltemp, "0.0°C");
        if (ui_labelvolt != NULL) lv_label_set_text(ui_labelvolt, "0.0");
        if (ui_wifi != NULL) lv_label_set_text(ui_wifi, "Connecting...");
        
        // Initialize clock displays
        if (uic_clock_time != NULL) lv_label_set_text(uic_clock_time, "--:--:--");
        if (uic_clock_date != NULL) lv_label_set_text(uic_clock_date, "----/--/--");
        
        lvgl_port_unlock();
    }
}