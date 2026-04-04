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
#include "mdns.h"
#include "rtc_pcf85063a.h"

// Function prototypes
void get_trip_statistics(float *total_energy, float *total_distance, float *avg_consumption);
void send_heater_command(const char *command);

// Global SOC variable for trip reset
static float soc = 100.0f; // Initialize to 100%, will be updated with real data

#define PORT 4210
#define UDP_RX_BUFFER_SIZE 256
// WiFi Configuration - Update these with your network credentials
#define WIFI_SSID "HunorCar"
#define WIFI_PASSWORD "Jucymucy8581"

// Set to 1 for verbose serial logging, 0 for quieter normal operation.
#define APP_DEBUG_LOGS 0

// Diesel Heater Control Configuration
#define HEATER_UDP_PORT 4211
#define HEATER_HOSTNAME "diesel-heater.local"  // mDNS hostname instead of static IP

#define NTP_SERVER "pool.ntp.org"
#define TIMEZONE_OFFSET 1 * 3600 
// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static bool time_synchronized = false;
static bool rtc_available = false;
static char current_time_str[32] = "00:00:00";
static char current_date_str[32] = "2024-01-01";
static time_t last_clock_now = 0;
static int64_t last_clock_uptime_us = 0;

static bool has_valid_system_time(void);

static bool initialize_time_from_rtc_if_available(void)
{
    if (!rtc_available) {
        return false;
    }

    rtc_datetime_t rtc_now = {0};
    esp_err_t err = rtc_pcf85063a_read(&rtc_now);
    if (err != ESP_OK) {
        ESP_LOGW("TIME", "RTC read failed: %s", esp_err_to_name(err));
        return false;
    }

    if (rtc_now.year < 2024 || rtc_now.year > 2069 || rtc_now.month < 1 || rtc_now.month > 12 || rtc_now.day < 1 || rtc_now.day > 31 ||
        rtc_now.hour > 23 || rtc_now.min > 59 || rtc_now.sec > 59) {
        ESP_LOGW("TIME", "RTC contains invalid date/time");
        return false;
    }

    struct tm tm_rtc = {0};
    tm_rtc.tm_year = rtc_now.year - 1900;
    tm_rtc.tm_mon = rtc_now.month - 1;
    tm_rtc.tm_mday = rtc_now.day;
    tm_rtc.tm_hour = rtc_now.hour;
    tm_rtc.tm_min = rtc_now.min;
    tm_rtc.tm_sec = rtc_now.sec;
    tm_rtc.tm_isdst = -1;

    time_t rtc_epoch = mktime(&tm_rtc);
    if (rtc_epoch <= 0) {
        ESP_LOGW("TIME", "RTC date/time conversion failed");
        return false;
    }

    struct timeval tv = {
        .tv_sec = rtc_epoch,
        .tv_usec = 0,
    };
    settimeofday(&tv, NULL);

    ESP_LOGI("TIME", "System time seeded from RTC: %04u-%02u-%02u %02u:%02u:%02u",
             rtc_now.year, rtc_now.month, rtc_now.day, rtc_now.hour, rtc_now.min, rtc_now.sec);
    return true;
}

static void update_rtc_from_system_time_if_available(void)
{
    if (!rtc_available || !has_valid_system_time()) {
        return;
    }

    time_t now = 0;
    struct tm timeinfo = {0};
    time(&now);
    localtime_r(&now, &timeinfo);

    rtc_datetime_t rtc_set = {
        .year = (uint16_t)(timeinfo.tm_year + 1900),
        .month = (uint8_t)(timeinfo.tm_mon + 1),
        .day = (uint8_t)timeinfo.tm_mday,
        .dotw = (uint8_t)timeinfo.tm_wday,
        .hour = (uint8_t)timeinfo.tm_hour,
        .min = (uint8_t)timeinfo.tm_min,
        .sec = (uint8_t)timeinfo.tm_sec,
    };

    esp_err_t err = rtc_pcf85063a_write(&rtc_set);
    if (err != ESP_OK) {
        ESP_LOGW("TIME", "RTC write failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("TIME", "RTC updated from synchronized system time");
    }
}

static void ensure_system_clock_progresses(void)
{
    time_t now = 0;
    time(&now);
    int64_t uptime_us = esp_timer_get_time();

    if (last_clock_now > 0 && now <= last_clock_now && (uptime_us - last_clock_uptime_us) > 1500000) {
        struct timeval tv = {
            .tv_sec = last_clock_now + 1,
            .tv_usec = 0,
        };
        settimeofday(&tv, NULL);
        now = last_clock_now + 1;
    }

    last_clock_now = now;
    last_clock_uptime_us = uptime_us;
}

static void configure_time_zone(void)
{
    // Central European Time with DST (Oslo)
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
}

static void configure_app_logging(void)
{
#if APP_DEBUG_LOGS
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("lvgl_port", ESP_LOG_DEBUG);
#else
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_ERROR);
    esp_log_level_set("mdns", ESP_LOG_ERROR);
    esp_log_level_set("example", ESP_LOG_ERROR);
    esp_log_level_set("lvgl_port", ESP_LOG_ERROR);
    esp_log_level_set("TIME", ESP_LOG_WARN);
    esp_log_level_set("WIFI", ESP_LOG_WARN);
    esp_log_level_set("UDP", ESP_LOG_WARN);
    esp_log_level_set("HEATER", ESP_LOG_WARN);
    esp_log_level_set("MAIN", ESP_LOG_WARN);
    esp_log_level_set("DATA", ESP_LOG_WARN);
    esp_log_level_set("UI", ESP_LOG_WARN);
    esp_log_level_set("TRIP", ESP_LOG_WARN);
    esp_log_level_set("UDS", ESP_LOG_WARN);
#endif
}

static void refresh_clock_labels_locked(void)
{
    if (uic_clock_time != NULL) {
        lv_label_set_text(uic_clock_time, current_time_str);
    }

    if (uic_clock_date != NULL) {
        lv_label_set_text(uic_clock_date, current_date_str);
    }
}

static bool has_valid_system_time(void)
{
    time_t now = 0;
    time(&now);
    // 2024-01-01 00:00:00 UTC
    return now >= 1704067200;
}

static int month_from_str(const char *mon)
{
    static const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++) {
        if (strncmp(mon, months[i], 3) == 0) {
            return i;
        }
    }
    return -1;
}

static void initialize_time_from_build_if_needed(void)
{
    if (has_valid_system_time()) {
        return;
    }

    char mon[4] = {0};
    int day = 0, year = 0;
    int hour = 0, min = 0, sec = 0;

    if (sscanf(__DATE__, "%3s %d %d", mon, &day, &year) != 3) {
        return;
    }
    if (sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec) != 3) {
        return;
    }

    int month = month_from_str(mon);
    if (month < 0) {
        return;
    }

    struct tm tm_build = {0};
    tm_build.tm_year = year - 1900;
    tm_build.tm_mon = month;
    tm_build.tm_mday = day;
    tm_build.tm_hour = hour;
    tm_build.tm_min = min;
    tm_build.tm_sec = sec;
    tm_build.tm_isdst = -1;

    time_t build_time = mktime(&tm_build);
    if (build_time <= 0) {
        return;
    }

    struct timeval tv = {
        .tv_sec = build_time,
        .tv_usec = 0,
    };
    settimeofday(&tv, NULL);
}

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

// UDS data timeout monitoring
static uint64_t last_data_received_time = 0;  // Timestamp of last received data in milliseconds
static const uint64_t DATA_TIMEOUT_MS = 20000;  // 20 seconds timeout

#define POWER_ANIM_DURATION_MS 220
#define BATTW_ARC_SCALE 10

static int32_t battw_arc_last = 0;
static int32_t battw_label_last_tenths = 0;

static void battw_arc_anim_exec_cb(void *obj, int32_t v)
{
    if (obj != NULL) {
        lv_arc_set_value((lv_obj_t *)obj, (int)v);
    }
}

static void battw_label_anim_exec_cb(void *obj, int32_t v)
{
    if (obj != NULL) {
        char watt_text[16];
        snprintf(watt_text, sizeof(watt_text), "%.1fkw", ((float)v) / 10.0f);
        lv_label_set_text((lv_obj_t *)obj, watt_text);
    }
}

static void update_power_display_animated(float watt)
{
    float clamped_watt = (watt < -60000.0f) ? -60000.0f : (watt > 60000.0f) ? 60000.0f : watt;
    float kw = clamped_watt / 1000.0f;
    // Animate arc with 0.1 kW resolution to avoid step-like jumps.
    int32_t target_arc = (int32_t)(kw * BATTW_ARC_SCALE);
    int32_t target_label_tenths = (int32_t)(kw * 10.0f);

    if (ui_battw != NULL) {
        lv_anim_del(ui_battw, battw_arc_anim_exec_cb);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, ui_battw);
        lv_anim_set_exec_cb(&a, battw_arc_anim_exec_cb);
        lv_anim_set_values(&a, battw_arc_last, target_arc);
        lv_anim_set_time(&a, POWER_ANIM_DURATION_MS);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);

        if (target_arc >= 0) {
            lv_obj_set_style_arc_color(ui_battw, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        } else {
            lv_obj_set_style_arc_color(ui_battw, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }
    }

    if (ui_labelbattw != NULL) {
        lv_anim_del(ui_labelbattw, battw_label_anim_exec_cb);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, ui_labelbattw);
        lv_anim_set_exec_cb(&a, battw_label_anim_exec_cb);
        lv_anim_set_values(&a, battw_label_last_tenths, target_label_tenths);
        lv_anim_set_time(&a, POWER_ANIM_DURATION_MS);
        lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
        lv_anim_start(&a);
    }

    battw_arc_last = target_arc;
    battw_label_last_tenths = target_label_tenths;
}

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
    update_rtc_from_system_time_if_available();
}

// Initialize NTP client
void initialize_ntp(void)
{
    ESP_LOGI("TIME", "Initializing NTP client...");

    configure_time_zone();
    
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
    
    if (!has_valid_system_time()) {
        snprintf(current_time_str, sizeof(current_time_str), "--:--");
        snprintf(current_date_str, sizeof(current_date_str), "Syncing...");
        return;
    }

    time_synchronized = true;

    // Include seconds so live updates are visible during testing.
    strftime(current_time_str, sizeof(current_time_str), "%H:%M:%S", &timeinfo);

    // Format date
    strftime(current_date_str, sizeof(current_date_str), "%Y-%m-%d", &timeinfo);
}

// Clock update task
void clock_update_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Update every second
    
    while (1) {
        ensure_system_clock_progresses();

        if (has_valid_system_time()) {
            time_synchronized = true;
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
        } else {
            if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
                if (uic_clock_time != NULL) {
                    lv_label_set_text(uic_clock_time, "Syncing...");
                }
                if (uic_clock_date != NULL) {
                    lv_label_set_text(uic_clock_date, "----/--/--");
                }
                lvgl_port_unlock();
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// UDS Data timeout monitor task
void uds_monitor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Check every 500ms
    
    while (1) {
        uint64_t current_time_ms = esp_timer_get_time() / 1000;  // Convert to milliseconds
        uint64_t time_since_last_data = current_time_ms - last_data_received_time;
        
        // Update UI with LVGL lock
        if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
            if (ui_wifi != NULL) {
                if (time_since_last_data > DATA_TIMEOUT_MS) {
                    // No data received for more than 20 seconds - show UDS failure in red
                    lv_label_set_text(ui_wifi, "UDS failure");
                    lv_obj_set_style_text_color(ui_wifi, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
                    ESP_LOGW("UDS", "UDS data timeout - %.0f seconds since last data", (float)time_since_last_data / 1000.0f);
                } else {
                    // Data is being received - show Connected in normal color
                    lv_label_set_text(ui_wifi, "Connected");
                    lv_obj_set_style_text_color(ui_wifi, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
                }
            }
            lvgl_port_unlock();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                
                update_power_display_animated(data.watt);
                
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

                if (uic_consumption_label != NULL) {
                    char consumption_text[32];
                    if (current_consumption > 0.0f) {
                        snprintf(consumption_text, sizeof(consumption_text), "%.1f", current_consumption);
                    } else {
                        snprintf(consumption_text, sizeof(consumption_text), "00.0");
                    }
                    lv_label_set_text(uic_consumption_label, consumption_text);
                }
                // Update consumption label
                if (uic_avgconsumption) {
                    float total_energy, total_distance, avg_consumption;
                    get_trip_statistics(&total_energy, &total_distance, &avg_consumption);
                    char buf[32];
                    if (total_distance > 0.0f) {
                        snprintf(buf, sizeof(buf), "%.1f", avg_consumption);
                    } else {
                        snprintf(buf, sizeof(buf), "00.0");
                    }
                    lv_label_set_text(uic_avgconsumption, buf);
                }
                // Update total distance traveled label
                if (uic_totaldistancetraveled) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%.1f", total_distance_traveled);
                    lv_label_set_text(uic_totaldistancetraveled, buf);
                }
                // Update total energy consumed label
                if (uic_totalenergyconsumed) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "%.1f", total_energy_consumed);
                    lv_label_set_text(uic_totalenergyconsumed, buf);
                }
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
    
    // Update timestamp when data is received
    last_data_received_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    // If speed is out of realistic range, do NOT abort the update —
    // ignore speed for statistics/calculation but still update UI with other values.
    bool speed_valid = (speed_kmh >= 0.0f && speed_kmh <= 130.0f);
    if (!speed_valid) {
        ESP_LOGW("DATA", "Speed %.1f km/h ignored for statistics (out of range)", speed_kmh);
    }

    // Update global soc variable
    soc = soc_in;

    // Convert watts to kW
    float power_kw = watt / 1000.0f;

    // Get current time for trip statistics
    float current_time = (float)esp_timer_get_time() / 1000000.0f; // Convert to seconds

    // Update trip statistics and consumption only when speed is valid
    if (speed_valid && speed_kmh > 1.0f) {
        update_trip_statistics(power_kw, speed_kmh, current_time);

        // Calculate instantaneous consumption
        float instantaneous_consumption = calculate_consumption(power_kw, speed_kmh);

        // Update rolling average
        if (instantaneous_consumption > 0.0f) {
            update_consumption_average(instantaneous_consumption);
        }
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
        // store negative speed to indicate invalid/unavailable speed
        .speed = speed_valid ? speed_kmh : -1.0f
    };
    
    // Keep only the newest sample so the UI never lags behind old telemetry.
    if (sensor_data_queue != NULL) {
        if (xQueueOverwrite(sensor_data_queue, &data) != pdTRUE) {
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

// Function to send UDP commands to diesel heater
void send_heater_command(const char *command) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("HEATER", "Failed to create UDP socket: errno %d", errno);
        return;
    }

    // Resolve hostname to IP address
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", HEATER_UDP_PORT);
    
    int err = getaddrinfo(HEATER_HOSTNAME, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE("HEATER", "Failed to resolve hostname '%s': %d", HEATER_HOSTNAME, err);
        close(sock);
        return;
    }

    struct sockaddr_in *dest_addr = (struct sockaddr_in *)res->ai_addr;
    ESP_LOGI("HEATER", "Resolved %s to %s", HEATER_HOSTNAME, inet_ntoa(dest_addr->sin_addr));

    err = sendto(sock, command, strlen(command), 0, 
                 (struct sockaddr *)dest_addr, sizeof(*dest_addr));
    
    if (err < 0) {
        ESP_LOGE("HEATER", "Failed to send UDP command '%s': errno %d", command, errno);
    } else {
        ESP_LOGI("HEATER", "Sent command '%s' to %s:%d", command, HEATER_HOSTNAME, HEATER_UDP_PORT);
    }

    freeaddrinfo(res);
    close(sock);
}

// Function to request and parse temperature data from heater
typedef struct {
    float air_temp;
    float car_temp;
} heater_temps_t;

heater_temps_t request_heater_temperatures(void) {
    heater_temps_t temps = {.air_temp = 0.0f, .car_temp = 0.0f};
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("HEATER", "Failed to create UDP socket for temp request: errno %d", errno);
        return temps;
    }

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 2;  // 2 second timeout
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Resolve hostname to IP address
    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", HEATER_UDP_PORT);
    
    int err = getaddrinfo(HEATER_HOSTNAME, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE("HEATER", "Failed to resolve hostname '%s': %d", HEATER_HOSTNAME, err);
        close(sock);
        return temps;
    }

    struct sockaddr_in *dest_addr = (struct sockaddr_in *)res->ai_addr;

    // Send request
    const char *request = "temps";
    err = sendto(sock, request, strlen(request), 0, 
                 (struct sockaddr *)dest_addr, sizeof(*dest_addr));
    
    if (err < 0) {
        ESP_LOGE("HEATER", "Failed to send temp request: errno %d", errno);
        freeaddrinfo(res);
        close(sock);
        return temps;
    }

    // Receive response
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                       (struct sockaddr *)&source_addr, &socklen);
    
    if (len > 0) {
        rx_buffer[len] = 0;  // Null terminate
        ESP_LOGI("HEATER", "Received temp response: %s", rx_buffer);
        
        // Parse JSON: {"air":X.X,"car":X.X}
        // Simple parsing without full JSON library
        char *air_ptr = strstr(rx_buffer, "\"air\":");
        char *car_ptr = strstr(rx_buffer, "\"car\":");
        
        if (air_ptr && car_ptr) {
            sscanf(air_ptr, "\"air\":%f", &temps.air_temp);
            sscanf(car_ptr, "\"car\":%f", &temps.car_temp);
            ESP_LOGI("HEATER", "Parsed temps - Air: %.1f°C, Car: %.1f°C", temps.air_temp, temps.car_temp);
        } else {
            ESP_LOGW("HEATER", "Failed to parse temperature JSON");
        }
    } else {
        ESP_LOGW("HEATER", "No response from heater temp request");
    }
    
    freeaddrinfo(res);
    close(sock);
    return temps;
}

// Task to periodically request and update heater temperatures
void heater_temperature_task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10000);  // Update every 10 seconds
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Request temperatures from heater
        heater_temps_t temps = request_heater_temperatures();
        
        // Update UI with LVGL lock
        if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
            // Get the temperature labels and arcs from Screen2
            extern lv_obj_t * ui_tempair;
            extern lv_obj_t * ui_tempcar;
            extern lv_obj_t * ui_air;
            extern lv_obj_t * ui_car;
            
            if (ui_tempair != NULL) {
                char temp_str[16];
                snprintf(temp_str, sizeof(temp_str), "%.1f°C", temps.air_temp);
                lv_label_set_text(ui_tempair, temp_str);
            }
            
            if (ui_tempcar != NULL) {
                char temp_str[16];
                snprintf(temp_str, sizeof(temp_str), "%.1f°C", temps.car_temp);
                lv_label_set_text(ui_tempcar, temp_str);
            }
            
            // Update arc widgets with proper temperature ranges
            if (ui_air != NULL) {
                // Air temperature: -30°C to 230°C (range of 260 degrees)
                int arc_value = (int)((temps.air_temp + 30) * 100 / 260);
                arc_value = arc_value < 0 ? 0 : (arc_value > 100 ? 100 : arc_value);
                lv_arc_set_value(ui_air, arc_value);
            }
            
            if (ui_car != NULL) {
                // Car temperature: -30°C to 50°C (range of 80 degrees)
                int arc_value = (int)((temps.car_temp + 30) * 100 / 80);
                arc_value = arc_value < 0 ? 0 : (arc_value > 100 ? 100 : arc_value);
                lv_arc_set_value(ui_car, arc_value);
            }
            
            lvgl_port_unlock();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

void wifi_init_sta_with_ntp(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    // Keep default station netif configuration: DHCP assigns IP/gateway/DNS.
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
        
        // Initialize mDNS for .local hostname resolution
        ESP_ERROR_CHECK(mdns_init());
        ESP_ERROR_CHECK(mdns_hostname_set("esp32"));
        
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
    char rx_buffer[UDP_RX_BUFFER_SIZE];
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
            if (len >= (int)sizeof(rx_buffer) - 1) {
                ESP_LOGW("UDP", "Dropped oversized UDP packet (%d bytes), buffer=%d", len, (int)sizeof(rx_buffer));
                continue;
            }

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
    configure_app_logging();
    configure_time_zone();

    esp_err_t rtc_err = rtc_pcf85063a_init(CONFIG_EXAMPLE_RTC_I2C_SDA,
                                           CONFIG_EXAMPLE_RTC_I2C_SCL,
                                           CONFIG_EXAMPLE_RTC_I2C_FREQ_HZ);
    if (rtc_err == ESP_OK) {
        rtc_available = true;
    } else {
        ESP_LOGW("TIME", "RTC init failed (%s), falling back to build/NTP time", esp_err_to_name(rtc_err));
    }

    if (!initialize_time_from_rtc_if_available()) {
        initialize_time_from_build_if_needed();
    }

    // Create queue for latest UI sample only (overwrite semantics for low latency)
    sensor_data_queue = xQueueCreate(1, sizeof(sensor_data_t));
    
    if (sensor_data_queue == NULL) {
        ESP_LOGE("MAIN", "Failed to create sensor data queue");
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
    if (xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create udp_server task");
    }
    
    // Start UI update task
    if (xTaskCreate(ui_update_task_with_consumption, "ui_update", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create ui_update task");
    }
    
    // Start clock update task
    if (xTaskCreate(clock_update_task, "clock_update", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create clock_update task");
    }
    
    // Start UDS data monitor task
    if (xTaskCreate(uds_monitor_task, "uds_monitor", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create uds_monitor task");
    }
    
    // Start heater temperature polling task
    if (xTaskCreate(heater_temperature_task, "heater_temps", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create heater_temps task");
    }
    
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

        if (ui_battw != NULL) {
            // Center-zero behavior: positive and negative kW move in opposite directions.
            lv_arc_set_range(ui_battw, -60 * BATTW_ARC_SCALE, 60 * BATTW_ARC_SCALE);
            lv_arc_set_mode(ui_battw, LV_ARC_MODE_SYMMETRICAL);
            lv_arc_set_value(ui_battw, 0);
        }

        battw_arc_last = 0;
        battw_label_last_tenths = 0;
        
        update_time_strings();
        refresh_clock_labels_locked();
        
        lvgl_port_unlock();
    }
}