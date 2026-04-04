#ifndef RTC_PCF85063A_H
#define RTC_PCF85063A_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t dotw;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} rtc_datetime_t;

esp_err_t rtc_pcf85063a_init(int sda_gpio, int scl_gpio, uint32_t freq_hz);
esp_err_t rtc_pcf85063a_read(rtc_datetime_t *out_time);
esp_err_t rtc_pcf85063a_write(const rtc_datetime_t *in_time);

#endif
