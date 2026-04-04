#include "rtc_pcf85063a.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#define RTC_I2C_PORT 0
#define RTC_I2C_ADDR 0x51
#define RTC_I2C_TIMEOUT_MS 1000

#define RTC_REG_CTRL1 0x00
#define RTC_REG_SECOND 0x04

#define RTC_CTRL1_CAP_SEL 0x01

static bool s_rtc_ready = false;

static uint8_t dec_to_bcd(uint8_t val)
{
    return (uint8_t)(((val / 10U) << 4U) | (val % 10U));
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return (uint8_t)(((val >> 4U) * 10U) + (val & 0x0FU));
}

esp_err_t rtc_pcf85063a_init(int sda_gpio, int scl_gpio, uint32_t freq_hz)
{
    if (s_rtc_ready) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t err = i2c_param_config(RTC_I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(RTC_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    uint8_t ctrl_payload[2] = {RTC_REG_CTRL1, RTC_CTRL1_CAP_SEL};
    err = i2c_master_write_to_device(RTC_I2C_PORT, RTC_I2C_ADDR, ctrl_payload, sizeof(ctrl_payload), pdMS_TO_TICKS(RTC_I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    s_rtc_ready = true;
    return ESP_OK;
}

esp_err_t rtc_pcf85063a_read(rtc_datetime_t *out_time)
{
    if (out_time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[7] = {0};
    uint8_t reg = RTC_REG_SECOND;
    esp_err_t err = i2c_master_write_read_device(RTC_I2C_PORT, RTC_I2C_ADDR, &reg, 1, raw, sizeof(raw), pdMS_TO_TICKS(RTC_I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    out_time->sec = bcd_to_dec(raw[0] & 0x7FU);
    out_time->min = bcd_to_dec(raw[1] & 0x7FU);
    out_time->hour = bcd_to_dec(raw[2] & 0x3FU);
    out_time->day = bcd_to_dec(raw[3] & 0x3FU);
    out_time->dotw = bcd_to_dec(raw[4] & 0x07U);
    out_time->month = bcd_to_dec(raw[5] & 0x1FU);
    out_time->year = (uint16_t)(1970 + bcd_to_dec(raw[6]));

    return ESP_OK;
}

esp_err_t rtc_pcf85063a_write(const rtc_datetime_t *in_time)
{
    if (in_time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (in_time->year < 1970 || in_time->year > 2069) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[8] = {
        RTC_REG_SECOND,
        dec_to_bcd(in_time->sec),
        dec_to_bcd(in_time->min),
        dec_to_bcd(in_time->hour),
        dec_to_bcd(in_time->day),
        dec_to_bcd(in_time->dotw),
        dec_to_bcd(in_time->month),
        dec_to_bcd((uint8_t)(in_time->year - 1970)),
    };

    return i2c_master_write_to_device(RTC_I2C_PORT, RTC_I2C_ADDR, payload, sizeof(payload), pdMS_TO_TICKS(RTC_I2C_TIMEOUT_MS));
}
