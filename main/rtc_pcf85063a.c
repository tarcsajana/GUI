#include "rtc_pcf85063a.h"

#include "driver/i2c_master.h"

#define RTC_I2C_PORT 1
#define RTC_I2C_ADDR 0x51
#define RTC_I2C_TIMEOUT_MS 1000

#define RTC_REG_CTRL1 0x00
#define RTC_REG_SECOND 0x04

#define RTC_CTRL1_CAP_SEL 0x01

static bool s_rtc_ready = false;
static i2c_master_bus_handle_t s_rtc_i2c_bus = NULL;
static i2c_master_dev_handle_t s_rtc_i2c_dev = NULL;

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

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = RTC_I2C_PORT,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (s_rtc_i2c_bus == NULL) {
        esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_rtc_i2c_bus);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (s_rtc_i2c_dev == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = RTC_I2C_ADDR,
            .scl_speed_hz = freq_hz,
        };

        esp_err_t err = i2c_master_bus_add_device(s_rtc_i2c_bus, &dev_cfg, &s_rtc_i2c_dev);
        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t ctrl_payload[2] = {RTC_REG_CTRL1, RTC_CTRL1_CAP_SEL};
    esp_err_t err = i2c_master_transmit(s_rtc_i2c_dev, ctrl_payload, sizeof(ctrl_payload), RTC_I2C_TIMEOUT_MS);
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
    if (s_rtc_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[7] = {0};
    uint8_t reg = RTC_REG_SECOND;
    esp_err_t err = i2c_master_transmit_receive(s_rtc_i2c_dev, &reg, 1, raw, sizeof(raw), RTC_I2C_TIMEOUT_MS);
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
    if (s_rtc_i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
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

    return i2c_master_transmit(s_rtc_i2c_dev, payload, sizeof(payload), RTC_I2C_TIMEOUT_MS);
}
