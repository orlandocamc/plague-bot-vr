#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/usb_serial_jtag.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     400000
#define PCA9685_ADDR    0x40

#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500
#define SERVO_COUNT     8
#define PWM_FREQ_HZ     50

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t pca_dev;
static bool pca_ok = false;
static float current_angles[SERVO_COUNT] = {0};

static void usb_write(const char *str) {
    usb_serial_jtag_write_bytes((const uint8_t *)str, strlen(str), pdMS_TO_TICKS(100));
}

static esp_err_t pca_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(pca_dev, buf, 2, 100);
}

static esp_err_t pca_init(void) {
    esp_err_t err = pca_write_reg(PCA9685_MODE1, 0x00);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    err = pca_write_reg(PCA9685_MODE1, 0x10);
    if (err != ESP_OK) return err;
    uint8_t prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * PWM_FREQ_HZ)) - 1);
    err = pca_write_reg(PCA9685_PRESCALE, prescale);
    if (err != ESP_OK) return err;
    err = pca_write_reg(PCA9685_MODE1, 0x80);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

static esp_err_t pca_set_servo(uint8_t channel, float angle_deg) {
    if (!pca_ok) return ESP_FAIL;
    if (channel >= SERVO_COUNT) return ESP_ERR_INVALID_ARG;
    if (angle_deg < -90.0f) angle_deg = -90.0f;
    if (angle_deg >  90.0f) angle_deg =  90.0f;
    float pulse_us = SERVO_MIN_US + (angle_deg + 90.0f) / 180.0f * (SERVO_MAX_US - SERVO_MIN_US);
    uint16_t ticks = (uint16_t)(pulse_us / 20000.0f * 4096.0f);
    uint8_t reg = PCA9685_LED0_ON_L + (channel * 4);
    uint8_t buf[5] = { reg, 0x00, 0x00, (uint8_t)(ticks & 0xFF), (uint8_t)(ticks >> 8) };
    return i2c_master_transmit(pca_dev, buf, 5, 100);
}

static void process_command(char *cmd) {
    cmd[strcspn(cmd, "\r\n")] = 0;
    if (strlen(cmd) == 0) return;

    char resp[128];

    if (cmd[0] == 'J') {
        int channel; float angle;
        if (sscanf(cmd + 1, "%d,%f", &channel, &angle) == 2) {
            if (channel >= 0 && channel < SERVO_COUNT) {
                if (pca_ok) {
                    esp_err_t err = pca_set_servo((uint8_t)channel, angle);
                    if (err == ESP_OK) {
                        current_angles[channel] = angle;
                        snprintf(resp, sizeof(resp), "OK J%d %.1f\n", channel, angle);
                    } else {
                        snprintf(resp, sizeof(resp), "ERR I2C %d\n", err);
                    }
                } else {
                    // Modo simulacion: aceptar comando pero solo guardar angulo
                    current_angles[channel] = angle;
                    snprintf(resp, sizeof(resp), "SIM J%d %.1f\n", channel, angle);
                }
            } else {
                snprintf(resp, sizeof(resp), "ERR canal invalido\n");
            }
        } else {
            snprintf(resp, sizeof(resp), "ERR formato\n");
        }
        usb_write(resp);

    } else if (strcmp(cmd, "CENTER") == 0) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (pca_ok) pca_set_servo(i, 0.0f);
            current_angles[i] = 0.0f;
        }
        usb_write(pca_ok ? "OK CENTER\n" : "SIM CENTER\n");

    } else if (strcmp(cmd, "STATUS") == 0) {
        char tmp[16];
        usb_write(pca_ok ? "STATUS PCA:OK" : "STATUS PCA:NO");
        for (int i = 0; i < SERVO_COUNT; i++) {
            snprintf(tmp, sizeof(tmp), " J%d:%.1f", i, current_angles[i]);
            usb_write(tmp);
        }
        usb_write("\n");

    } else if (strcmp(cmd, "PING") == 0) {
        usb_write("PONG\n");

    } else {
        snprintf(resp, sizeof(resp), "ERR desconocido: %s\n", cmd);
        usb_write(resp);
    }
}

static void usb_task(void *arg) {
    uint8_t rxbuf[64];
    char linebuf[256];
    int idx = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    usb_write("READY\n");

    while (1) {
        int len = usb_serial_jtag_read_bytes(rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(20));
        for (int i = 0; i < len; i++) {
            char c = (char)rxbuf[i];
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    linebuf[idx] = 0;
                    process_command(linebuf);
                    idx = 0;
                }
            } else if (idx < (int)sizeof(linebuf) - 1) {
                linebuf[idx++] = c;
            }
        }
    }
}

void app_main(void) {
    // Driver USB Serial JTAG (control exclusivo)
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // I2C
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_master_bus_init(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &pca_dev));

    if (pca_init() == ESP_OK) {
        pca_ok = true;
        for (int i = 0; i < SERVO_COUNT; i++) pca_set_servo(i, 0.0f);
    }

    xTaskCreate(usb_task, "usb_task", 4096, NULL, 5, NULL);
}
