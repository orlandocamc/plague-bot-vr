/**
 * Plague-Bot VR - Base Motor Controller
 * 
 * micro-ROS firmware for ESP32-C6
 * Subscribes to /cmd_vel (geometry_msgs/msg/Twist)
 * Controls 4 IBT-2 motor drivers in skid-steer configuration
 * Communicates with RPi5 via USB Serial/JTAG (custom transport)
 * 
 * Motor layout (viewed from behind):
 *   Left:  M1 (front), M3 (rear)
 *   Right: M2 (front), M4 (rear)
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "soc/ledc_periph.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

// =====================================================================
// Pin mapping - matches existing IBT-2 wiring
// =====================================================================
// Left side
#define M1_RPWM_PIN  0    // Motor 1 forward
#define M1_LPWM_PIN  1    // Motor 1 reverse
#define M3_RPWM_PIN  6    // Motor 3 forward
#define M3_LPWM_PIN  7    // Motor 3 reverse
// Right side
#define M2_RPWM_PIN  4    // Motor 2 forward
#define M2_LPWM_PIN  5    // Motor 2 reverse
#define M4_RPWM_PIN  2    // Motor 4 forward
#define M4_LPWM_PIN  23   // Motor 4 reverse

// =====================================================================
// LEDC PWM configuration
// =====================================================================
// We use 4 LEDC channels. M3 mirrors M1 and M4 mirrors M2 via GPIO matrix.
#define LEDC_FREQ_HZ       5000
#define LEDC_RESOLUTION     LEDC_TIMER_8_BIT   // 0-255
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0

#define CH_LEFT_FWD     LEDC_CHANNEL_0   // M1_RPWM -> mirrored to M3_RPWM
#define CH_LEFT_REV     LEDC_CHANNEL_1   // M1_LPWM -> mirrored to M3_LPWM
#define CH_RIGHT_FWD    LEDC_CHANNEL_2   // M2_RPWM -> mirrored to M4_RPWM
#define CH_RIGHT_REV    LEDC_CHANNEL_3   // M2_LPWM -> mirrored to M4_LPWM

// =====================================================================
// Robot parameters
// =====================================================================
#define MAX_PWM             255
#define MAX_LINEAR_SPEED    0.5f    // m/s at PWM 255 (tune to your robot)
#define MAX_ANGULAR_SPEED   2.0f    // rad/s at PWM 255 (tune to your robot)

// =====================================================================
// Safety
// =====================================================================
#define CMD_VEL_TIMEOUT_MS  500     // Stop motors if no message for this long

// =====================================================================
// Logging tag
// =====================================================================
static const char *TAG = "plaguebot_base";

// =====================================================================
// Global state
// =====================================================================
static esp_timer_handle_t watchdog_timer = NULL;
static volatile bool cmd_vel_received = false;

// =====================================================================
// Motor control
// =====================================================================
static void motors_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Configure 4 LEDC channels (primary GPIOs)
    ledc_channel_config_t channels[] = {
        { .channel = CH_LEFT_FWD,  .gpio_num = M1_RPWM_PIN, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = CH_LEFT_REV,  .gpio_num = M1_LPWM_PIN, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = CH_RIGHT_FWD, .gpio_num = M2_RPWM_PIN, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
        { .channel = CH_RIGHT_REV, .gpio_num = M2_LPWM_PIN, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0 },
    };

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }

    // Mirror left channels to M3 via GPIO matrix
    gpio_set_direction(M3_RPWM_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(M3_LPWM_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        M3_RPWM_PIN,
        ledc_periph_signal[LEDC_MODE].sig_out0_idx + CH_LEFT_FWD,
        false, false);
    esp_rom_gpio_connect_out_signal(
        M3_LPWM_PIN,
        ledc_periph_signal[LEDC_MODE].sig_out0_idx + CH_LEFT_REV,
        false, false);

    // Mirror right channels to M4 via GPIO matrix
    gpio_set_direction(M4_RPWM_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(M4_LPWM_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        M4_RPWM_PIN,
        ledc_periph_signal[LEDC_MODE].sig_out0_idx + CH_RIGHT_FWD,
        false, false);
    esp_rom_gpio_connect_out_signal(
        M4_LPWM_PIN,
        ledc_periph_signal[LEDC_MODE].sig_out0_idx + CH_RIGHT_REV,
        false, false);

    ESP_LOGI(TAG, "Motors initialized: 4 LEDC channels, 8 GPIOs via matrix");
}

static void motors_set(int left_pwm, int right_pwm)
{
    // Clamp to valid range
    if (left_pwm > MAX_PWM)  left_pwm = MAX_PWM;
    if (left_pwm < -MAX_PWM) left_pwm = -MAX_PWM;
    if (right_pwm > MAX_PWM)  right_pwm = MAX_PWM;
    if (right_pwm < -MAX_PWM) right_pwm = -MAX_PWM;

    // Left side
    if (left_pwm >= 0) {
        ledc_set_duty(LEDC_MODE, CH_LEFT_FWD, left_pwm);
        ledc_set_duty(LEDC_MODE, CH_LEFT_REV, 0);
    } else {
        ledc_set_duty(LEDC_MODE, CH_LEFT_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_LEFT_REV, -left_pwm);
    }

    // Right side
    if (right_pwm >= 0) {
        ledc_set_duty(LEDC_MODE, CH_RIGHT_FWD, right_pwm);
        ledc_set_duty(LEDC_MODE, CH_RIGHT_REV, 0);
    } else {
        ledc_set_duty(LEDC_MODE, CH_RIGHT_FWD, 0);
        ledc_set_duty(LEDC_MODE, CH_RIGHT_REV, -right_pwm);
    }

    // Apply all changes
    ledc_update_duty(LEDC_MODE, CH_LEFT_FWD);
    ledc_update_duty(LEDC_MODE, CH_LEFT_REV);
    ledc_update_duty(LEDC_MODE, CH_RIGHT_FWD);
    ledc_update_duty(LEDC_MODE, CH_RIGHT_REV);
}

static void motors_stop(void)
{
    motors_set(0, 0);
}

// =====================================================================
// Skid-steer kinematics: Twist -> left/right PWM
// =====================================================================
static void twist_to_pwm(float linear_x, float angular_z, int *left_pwm, int *right_pwm)
{
    // Normalize velocities to -1.0 ... 1.0
    float linear_norm  = linear_x / MAX_LINEAR_SPEED;
    float angular_norm = angular_z / MAX_ANGULAR_SPEED;

    // Clamp normalized values
    if (linear_norm > 1.0f)  linear_norm = 1.0f;
    if (linear_norm < -1.0f) linear_norm = -1.0f;
    if (angular_norm > 1.0f)  angular_norm = 1.0f;
    if (angular_norm < -1.0f) angular_norm = -1.0f;

    // Differential drive: left = linear - angular, right = linear + angular
    float left_f  = linear_norm - angular_norm;
    float right_f = linear_norm + angular_norm;

    // Clamp and scale to PWM
    if (left_f > 1.0f)  left_f = 1.0f;
    if (left_f < -1.0f) left_f = -1.0f;
    if (right_f > 1.0f)  right_f = 1.0f;
    if (right_f < -1.0f) right_f = -1.0f;

    *left_pwm  = (int)(left_f * MAX_PWM);
    *right_pwm = (int)(right_f * MAX_PWM);
}

// =====================================================================
// Watchdog: stop motors if /cmd_vel stops arriving
// =====================================================================
static void watchdog_callback(void *arg)
{
    if (!cmd_vel_received) {
        motors_stop();
    }
    cmd_vel_received = false;
}

static void watchdog_init(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = watchdog_callback,
        .name = "cmd_vel_watchdog",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &watchdog_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(watchdog_timer, CMD_VEL_TIMEOUT_MS * 1000));
    ESP_LOGI(TAG, "Watchdog started: %d ms timeout", CMD_VEL_TIMEOUT_MS);
}

// =====================================================================
// micro-ROS custom transport over USB Serial/JTAG
// =====================================================================
#define USB_BUF_SIZE 512

static bool transport_open(struct uxrCustomTransport *transport)
{
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = USB_BUF_SIZE,
        .rx_buffer_size = USB_BUF_SIZE,
    };
    esp_err_t ret = usb_serial_jtag_driver_install(&config);
    return (ret == ESP_OK);
}

static bool transport_close(struct uxrCustomTransport *transport)
{
    usb_serial_jtag_driver_uninstall();
    return true;
}

static size_t transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err)
{
    int tx = usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
    if (tx < 0) {
        *err = 1;
        return 0;
    }
    return (size_t)tx;
}

static size_t transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    int rx = usb_serial_jtag_read_bytes(buf, len, pdMS_TO_TICKS(timeout));
    if (rx < 0) {
        *err = 1;
        return 0;
    }
    return (size_t)rx;
}

// =====================================================================
// micro-ROS /cmd_vel callback
// =====================================================================
static geometry_msgs__msg__Twist cmd_vel_msg;

static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    int left_pwm = 0;
    int right_pwm = 0;
    twist_to_pwm((float)msg->linear.x, (float)msg->angular.z, &left_pwm, &right_pwm);

    motors_set(left_pwm, right_pwm);
    cmd_vel_received = true;
}

// =====================================================================
// Error handling macro
// =====================================================================
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Failed: %s (line %d, rc=%d)", #fn, __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

// =====================================================================
// Main micro-ROS task
// =====================================================================
static void micro_ros_task(void *arg)
{
    // Set custom transport
    rmw_uros_set_custom_transport(
        true,  // framing = true for serial
        NULL,
        transport_open,
        transport_close,
        transport_write,
        transport_read
    );

    // Wait for agent to be available
    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        ESP_LOGI(TAG, "Agent not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent connected!");

    // Initialize micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "plaguebot_base", "", &support));

    // Create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA
    ));

    ESP_LOGI(TAG, "Subscribed to /cmd_vel - ready to drive!");

    // Start watchdog
    watchdog_init();

    // Spin
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup (unreachable in normal operation)
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    motors_stop();
    vTaskDelete(NULL);
}

// =====================================================================
// Entry point
// =====================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Plague-Bot VR Base Controller starting...");

    // Initialize motors (stopped)
    motors_init();
    motors_stop();

    // Launch micro-ROS task with enough stack
    xTaskCreate(micro_ros_task, "micro_ros_task", 16384, NULL, 5, NULL);
}
