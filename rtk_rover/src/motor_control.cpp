#include "motor_control.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

uint32_t speed_to_pwm(float speed) {
    // Clamp speed to valid range
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;
    
    // Apply deadzone
    if (speed > -0.05 && speed < 0.05) {
        return PWM_NEUTRAL;
    }
    
    // Convert speed to PWM value
    if (speed > 0) {
        return PWM_NEUTRAL + (uint32_t)(speed * (PWM_MAX - PWM_NEUTRAL));
    } else {
        return PWM_NEUTRAL + (uint32_t)(speed * (PWM_NEUTRAL - PWM_MIN));
    }
}

esp_err_t motor_set_speeds(float left_speed, float right_speed) {
    uint32_t left_pwm = speed_to_pwm(left_speed);
    uint32_t right_pwm = speed_to_pwm(right_speed);
    
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL, left_pwm));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL));
    
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL, right_pwm));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL));
    
    ESP_LOGD(TAG, "Motor speeds set - Left: %.2f (PWM: %lu), Right: %.2f (PWM: %lu)", 
             left_speed, left_pwm, right_speed, right_pwm);
    
    return ESP_OK;
}

esp_err_t motor_stop(void) {
    return motor_set_speeds(0.0, 0.0);
}

esp_err_t motor_move_forward(float speed) {
    return motor_set_speeds(speed, speed);
}

esp_err_t motor_move_backward(float speed) {
    return motor_set_speeds(-speed, -speed);
}

esp_err_t motor_turn_left(float speed) {
    return motor_set_speeds(-speed, speed);
}

esp_err_t motor_turn_right(float speed) {
    return motor_set_speeds(speed, -speed);
}

esp_err_t motor_differential_drive(float linear, float angular) {
    // Differential drive calculation
    // linear: forward/backward speed (-1.0 to 1.0)
    // angular: turning speed (-1.0 to 1.0, positive = right turn)
    
    float left_speed = linear - angular;
    float right_speed = linear + angular;
    
    // Normalize if either speed exceeds limits
    float max_speed = fmax(fabs(left_speed), fabs(right_speed));
    if (max_speed > 1.0) {
        left_speed /= max_speed;
        right_speed /= max_speed;
    }
    
    return motor_set_speeds(left_speed, right_speed);
}

esp_err_t motor_control_init(void) {
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = MOTOR_PWM_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    
    // Configure left motor channel
    ledc_channel_config_t left_channel_config = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_LEFT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_LEFT_PIN,
        .duty = PWM_NEUTRAL,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_channel_config));
    
    // Configure right motor channel
    ledc_channel_config_t right_channel_config = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_RIGHT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_RIGHT_PIN,
        .duty = PWM_NEUTRAL,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_channel_config));
    
    // Set both motors to neutral position
    ESP_ERROR_CHECK(motor_stop());
    
    ESP_LOGI(TAG, "Motor control initialized - Left pin: %d, Right pin: %d", 
             MOTOR_LEFT_PIN, MOTOR_RIGHT_PIN);
    
    return ESP_OK;
}