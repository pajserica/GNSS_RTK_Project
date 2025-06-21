#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_err.h"
#include "driver/ledc.h"

// Motor Control Configuration
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_FREQ_HZ 50  // Standard RC servo frequency
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_16_BIT

// Motor pins
#define MOTOR_LEFT_PIN GPIO_NUM_32
#define MOTOR_RIGHT_PIN GPIO_NUM_33
#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_1

// PWM values for RC-style control
#define PWM_NEUTRAL 4915   // 1.5ms pulse (neutral)
#define PWM_MIN 3277       // 1.0ms pulse (full reverse)
#define PWM_MAX 6553       // 2.0ms pulse (full forward)
#define PWM_DEADZONE 164   // 0.05ms deadzone around neutral

// Motor speed and direction
typedef struct {
    float left_speed;   // -1.0 to 1.0
    float right_speed;  // -1.0 to 1.0
} motor_command_t;

// Function declarations
esp_err_t motor_control_init(void);
esp_err_t motor_set_speeds(float left_speed, float right_speed);
esp_err_t motor_stop(void);
esp_err_t motor_move_forward(float speed);
esp_err_t motor_move_backward(float speed);
esp_err_t motor_turn_left(float speed);
esp_err_t motor_turn_right(float speed);
esp_err_t motor_differential_drive(float linear, float angular);
uint32_t speed_to_pwm(float speed);

#endif // MOTOR_CONTROL_H