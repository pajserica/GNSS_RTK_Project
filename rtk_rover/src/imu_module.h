#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include "esp_err.h"
#include "driver/i2c.h"

// I2C Configuration for IMU
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_SDA_PIN GPIO_NUM_21
#define IMU_SCL_PIN GPIO_NUM_22
#define IMU_I2C_FREQ_HZ 400000

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_ACCEL_XOUT_H 0x3B

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float heading;
    uint32_t timestamp;
} imu_data_t;

// Function declarations
esp_err_t imu_init(void);
esp_err_t imu_read_data(imu_data_t* data);
esp_err_t imu_calibrate(void);
void imu_task(void *pvParameters);
float get_current_heading(void);
esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t* data);
esp_err_t mpu6050_read_regs(uint8_t start_reg, uint8_t* data, size_t len);

#endif // IMU_MODULE_H