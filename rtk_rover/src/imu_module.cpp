#include "imu_module.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char *TAG = "IMU";

static float gyro_offset_z = 0.0;
static float current_heading = 0.0;
static uint32_t last_update_time = 0;

esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t* data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_regs(uint8_t start_reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t imu_calibrate(void) {
    ESP_LOGI(TAG, "Calibrating gyroscope... Keep rover stationary!");
    
    float sum_gyro_z = 0;
    int samples = 1000;
    
    for (int i = 0; i < samples; i++) {
        uint8_t gyro_data[6];
        if (mpu6050_read_regs(MPU6050_GYRO_XOUT_H, gyro_data, 6) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro data during calibration");
            return ESP_FAIL;
        }
        
        int16_t gyro_z_raw = (gyro_data[4] << 8) | gyro_data[5];
        float gyro_z = gyro_z_raw / 131.0; // Convert to degrees/sec
        sum_gyro_z += gyro_z;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    gyro_offset_z = sum_gyro_z / samples;
    ESP_LOGI(TAG, "Gyroscope calibration complete. Z-axis offset: %.3f deg/s", gyro_offset_z);
    
    return ESP_OK;
}

esp_err_t imu_read_data(imu_data_t* data) {
    uint8_t accel_data[6];
    uint8_t gyro_data[6];
    
    // Read accelerometer data
    if (mpu6050_read_regs(MPU6050_ACCEL_XOUT_H, accel_data, 6) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Read gyroscope data
    if (mpu6050_read_regs(MPU6050_GYRO_XOUT_H, gyro_data, 6) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Convert accelerometer data
    int16_t accel_x_raw = (accel_data[0] << 8) | accel_data[1];
    int16_t accel_y_raw = (accel_data[2] << 8) | accel_data[3];
    int16_t accel_z_raw = (accel_data[4] << 8) | accel_data[5];
    
    data->accel_x = accel_x_raw / 16384.0; // Convert to g
    data->accel_y = accel_y_raw / 16384.0;
    data->accel_z = accel_z_raw / 16384.0;
    
    // Convert gyroscope data
    int16_t gyro_x_raw = (gyro_data[0] << 8) | gyro_data[1];
    int16_t gyro_y_raw = (gyro_data[2] << 8) | gyro_data[3];
    int16_t gyro_z_raw = (gyro_data[4] << 8) | gyro_data[5];
    
    data->gyro_x = gyro_x_raw / 131.0; // Convert to degrees/sec
    data->gyro_y = gyro_y_raw / 131.0;
    data->gyro_z = (gyro_z_raw / 131.0) - gyro_offset_z; // Apply calibration
    
    data->timestamp = xTaskGetTickCount();
    
    return ESP_OK;
}

void imu_task(void *pvParameters) {
    imu_data_t imu_data;
    uint32_t current_time;
    float dt;
    
    last_update_time = xTaskGetTickCount();
    
    while (1) {
        if (imu_read_data(&imu_data) == ESP_OK) {
            current_time = xTaskGetTickCount();
            dt = (current_time - last_update_time) * portTICK_PERIOD_MS / 1000.0; // Convert to seconds
            
            // Integrate gyroscope Z-axis to get heading
            current_heading += imu_data.gyro_z * dt;
            
            // Keep heading in 0-360 range
            while (current_heading < 0) current_heading += 360.0;
            while (current_heading >= 360.0) current_heading -= 360.0;
            
            imu_data.heading = current_heading;
            last_update_time = current_time;
            
            ESP_LOGD(TAG, "Heading: %.1f°, Gyro Z: %.2f°/s", current_heading, imu_data.gyro_z);
        } else {
            ESP_LOGW(TAG, "Failed to read IMU data");
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz update rate
    }
}

float get_current_heading(void) {
    return current_heading;
}

esp_err_t imu_init(void) {
    // Configure I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_SDA_PIN,
        .scl_io_num = IMU_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = IMU_I2C_FREQ_HZ
        }
    };
    
    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    
    // Initialize MPU6050
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for MPU6050 to boot
    
    // Wake up MPU6050
    if (mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Configure gyroscope (±250°/s)
    if (mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ESP_FAIL;
    }
    
    // Configure accelerometer (±2g)
    if (mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Calibrate gyroscope
    if (imu_calibrate() != ESP_OK) {
        ESP_LOGE(TAG, "IMU calibration failed");
        return ESP_FAIL;
    }
    
    // Create IMU task
    xTaskCreate(imu_task, "imu_task", 3072, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "IMU module initialized");
    return ESP_OK;
}