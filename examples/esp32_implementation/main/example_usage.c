#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* MPU6050 components */
#include "mpu6050.h"
#include "mpu6050_hal.h"

static const char *TAG = "example_usage";

Mpu6050_Dev_t Dev = {
    (uint8_t)I2C_NUM_0,
    I2C_ADDRESS_MPU5060_AD0_L
};

void app_main(void)
{
    int16_t err = MPU6050_OK;
    uint8_t dev_id = 0;
    Mpu6050_AccelData_t AccelData;
    Mpu6050_GyroData_t GyroData;

    err |= mpu6050_hal_init(Dev.pI2cPort);

    err |= Mpu6050_Init(&Dev, &Mpu6050_DefaultConfig);

    err |= Mpu6050_GetDevideId(&Dev, &dev_id);

    if(err == MPU6050_OK){
        ESP_LOGI(TAG, "Revision ID: 0x%02x", dev_id);
    } 
    else{
        ESP_LOGE(TAG, "Unable to read device ID!");
    }

    if (err == MPU6050_OK && dev_id == WHO_AM_I_VAL)
    {
        ESP_LOGI(TAG, "MPU6050 initialization successful");
        while(1)
        {
            err |= Mpu6050_GetAccelData(&Dev, &AccelData);
            err |= Mpu6050_GetGyroData(&Dev, &GyroData);

            if(err == MPU6050_OK)
            {
                ESP_LOGI(TAG, "Accelerometer:");
                ESP_LOGI(TAG, "Linear Acceleration X = %.02fm/s^2", AccelData.Accel_X);
                ESP_LOGI(TAG, "Linear Acceleration Y = %.02fm/s^2", AccelData.Accel_Y);
                ESP_LOGI(TAG, "Linear Acceleration Z = %.02fm/s^2", AccelData.Accel_Z);
                ESP_LOGI(TAG, "Gyroscope:");
                ESP_LOGI(TAG, "Angular Velocity X = %.02f°/s", GyroData.Gyro_X);
                ESP_LOGI(TAG, "Angular Velocity Y = %.02f°/s", GyroData.Gyro_Y);
                ESP_LOGI(TAG, "Angular Velocity Z = %.02f°/s", GyroData.Gyro_Z);
            }
            else{
                ESP_LOGE(TAG, "Error reading data!");
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    else{
        ESP_LOGE(TAG, "MPU6050 initialization failed!");
    }
}
