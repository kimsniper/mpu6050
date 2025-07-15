/*
 * Copyright (c) 2025, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "mpu6050.hpp"

using namespace std::chrono_literals;

static MPU6050::Device Dev = {
    .i2cPort = 0,
    .i2cAddress = MPU6050::I2C_ADDRESS_MPU6050_AD0_L
};

void mpu6050_thread()
{
    // Initialize HAL layer first
    if (mpu6050_hal_init(Dev.i2cPort) == Mpu6050_Error_t::MPU6050_ERR) {
        std::cerr << "Failed to initialize I2C HAL" << std::endl;
        return;
    }

    // Create driver instance
    MPU6050::MPU6050_Driver mpu(Dev);
    
    // Initialize sensor
    auto err = mpu.Mpu6050_Init(&MPU6050::DefaultConfig);
    if (err != Mpu6050_Error_t::MPU6050_OK) {
        std::cerr << "MPU6050 initialization failed!"<< std::endl;
        return;
    }

    // Verify device ID
    uint8_t dev_id = 0;
    err = mpu.Mpu6050_GetDevideId(dev_id);
    if (err != Mpu6050_Error_t::MPU6050_OK || dev_id != MPU6050::WHO_AM_I_VAL) {
        std::cerr << "Invalid MPU6050 device ID: 0x" 
                  << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;
        return;
    }

    std::cout << "MPU6050 initialized successfully. Device ID: 0x"
              << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;

    // Main sensor reading loop
    while (true) {
        MPU6050::Mpu6050_AccelData_t accelData;
        MPU6050::Mpu6050_GyroData_t gyroData;

        // Read accelerometer
        err = mpu.Mpu6050_GetAccelData(accelData);
        if (err == Mpu6050_Error_t::MPU6050_OK) {
            std::cout << "Accelerometer - X: " << accelData.Accel_X << " m/s² | "
                      << "Y: " << accelData.Accel_Y << " m/s² | "
                      << "Z: " << accelData.Accel_Z << " m/s²" << std::endl;
        } else {
            std::cerr << "Accelerometer read error: " << static_cast<int>(err) << std::endl;
        }

        // Read gyroscope
        err = mpu.Mpu6050_GetGyroData(gyroData);
        if (err == Mpu6050_Error_t::MPU6050_OK) {
            std::cout << "Gyroscope - X: " << gyroData.Gyro_X << " °/s | "
                      << "Y: " << gyroData.Gyro_Y << " °/s | "
                      << "Z: " << gyroData.Gyro_Z << " °/s" << std::endl;
        } else {
            std::cerr << "Gyroscope read error: " << static_cast<int>(err) << std::endl;
        }

        std::cout << "----------------------------------------" << std::endl;
        std::this_thread::sleep_for(500ms);
    }
}

extern "C" void app_main(void)
{
    // Configure ESP32 thread attributes
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 4096;
    cfg.prio = 5;
    cfg.pin_to_core = tskNO_AFFINITY;
    cfg.thread_name = "mpu6050_thread";

    if (esp_pthread_set_cfg(&cfg) != ESP_OK) {
        ESP_LOGE("Main", "Failed to configure thread attributes");
        return;
    }

    // Start sensor thread
    std::thread sensor_thread(mpu6050_thread);
    
    // Set thread name for debugging
    // pthread_setname_np(sensor_thread.native_handle(), "mpu6050_thread");
    
    // Detach thread (FreeRTOS will manage it)
    sensor_thread.detach();

    // Main thread can continue with other tasks
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
