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

#include "mpu6050_hal.hpp"

Mpu6050_Error_t mpu6050_hal_init(uint8_t pI2cPort) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0,
    };

    i2c_param_config(static_cast<i2c_port_t>(pI2cPort), &conf);

    if (i2c_driver_install(static_cast<i2c_port_t>(pI2cPort), 
                          conf.mode, 
                          0,  // RX buffer disabled
                          0,  // TX buffer disabled
                          0) != ESP_OK) {
        return Mpu6050_Error_t::MPU6050_ERR;
    }

    return Mpu6050_Error_t::MPU6050_OK;
}

Mpu6050_Error_t mpu6050_i2c_hal_read(const uint8_t address, 
                            uint8_t pI2cPort, 
                            uint8_t* reg, 
                            uint8_t* pRxBuffer, 
                            const uint16_t count) {
    if (reg == nullptr || pRxBuffer == nullptr) {
        return Mpu6050_Error_t::MPU6050_ERR;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return Mpu6050_Error_t::MPU6050_ERR;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg, 1, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pRxBuffer, count, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(static_cast<i2c_port_t>(pI2cPort), 
                                     cmd, 
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? Mpu6050_Error_t::MPU6050_OK : Mpu6050_Error_t::MPU6050_ERR;
}

Mpu6050_Error_t mpu6050_i2c_hal_write(const uint8_t address, 
                             uint8_t pI2cPort, 
                             uint8_t* pTxBuffer, 
                             const uint16_t count) {
    if (pTxBuffer == nullptr) {
        return Mpu6050_Error_t::MPU6050_ERR;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return Mpu6050_Error_t::MPU6050_ERR;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pTxBuffer, count, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(static_cast<i2c_port_t>(pI2cPort), 
                                      cmd, 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? Mpu6050_Error_t::MPU6050_OK : Mpu6050_Error_t::MPU6050_ERR;
}

void mpu6050_i2c_hal_ms_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}