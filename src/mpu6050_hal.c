/*
 * Copyright (c) 2024, Mezael Docoy
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

#include "mpu6050_hal.h" 

/* I2C User Defines */

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

/*================================================================================================*/
/**
* @brief        I2c initialization.
* @details      I2c initialization.
*
* @return       int16_t     Return code.
*
*/
int16_t mpu6050_hal_init()
{
    int16_t err = MPU6050_OK;

    /* User implementation here */


    return err;
}

/*================================================================================================*/
/**
* @brief        Execute I2C read.
* @details      Execute I2C read sequence.
*
* @param[in]    address     Device i2c address.
* @param[in]    reg         Register address.
* @param[in]    count       Number of bytes to read.
* @param[out]   pRxBuffer   Pointer to which data will be stored.
*
* @return       int16_t     Return code.
*
*/
int16_t mpu6050_i2c_hal_read(const uint8_t address, void *pI2cPort, uint8_t *reg, uint8_t *pRxBuffer, const uint16_t count)
{
    int16_t err = MPU6050_OK;

    /* User implementation here */


    return err;
}

/*================================================================================================*/
/**
* @brief        Execute I2C write.
* @details      Execute I2C write sequence.
*
* @param[in]    address     Device i2c address.
* @param[in]    count       Number of bytes to read.
* @param[out]   pTxBuffer   Pointer to data that will be written.
*
* @return       int16_t     Return code.
*
*/
int16_t mpu6050_i2c_hal_write(const uint8_t address, void *pI2cPort, uint8_t *pTxBuffer, const uint16_t count)
{
    int16_t err = MPU6050_OK;

    /* User implementation here */


    return err;
}

/*================================================================================================*/
/**
* @brief        Execute ms delay.
* @details      Execute ms delay for hal usage.
*
* @param[in]    ms      Time in milliseconds.
*
* @return       void
*
*/
void mpu6050_i2c_hal_ms_delay(uint32_t ms) {

    /* User implementation here */
    
    
}
