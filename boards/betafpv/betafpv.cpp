/*
 * Copyright (c) 2018, Parker Lusk, BYU MAGICC Lab
 *
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wold-style-cast"

#include "betafpv.h"

namespace rosflight_firmware {

BetaFPV::BetaFPV(){}

void BetaFPV::init_board()
{
  board_init();
}

void BetaFPV::board_reset(bool bootloader)
{
  // systemReset(bootloader);
}

// clock

uint32_t BetaFPV::clock_millis()
{
  return millis();
}

uint64_t BetaFPV::clock_micros()
{
  return micros();
}

void BetaFPV::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial

void BetaFPV::serial_init(uint32_t baud_rate)
{
  // Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void BetaFPV::serial_write(const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    // serialWrite(Serial1, src[i]);
  }
}

uint16_t BetaFPV::serial_bytes_available(void)
{
  // return serialTotalBytesWaiting(Serial1);
  return 0;
}

uint8_t BetaFPV::serial_read(void)
{
  // return serialRead(Serial1);
  return 'A';
}

// sensors

void BetaFPV::sensors_init()
{
  // // Initialize I2c
  // i2cInit(I2CDEV_2);

  // while(millis() < 50);

  // i2cWrite(0,0,0);
  // if (bmp280_init())
  //   baro_type = BARO_BMP280;
  // else if (ms5611_init())
  //   baro_type = BARO_MS5611;

  // hmc5883lInit(_board_revision);
  // mb1242_init();
  // ms4525_init();


  // // IMU
  // uint16_t acc1G;
  // mpu6050_init(true, &acc1G, &_gyro_scale, _board_revision);
  // _accel_scale = 9.80665f/acc1G;
}

uint16_t BetaFPV::num_sensor_errors(void)
{
  // return i2cGetErrorCounter();
  return 0;
}

bool BetaFPV::new_imu_data()
{
  // return mpu6050_new_data();
  return false;
}

bool BetaFPV::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  // volatile int16_t gyro_raw[3], accel_raw[3];
  // volatile int16_t raw_temp;
  // mpu6050_async_read_all(accel_raw, &raw_temp, gyro_raw, time_us);

  // accel[0] = accel_raw[0] * _accel_scale;
  // accel[1] = -accel_raw[1] * _accel_scale;
  // accel[2] = -accel_raw[2] * _accel_scale;

  // gyro[0] = gyro_raw[0] * _gyro_scale;
  // gyro[1] = -gyro_raw[1] * _gyro_scale;
  // gyro[2] = -gyro_raw[2] * _gyro_scale;

  // (*temperature) = (float)raw_temp/340.0f + 36.53f;

  // if (accel[0] == 0 && accel[1] == 0 && accel[2] == 0)
  // {
  //   return false;
  // }
  // else return true;

  return false;
}

void BetaFPV::imu_not_responding_error(void)
{
  // // If the IMU is not responding, then we need to change where we look for the interrupt
  // _board_revision = (_board_revision < 4) ? 5 : 2;
  // sensors_init();
}

void BetaFPV::mag_read(float mag[3])
{
  // // Convert to NED
  // int16_t raw_mag[3];
  // //  hmc5883l_update();
  // hmc5883l_request_async_update();
  // hmc5883l_async_read(raw_mag);
  mag[0] = 0; //(float)raw_mag[0];
  mag[1] = 0; //(float)raw_mag[1];
  mag[2] = 0; //(float)raw_mag[2];
}

bool BetaFPV::mag_check(void)
{
  return false;
}

void BetaFPV::baro_read(float *pressure, float *temperature)
{
  // if (baro_type == BARO_BMP280)
  // {
  //   bmp280_async_update();
  //   bmp280_async_read(pressure, temperature);
  // }
  // else if (baro_type == BARO_MS5611)
  // {
  //   ms5611_async_update();
  //   ms5611_async_read(pressure, temperature);
  // }
}

bool BetaFPV::baro_check()
{
  return false;
}

bool BetaFPV::diff_pressure_check(void)
{
  return false;
}

void BetaFPV::diff_pressure_read(float *diff_pressure, float *temperature)
{
  // ms4525_async_update();
  // ms4525_async_read(diff_pressure, temperature);
}

bool BetaFPV::sonar_check(void)
{
  return false;
}

float BetaFPV::sonar_read(void)
{
  return 0.0f;
}

uint16_t num_sensor_errors(void)
{
  // return i2cGetErrorCounter();
  return 0;
}

// PWM

void BetaFPV::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  // pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t BetaFPV::pwm_read(uint8_t channel)
{
  // return pwmRead(channel);
  return 0;
}

void BetaFPV::pwm_write(uint8_t channel, uint16_t value)
{
  // pwmWriteMotor(channel, value);
}

bool BetaFPV::pwm_lost()
{
  // return ((millis() - pwmLastUpdate()) > 40);
  return true;
}

// non-volatile memory

void BetaFPV::memory_init(void)
{
  // initEEPROM();
}

bool BetaFPV::memory_read(void * dest, size_t len)
{
  // return readEEPROM(dest, len);
  return false;
}

bool BetaFPV::memory_write(const void * src, size_t len)
{
  // return writeEEPROM(src, len);
  return false;
}

// LED

void BetaFPV::led0_on(void) { /*LED0_ON*/; }
void BetaFPV::led0_off(void) { /*LED0_OFF*/; }
void BetaFPV::led0_toggle(void) { /*LED0_TOGGLE*/; }

void BetaFPV::led1_on(void) { /*LED1_ON*/; }
void BetaFPV::led1_off(void) { /*LED1_OFF*/; }
void BetaFPV::led1_toggle(void) { /*LED1_TOGGLE*/; }

}

// #pragma GCC diagnostic pop
