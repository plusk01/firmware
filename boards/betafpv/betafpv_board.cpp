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

#include "betafpv_board.h"

namespace rosflight_firmware {

void BetaFPVBoard::init_board()
{
  board_init();

  led_.init(GPIOB, GPIO_Pin_8);

  spi1_.init(&spi_config[CFG_SPI1]);
  cs_.init(GPIOB, GPIO_Pin_9, GPIO::OUTPUT);
}

void BetaFPVBoard::board_reset(bool bootloader)
{
  // systemReset(bootloader);
  // ()bootloader;
  //   NVIC_SystemReset();
}

// clock
uint32_t BetaFPVBoard::clock_millis() { return millis(); }
uint64_t BetaFPVBoard::clock_micros() { return micros(); }
void BetaFPVBoard::clock_delay(uint32_t milliseconds) { delay(milliseconds); }

// serial
void BetaFPVBoard::serial_init(uint32_t baud_rate, uint32_t dev) { vcp_.init(); }
void BetaFPVBoard::serial_write(const uint8_t *src, size_t len) { vcp_.write(src, len); }
uint16_t BetaFPVBoard::serial_bytes_available() { return vcp_.rx_bytes_waiting(); }
uint8_t BetaFPVBoard::serial_read() { return vcp_.read_byte(); }
void BetaFPVBoard::serial_flush() { /*vcp_.flush();*/ }

// sensors
void BetaFPVBoard::sensors_init()
{
  while(millis() < 50); // wait for sensors to boot up
  imu_.init(&spi1_, &cs_);
}

uint16_t BetaFPVBoard::num_sensor_errors()
{
  // return i2cGetErrorCounter();
  return 0;
}

bool BetaFPVBoard::new_imu_data()
{
  return imu_.has_new_data();
}

bool BetaFPVBoard::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
  float read_accel[3], read_gyro[3];
  imu_.read(read_accel, read_gyro, temperature, time_us);

  // transform to body FLU coordinate frame
  accel[0] = -read_accel[0];
  accel[1] =  read_accel[1];
  accel[2] = -read_accel[2];

  gyro[0] = -read_gyro[0];
  gyro[1] =  read_gyro[1];
  gyro[2] = -read_gyro[2];

  return true;
}

void BetaFPVBoard::imu_not_responding_error() { sensors_init(); }

// unused sensors
bool BetaFPVBoard::mag_present() { return false; }
void BetaFPVBoard::mag_update() {}
void BetaFPVBoard::mag_read(float mag[3]) { mag[0] = mag[1] = mag[2] = 0; }
bool BetaFPVBoard::baro_present() { return false; }
void BetaFPVBoard::baro_read(float *pressure, float *temperature) { *pressure = *temperature = 0; }
void BetaFPVBoard::baro_update() {}
bool BetaFPVBoard::diff_pressure_present() { return false; }
void BetaFPVBoard::diff_pressure_read(float *diff_pressure, float *temperature) { *diff_pressure = *temperature = 0; }
void BetaFPVBoard::diff_pressure_update() {}
bool BetaFPVBoard::sonar_present() { return false; }
float BetaFPVBoard::sonar_read() { return 0.0f; }
void BetaFPVBoard::sonar_update() {}

// RC
void BetaFPVBoard::rc_init(rc_type_t rc_type)
{
  // TODO: We don't know what to do unless RC is SBUS
  rc_type = RC_TYPE_SBUS;

  switch (rc_type)
  {
  default:
  case RC_TYPE_SBUS:
    uart2_.init(&uart_config[CFG_UART2]);
    rc_sbus_.init(&uart2_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
    break;
  }
}

float BetaFPVBoard::rc_read(uint8_t channel)
{
  return rc_->read(channel);
}

bool BetaFPVBoard::rc_lost()
{
  return rc_->lost();
}

// PWM
void BetaFPVBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (uint8_t i=0; i<NUM_PWMS; ++i)
  {
      motors_[i].init(&pwm_config[i], 16000, 1000, 2000);
      motors_[i].write_us(1000);
  }
}

void BetaFPVBoard::pwm_disable()
{
  for (uint8_t i=0; i<NUM_PWMS; ++i)
  {
    motors_[i].write(0.0f);
    // motors_[i].disable();
  }
}

void BetaFPVBoard::pwm_write(uint8_t channel, float value)
{
  uint8_t ch = channel;

  // Instead of defining a new mixer or crossing wires,
  // we will just hardcode the proper motor number here.
  switch (channel)
  {
    case 0:
      ch = 1;
      break;
    case 1:
      ch = 0;
      break;
    case 2:
      ch = 2;
      break;
    case 3:
      ch = 3;
      break;
    default:
      ch = channel;
  }

  motors_[ch].write(value);
}

// non-volatile memory
void BetaFPVBoard::memory_init() {}

bool BetaFPVBoard::memory_read(void *data, size_t len)
{
  if (!eeprom_initialized_) {
    eeprom_.init(len);
    eeprom_initialized_ = true;
  }

  return eeprom_.read(data, len);
}

bool BetaFPVBoard::memory_write(const void *data, size_t len)
{
  if (!eeprom_initialized_) {
    eeprom_.init(len);
    eeprom_initialized_ = true;
  }

  return eeprom_.write(data, len);
}

// LED 0 is RC override status
void BetaFPVBoard::led0_on() {}
void BetaFPVBoard::led0_off() {}
void BetaFPVBoard::led0_toggle() {}

// LED 1 is arming status
void BetaFPVBoard::led1_on() { led_.on(); }
void BetaFPVBoard::led1_off() { led_.off(); }
void BetaFPVBoard::led1_toggle() { led_.toggle(); }

// Backup memory
bool BetaFPVBoard::has_backup_data() { return false; }
rosflight_firmware::BackupData BetaFPVBoard::get_backup_data() { return {}; }

} // ns rosflight_firmware
