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

#include "discoveryf3.h"

namespace rosflight_firmware {

void DiscoveryF3::init_board()
{
  board_init();

  led_.init(GPIOE, GPIO_Pin_13);

  spi1_.init(&spi_config[CFG_SPI1]);
  cs_.init(GPIOB, GPIO_Pin_9, GPIO::OUTPUT);
}

void DiscoveryF3::board_reset(bool bootloader)
{
  // systemReset(bootloader);
  // ()bootloader;
  //   NVIC_SystemReset();
}

// clock
uint32_t DiscoveryF3::clock_millis() { return millis(); }
uint64_t DiscoveryF3::clock_micros() { return micros(); }
void DiscoveryF3::clock_delay(uint32_t milliseconds) { delay(milliseconds); }

// serial
void DiscoveryF3::serial_init(uint32_t baud_rate, uint32_t dev) { vcp_.init(); }
void DiscoveryF3::serial_write(const uint8_t *src, size_t len) { vcp_.write(src, len); }
uint16_t DiscoveryF3::serial_bytes_available() { return vcp_.rx_bytes_waiting(); }
uint8_t DiscoveryF3::serial_read() { return vcp_.read_byte(); }
void DiscoveryF3::serial_flush() { /*vcp_.flush();*/ }

// sensors
void DiscoveryF3::sensors_init()
{
  while(millis() < 50); // wait for sensors to boot up
  imu_.init(&spi1_, &cs_);
}

uint16_t DiscoveryF3::num_sensor_errors()
{
  // return i2cGetErrorCounter();
  return 0;
}

bool DiscoveryF3::new_imu_data()
{
  return true; //imu_.has_new_data();
}

bool DiscoveryF3::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
  float read_accel[3] = {0}, read_gyro[3] = {0};
  imu_.read(read_accel, read_gyro, temperature, time_us);

  // transform to body FLU coordinate frame
  accel[0] = -0.02f;
  accel[1] =  0.001f;
  accel[2] = -9.80655f;

  gyro[0] = -0.1f;
  gyro[1] =  0.002f;
  gyro[2] = -0.05f;

  return true;
}

void DiscoveryF3::imu_not_responding_error() { /*sensors_init();*/ }

// unused sensors
bool DiscoveryF3::mag_present() { return false; }
void DiscoveryF3::mag_update() {}
void DiscoveryF3::mag_read(float mag[3]) { mag[0] = mag[1] = mag[2] = 0; }
bool DiscoveryF3::baro_present() { return false; }
void DiscoveryF3::baro_read(float *pressure, float *temperature) { *pressure = *temperature = 0; }
void DiscoveryF3::baro_update() {}
bool DiscoveryF3::diff_pressure_present() { return false; }
void DiscoveryF3::diff_pressure_read(float *diff_pressure, float *temperature) { *diff_pressure = *temperature = 0; }
void DiscoveryF3::diff_pressure_update() {}
bool DiscoveryF3::sonar_present() { return false; }
float DiscoveryF3::sonar_read() { return 0.0f; }
void DiscoveryF3::sonar_update() {}

// RC
void DiscoveryF3::rc_init(rc_type_t rc_type)
{
  // TODO: We don't know what to do unless RC is SBUS
  rc_type = RC_TYPE_SBUS;

  switch (rc_type)
  {
  default:
  case RC_TYPE_SBUS:
    uart2_.init(&uart_config[CFG_UART1]);
    rc_sbus_.init(&uart2_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
    break;
  }
}

float DiscoveryF3::rc_read(uint8_t channel)
{
//    if (rc_ != nullptr) {
//        return rc_->read(channel);
//    }

    return 0.0f;
}

bool DiscoveryF3::rc_lost()
{
  return rc_->lost();
}

// PWM
void DiscoveryF3::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  motors_[0].init(&pwm_config[0], 16000, 1000, 2000);
  motors_[1].init(&pwm_config[1], 16000, 1000, 2000);
}

void DiscoveryF3::pwm_disable()
{
  // for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  // {
  //   esc_out_[i].disable();
  // }
}

void DiscoveryF3::pwm_write(uint8_t channel, float value)
{
  // esc_out_[channel].write(value);
}

// non-volatile memory
void DiscoveryF3::memory_init()
{
  // initEEPROM();
}

bool DiscoveryF3::memory_read(void *data, size_t len)
{
  // return readEEPROM(dest, len);
  return false;
}

bool DiscoveryF3::memory_write(const void *data, size_t len)
{
  // return writeEEPROM(src, len);
  return false;
}

// LED 0 is RC override status
void DiscoveryF3::led0_on() {}
void DiscoveryF3::led0_off() {}
void DiscoveryF3::led0_toggle() {}

// LED 1 is arming status
void DiscoveryF3::led1_on() { led_.on(); }
void DiscoveryF3::led1_off() { led_.off(); }
void DiscoveryF3::led1_toggle() { led_.toggle(); }

// Backup memory
bool DiscoveryF3::has_backup_data() { return false; }
rosflight_firmware::BackupData DiscoveryF3::get_backup_data() { return {}; }

} // ns rosflight_firmware

// #pragma GCC diagnostic pop
