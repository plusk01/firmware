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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <betafpv.h>

#include "board.h"

namespace rosflight_firmware {

class BetaFPV : public Board
{

private:
    VCP vcp_;
    airdamon::UART uart2_;
    airdamon::SPI spi1_;
    airdamon::sensors::MPU6500 imu_;
    GPIO cs_;
    LED led_;

    // RC polymorphism
    airdamon::sensors::RC* rc_ = nullptr;
    airdamon::sensors::RC_SBUS rc_sbus_;

public:
  BetaFPV();

  // setup
  void init_board();
  void board_reset(bool bootloader);

  // clock
  uint32_t clock_millis();
  uint64_t clock_micros();
  void clock_delay(uint32_t milliseconds);

  // serial
  void serial_init(uint32_t baud_rate);
  void serial_write(const uint8_t *src, size_t len);
  uint16_t serial_bytes_available();
  uint8_t serial_read();
  void serial_flush();

  // sensors
  void sensors_init();
  uint16_t num_sensor_errors();

  bool new_imu_data();
  bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us);
  void imu_not_responding_error();

  bool mag_check();
  void mag_read(float mag[3]);

  bool baro_check();
  void baro_read(float *pressure, float *temperature);

  bool diff_pressure_check();
  void diff_pressure_read(float *diff_pressure, float *temperature);

  bool sonar_check();
  float sonar_read();

  // RC
  void rc_init(rc_type_t rc_type);
  bool rc_lost();
  float rc_read(uint8_t channel);

  // PWM
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm);
  void pwm_write(uint8_t channel, float value);

  // non-volatile memory
  void memory_init();
  bool memory_read(void *dest, size_t len);
  bool memory_write(void *src, size_t len);

  // LEDs
  void led0_on();
  void led0_off();
  void led0_toggle();

  void led1_on();
  void led1_off();
  void led1_toggle();
};

}
