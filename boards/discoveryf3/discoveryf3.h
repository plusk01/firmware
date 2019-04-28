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

#include <lib/airdamon_f3/boards/discoveryf3/discoveryf3.h>

/// Gross hack to allow discovery access to extra drivers
#include "pwm.h"
#include "mpu6500.h"
#include "rc_sbus.h"
/////////////////////////////////////////////////////////

#include "board.h"

namespace rosflight_firmware {

///////////////////////////////////////////////////////////////////////////////
//                            PWM Configuration                              //
///////////////////////////////////////////////////////////////////////////////

constexpr int NUM_PWMS = 4;
const airdamon::PWMConfig pwm_config[NUM_PWMS] = {
  // TIMx, channel, GPIOx, pin, pin_source, GPIO_AF, TIMx_IRQn
  {TIM2, TIM_Channel_1, GPIOA, GPIO_Pin_0, GPIO_PinSource0, GPIO_AF_1, TIM2_IRQn},
  {TIM2, TIM_Channel_2, GPIOA, GPIO_Pin_1, GPIO_PinSource1, GPIO_AF_1, TIM2_IRQn},
  {TIM2, TIM_Channel_3, GPIOA, GPIO_Pin_2, GPIO_PinSource2, GPIO_AF_1, TIM2_IRQn},
  {TIM2, TIM_Channel_4, GPIOA, GPIO_Pin_3, GPIO_PinSource3, GPIO_AF_1, TIM2_IRQn},
};

class DiscoveryF3 : public Board
{

private:
    VCP vcp_;
    airdamon::UART uart2_;
    airdamon::SPI spi1_;
    airdamon::sensors::MPU6500 imu_;
    GPIO cs_;
    LED led_;

    airdamon::PWM motors_[2];

    // RC polymorphism
    airdamon::sensors::RC* rc_ = nullptr;
    airdamon::sensors::RC_SBUS rc_sbus_;

public:
  DiscoveryF3() = default;

  // setup
  void init_board() override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // serial
  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t *src, size_t len) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors() override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
  void imu_not_responding_error() override;

  bool mag_present() override;
  void mag_update() override;
  void mag_read(float mag[3]) override;

  bool baro_present() override;
  void baro_update() override;
  void baro_read(float *pressure, float *temperature) override;

  bool diff_pressure_present() override;
  void diff_pressure_update() override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;

  bool sonar_present() override;
  void sonar_update() override;
  float sonar_read() override;

  // RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost() override;
  float rc_read(uint8_t channel) override;

  // PWM
  void pwm_init(uint32_t refresh_rate, uint16_t  idle_pwm) override;
  void pwm_disable() override;
  void pwm_write(uint8_t channel, float value) override;

  // non-volatile memory
  void memory_init() override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void *src, size_t len) override;

  // LEDs
  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;

  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

  //Backup Data
  bool has_backup_data() override;
  rosflight_firmware::BackupData get_backup_data() override;
};

} // ns rosflight_firmware
