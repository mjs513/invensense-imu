/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* MPU-6250 functions ported frome J.Rowbergs MPU-6250 library
* I2Cdev library collection - MPU6050 I2C device class
* Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
* 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
* Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
*
* Changelog:
*  2021/09/27 - split implementations out of header files, finally
*     ... - ongoing debug release
*
* NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
* DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
* YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
*/

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef INVENSENSE_IMU_SRC_MPU6250_H_  // NOLINT
#define INVENSENSE_IMU_SRC_MPU6250_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "invensense_imu.h"  // NOLINT

namespace bfs {

class Mpu6250 {
 public:
  /* Sensor and filter settings */
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,
    I2C_ADDR_SEC = 0x69
  };
  
  enum DlpfBandwidth : int8_t {
    DLPF_BANDWIDTH_260HZ = 0x00,
    DLPF_BANDWIDTH_184HZ = 0x01,
    DLPF_BANDWIDTH_94HZ = 0x02,
    DLPF_BANDWIDTH_44HZ = 0x03,
    DLPF_BANDWIDTH_21HZ = 0x04,
    DLPF_BANDWIDTH_10HZ = 0x05,
    DLPF_BANDWIDTH_5HZ = 0x06
  };
  
  enum AccelRange : uint8_t {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x01,
    ACCEL_RANGE_8G = 0x02,
    ACCEL_RANGE_16G = 0x03
  };
  
  enum GyroRange : uint8_t {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x01,
    GYRO_RANGE_1000DPS = 0x02,
    GYRO_RANGE_2000DPS = 0x03
  };
  
  Mpu6250() {}
  Mpu6250(TwoWire *i2c, const I2cAddr addr) :
          imu_(i2c, static_cast<uint8_t>(addr)) {}
  Mpu6250(SPIClass *spi, const uint8_t cs) :
          imu_(spi, cs) {}
  void Config(TwoWire *i2c, const I2cAddr addr);
  void Config(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool EnableDrdyInt();
  bool DisableDrdyInt();
  bool ConfigAccelRange(const AccelRange range);
  inline AccelRange accel_range() const {return accel_range_;}
  bool ConfigGyroRange(const GyroRange range);
  inline GyroRange gyro_range() const {return gyro_range_;}
  bool ConfigSrd(const uint8_t srd);
  inline uint8_t srd() const {return srd_;}
  bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf);
  inline DlpfBandwidth dlpf_bandwidth() const {return dlpf_bandwidth_;}
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
  inline float accel_x_mps2() const {return accel_[0];}
  inline float accel_y_mps2() const {return accel_[1];}
  inline float accel_z_mps2() const {return accel_[2];}
  inline float gyro_x_radps() const {return gyro_[0];}
  inline float gyro_y_radps() const {return gyro_[1];}
  inline float gyro_z_radps() const {return gyro_[2];}
  inline float die_temp_c() const {return temp_;}
  
  bool Read(float * values);
  void getScales(float *accScale, float *gyroScale, float *magScale);
  bool Read_raw(int16_t * values);
  
 private:
  InvensenseImu imu_;
  int32_t spi_clock_;
  /*
  * MPU-9250 supports an SPI clock of 1 MHz for config and 20 MHz for reading
  * data; however, in testing we found that 20 MHz was sometimes too fast and
  * scaled this down to 15 MHz, which consistently worked well.
  */
  static constexpr int32_t SPI_CFG_CLOCK_ = 1000000;
  static constexpr int32_t SPI_READ_CLOCK_ = 15000000;
  /* Configuration */
  AccelRange accel_range_, requested_accel_range_;
  GyroRange gyro_range_, requested_gyro_range_;
  DlpfBandwidth dlpf_bandwidth_, requested_dlpf_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_, requested_gyro_scale_;
  uint8_t srd_;
  static constexpr float TEMP_SCALE_ = 333.87f;
  uint8_t who_am_i_;
  /* Data */
  static constexpr float G_MPS2_ = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846264338327950288f /
                                    180.0f;
  bool new_imu_data_;
  uint8_t data_buf_[15];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_;
  float accel_[3], gyro_[3];
  float temp_;
  /* Registers */
  static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;
  static constexpr uint8_t INT_RAW_RDY_EN_ = 0x01;
  static constexpr uint8_t INT_DISABLE_ = 0x00;
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;
  
  #define ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
  #define ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
  #define DEFAULT_ADDRESS     ADDRESS_AD0_LOW
  #define DEFAULT_SS_PIN		4

  #define PWR1_DEVICE_RESET_BIT   7
  #define PWR1_SLEEP_BIT          6
  #define PWR1_CYCLE_BIT          5
  #define PWR1_TEMP_DIS_BIT       3
  #define PWR1_CLKSEL_BIT         2
  #define PWR1_CLKSEL_LENGTH      3
  
  #define ACONFIG_XA_ST_BIT           7
  #define ACONFIG_YA_ST_BIT           6
  #define ACONFIG_ZA_ST_BIT           5
  #define ACONFIG_AFS_SEL_BIT         4
  #define ACONFIG_AFS_SEL_LENGTH      2
  #define ACONFIG_ACCEL_HPF_BIT       2
  #define ACONFIG_ACCEL_HPF_LENGTH    3
  
  #define GCONFIG_FS_SEL_BIT      4
  #define GCONFIG_FS_SEL_LENGTH   2

  #define RA_SMPLRT_DIV       0x19
  #define RA_CONFIG           0x1A
  #define RA_GYRO_CONFIG      0x1B
  #define RA_ACCEL_CONFIG     0x1C
  #define MPU9250_RA_ACCEL_CONFIG2    0x1D
  
  #define RA_INT_PIN_CFG      0x37
  #define RA_INT_ENABLE       0x38
  #define RA_INT_STATUS       0x3A
  #define RA_ACCEL_XOUT_H     0x3B
  #define RA_ACCEL_XOUT_L     0x3C
  #define RA_ACCEL_YOUT_H     0x3D
  #define RA_ACCEL_YOUT_L     0x3E
  #define RA_ACCEL_ZOUT_H     0x3F
  #define RA_ACCEL_ZOUT_L     0x40
  #define RA_TEMP_OUT_H       0x41
  #define RA_TEMP_OUT_L       0x42
  #define RA_GYRO_XOUT_H      0x43
  #define RA_GYRO_XOUT_L      0x44
  #define RA_GYRO_YOUT_H      0x45
  #define RA_GYRO_YOUT_L      0x46
  #define RA_GYRO_ZOUT_H      0x47
  #define RA_GYRO_ZOUT_L      0x48
  
  #define RA_WHO_AM_I         0x75

  #define CLOCK_INTERNAL          0x00
  #define CLOCK_PLL_XGYRO         0x01
  #define CLOCK_PLL_YGYRO         0x02
  #define CLOCK_PLL_ZGYRO         0x03
  #define CLOCK_PLL_EXT32K        0x04
  #define CLOCK_PLL_EXT19M        0x05
  #define CLOCK_KEEP_RESET        0x07
  
  /* Utility functions */
  bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
};  //end class

}   // end namespace
#endif