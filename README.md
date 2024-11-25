[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)


### UPDATES 11/24/2024 by MJS513

@flybrianfly of bolderflight updated this library to include the
   * [Invensense ICM-20948](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
   * [Invensense ICM-20649](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20649/)
   * [Invensense MPU-6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)

@mjs513 added the following:
   * Additional examples for using Madgwick's new Fusion for the MPU-9250, ICM-20948 and MPU-6050 w/HMC5983 Magnetometer and ICM-20649 w/LIS3MDL\
   \
   **ICM-20649** examples folder:\
   **EXTMAG_Fusionv1** shows how you can use 6-point calibration using a
   LIS3MDL or a HMC5983\
   \
   **ICM-20948** examples folder:\
   **ICM20948_Fusionv1** - Fusion example for the ICM20948\
   **ICM20948_Fusionv1_Giga** - Fusion example for the ICM20948 using the Arduino Giga R1 with Dual Serial support
   \
   \
   **MPU-9250** examples folder:\
   **MPU9250_Fusion_GIGA**\
   **MPU9250_Fusionv1**\
   **MPU9250_Fusionv1_MPU9250_Fusionv1_GIGA_dual_serial**\
   
   * Filter implementation for the MPU9250 (Madgwick, Mahoney, Complimentary, DCM, and kalman.
   * Calibration examples for 6-point method and using PJRC MotionCal software\
   In the calibration folder of the examples there are several sketches
   \
   **CalAnalysisInvenseVer4** - provides 6-point calibration for the accelerometer and the magnetometer.\
   \
   **CalAnalysisInvenseVer4_GIGA** - is the 6-point calibration sketch for the giga which removes EEPROM.\
   \
   **CalibrateSensors_ICM20948_MotionCal** and **CalibrateSensors_MPU9250_MotionCal** examples are provided to interface to PJRC's MotionCal software\
   \
   **CalibrateSensors_MPU6050_MotionCal_extmag** - provides support for external sensor (HMC5983 and LIS3MDL) for the MPU-6050 and the ICM-20649
   
   
   List of dependencies:
   * [BolderFlight Units Library](https://github.com/bolderflight/units)
   * [BolderFlight Eigen Library](https://github.com/bolderflight/eigen)
   * [tonton81 Circular Buffer Library](https://github.com/tonton81/Circular_Buffer)
   * [Madgwick Fusion](https://github.com/xioTechnologies/Fusion)
   * PJRC [MotionCal](https://www.pjrc.com/store/prop_shield.html)
   
Please note I have not updated to allow for writing calibration values to the Teensy EEPROM.
   
for a GUI I have been using X-IO Technologies [X-IMU3 GUI](https://x-io.co.uk/x-imu3/) or from there repo: https://github.com/xioTechnologies/x-IMU3-Software

Tested on [Teensy 4.0](https://www.pjrc.com/store/teensy40.html), [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) and [Teensy Micromod](https://www.sparkfun.com/products/16402).  Also tested on the [Arduino GIGA R1](https://store.arduino.cc/products/giga-r1-wifi) (GIGA has no eeprom support).

#### Other Notes:

* An example of using 6-point Calibration and PJRC MotionCal for the BMI270/BMM150 can be found here: ]Using BMI270 Accel/Gyro and BMM150 Magnetomer Sensors with Teensy 4.x](https://forum.pjrc.com/index.php?threads/using-bmm270-accel-gyro-and-bmi150-magnetomer-sensors-with-teensy-4-x.76123/)

* Using the Giga: https://forum.arduino.cc/t/experimenting-wonder-if-giga-can-support-multiple-usb-serial-objects/1323753/6?u=merlin513

Three additional Methods have been added to each sensor:

* void getScales(float *accScale, float *gyroScale, float *magScale)
* bool Read(float *calValues_)
* bool Read_raw(int16_t *rawValues_)

### ICM-20948

The ICM-20948 is a 9-axis IMU.  The library supports both SPI and I2C communications:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range (MPU-9250 Only) |
| --- | --- | ---  |
| +/- 250 deg/s  | +/- 2g  | +/- 4900 uT |
| +/- 500 deg/s  | +/- 4g  | |
| +/- 1000 deg/s | +/- 8g  | |
| +/- 2000 deg/s | +/- 16g | |

The IMU uses a AK09916 for a magnetometer.  A separate library is used in Passthrough mode inorder to maintain high sampling rates.

The same command set that is used for the MPU-9250 is available for the the ICM-209648.  With the following exceptions:

#### Bandwidth

Unlike the MPU-9250 the bandwidth for the accelerometer and gyroscope is specified separately.


**bool ConfigAccelDlpfBandwidth(const AccelDlpfBandwidth dlpf)** Sets the cutoff frequency of the digital low pass filter for the accelerometer. Available bandwidths are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 473 Hz | ACCEL_DLPF_BANDWIDTH_473HZ |
| 246 Hz | ACCEL_DLPF_BANDWIDTH_246HZ |
| 111 Hz | ACCEL_DLPF_BANDWIDTH_111HZ |
| 50 Hz | ACCEL_DLPF_BANDWIDTH_50HZ |
| 23 Hz | ACCEL_DLPF_BANDWIDTH_23HZ |
| 11 Hz | ACCEL_DLPF_BANDWIDTH_11HZ |
| 5 Hz | ACCEL_DLPF_BANDWIDTH_5HZ |

True is returned on succesfully setting the digital low pass filters, otherwise, false is returned. The default bandwidth is 473 Hz.

**bool ConfigGyroDlpfBandwidth(const GyroDlpfBandwidth dlpf)** Sets the cutoff frequency of the digital low pass filter for the gyroscope. Available bandwidths are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 196 Hz | GYRO_DLPF_BANDWIDTH_196HZ |
| 196 Hz | GYRO_DLPF_BANDWIDTH_196HZ |
| 151 Hz | GYRO_DLPF_BANDWIDTH_151HZ |
| 119 Hz | GYRO_DLPF_BANDWIDTH_119HZ |
| 51 Hz | GYRO_DLPF_BANDWIDTH_51HZ |
| 23 Hz | GYRO_DLPF_BANDWIDTH_23HZ |
| 11 Hz | GYRO_DLPF_BANDWIDTH_11HZ |
| 5 Hz | GYRO_DLPF_BANDWIDTH_5HZ |

True is returned on succesfully setting the digital low pass filters, otherwise, false is returned. The default bandwidth is 361 Hz.

**bool ConfigTempDlpfBandwidth(const TempDlpfBandwidth dlpf)** Sets the cutoff frequency of the digital low pass filter for the temperature sensor. Available bandwidths are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 217 Hz | TEMP_DLPF_BANDWIDTH_217HZ |
| 123 Hz | TEMP_DLPF_BANDWIDTH_123HZ |
| 65 Hz | TEMP_DLPF_BANDWIDTH_65HZ |
| 34 Hz | TEMP_DLPF_BANDWIDTH_34HZ |
| 17 Hz | TEMP_DLPF_BANDWIDTH_17HZ |

True is returned on succesfully setting the digital low pass filters, otherwise, false is returned. The default bandwidth is 217 Hz.

#### Sample Rate (SRD)

The ICM-20948 has the ability to calculate the sample rates independently for the accelerometer and gyroscope.  However the current library uses a only the gyroscope sample rate to control both the accelerometer and gyroscope.

For the gyroscope the sample is defined:

```math
rate = 1125 / (srd + 1)
```

while for the accelerometer:
```math
rate = 1000 / (srd + 1)
```

The command for the sample rate is:

**bool ConfigSrd(const uint8_t srd)** Sets the sensor sample rate divider. The ICM-20948 samples the accelerometer and gyro at a rate, in Hz, defined by:

A *srd* setting of 0 means the ICM-20948 samples the accelerometer at a rate if 1125 Hz and gyro at 1000 Hz. A *srd* setting of 4 would set the sampling at 200 Hz for the gyro and 225 Hz. The IMU data ready interrupt is tied to the rate defined by the sample rate divider. For a 100 Hz sample rate a sample rate of 10 is recommended.  This gives 100 Hz for the accelerometer and 112 Hz for the gyroscope.


```C++
/* Set sample rate divider for 50 Hz */
bool status = mpu9250.sample_rate_divider(19);
if (!status) {
  // ERROR
}
```

The magnetometer is sample rate can be set using the following command when include the AK09916 library.
 
**ConfigMeasRate(const MeasRate rate)** Sets the sample rate for the magnetometer.

| Sample Rate | Enum Value |
| --- | --- |
| One Shot | MEAS_RATE_SINGLE |
| 10 Hz | MEAS_RATE_10HZ |
| 50 Hz | MEAS_RATE_50HZ |
| 100 Hz | MEAS_RATE_100HZ |


True is returned on succesfully setting the sample rate , otherwise, false is returned. The default sample rate divider value is 0, resulting in a 100 Hz sample rate.

### ICM-20649

The ICM-20649 is a extended range accelerometer and gyroscope in a single device.  It does provide an on-chip magnetometer so only a 6-axis device.

The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | 
| --- | --- |
| +/- 250 deg/s  | +/- 2g  | 
| +/- 500 deg/s  | +/- 4g  |
| +/- 1000 deg/s | +/- 8g  |
| +/- 2000 deg/s | +/- 16g |
| +/- 4000 deg/s | +/- 30g |

#### Bandwidth

Supports the same functions as ICM-20948 (see above for commands and enumerators).

#### Sample Rate (SRD)

See ICM-20948 for definition, command and setting description

#### Full Scale Ranges for Accel and GyroDlpfBandwidth


**bool ConfigAccelRange(const AccelRange range)** Sets the accelerometer full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 2g | ACCEL_RANGE_2G |
| +/- 4g | ACCEL_RANGE_4G |
| +/- 8g | ACCEL_RANGE_8G |
| +/- 16g | ACCEL_RANGE_16G |
| +/- 30g | ACCEL_RANGE_30G |

True is returned on succesfully setting the accelerometer range, otherwise, false is returned. The default range is +/-30g.

```C++
bool status = icm20649.ConfigAccelRange(bfs::Icm20649::ACCEL_RANGE_4G);
if (!status) {
  // ERROR
}
```

**bool ConfigGyroRange(const GyroRange range)** Sets the gyro full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 250 deg/s | GYRO_RANGE_250DPS |
| +/- 500 deg/s | GYRO_RANGE_500DPS |
| +/- 1000 deg/s | GYRO_RANGE_1000DPS |
| +/- 2000 deg/s | GYRO_RANGE_2000DPS |
| +/- 4000 deg/s | GYRO_RANGE_4000DPS |

True is returned on succesfully setting the gyro range, otherwise, false is returned. The default range is +/-4000 deg/s.

```C++
bool status = icm20649.ConfigGyroRange(bfs::Icm20649::GYRO_RANGE_1000DPS);
if (!status) {
  // ERROR
}
```

### MPU-6050

Current implementation for the MPU-6050 only supports I2C.








# InvensenseImu
This library communicates with [InvensenseMPU-6500](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6500/) and [InvenSense MPU-9250 and MPU-9255](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) Inertial Measurement Units (IMUs). This library is compatible with Arduino and CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
The Invense MPU-6500 is a three-axis gyroscope and three-axis accelerometer. The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 three-axis magnetometer. The MPU-6500 and MPU-9250 support I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 20 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range (MPU-9250 Only) |
| --- | --- | ---  |
| +/- 250 deg/s  | +/- 2g  | +/- 4800 uT |
| +/- 500 deg/s  | +/- 4g  | |
| +/- 1000 deg/s | +/- 8g  | |
| +/- 2000 deg/s | +/- 16g | |

The IMUs sample the gyros, accelerometers, and magnetometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, and an embedded temperature sensor.

# Installation

## Arduino
Use the Arduino Library Manager to install this library or clone to your Arduino/libraries folder.

For the MPU-6500, this library is added as:

```C++
#include "mpu6500.h"
```

For the MPU-9250, this library is added as:

```C++
#include "mpu9250.h"
```

Example Arduino executables are located in: *examples/arduino/*. Teensy 3.x, 4.x, and LC devices are used for testing under Arduino and this library should be compatible with other Arduino devices.

## CMake
CMake is used to build this library, which is exported as a library target called *invensense_imu*.

For the MPU-6500, this library is added as:

```C++
#include "mpu6500.h"
```

For the MPU-9250, this library is added as:

```C++
#include "mpu9250.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *i2c_example*, *spi_example*, *drdy_spi_example*, and *wom_example* (MPU-9250 only). The example executable source files are located at *examples/cmake*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41
   * IMXRT1062_MMOD

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The example targets create executables for communicating with the sensor using I2C or SPI communication, using the data ready interrupt, and using the wake on motion interrupt, respectively. Each target also has a *_hex*, for creating the hex file to upload to the microcontroller, and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools).

# Namespace
This library is within the namespace *bfs*.

# InvensenseImu
This class provides methods for reading and writing to registers on these sensors. It's expected that this should work with at least the MPU-6000, MPU-6050, MPU-6500, MPU-9150, and MPU-9250; although, it may work for other sensors as well. Most users will likely prefer the sensor specific classes, below; however, this class may enable people to unlock greater functionality and use as a starting point for their own sensor drivers.

## Methods

**InvensenseImu()** Default constructor, requires calling the *Config* method to setup the I2C or SPI bus and I2C address or SPI chip select pin.

**InvensenseImu(TwoWire &ast;i2c, const uint8_t addr)** Creates a InvensenseImu object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with the I2C address of the sensor.

**InvensenseImu(SPIClass &ast;bus, uint8_t cs)** Creates a InvensenseImu object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

**void Config(TwoWire &ast;bus, const uint8_t addr)** This is required when using the default constructor and sets up the I2C bus and I2C address.

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**void Begin()** Initializes communication with the sensor. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

**bool WriteRegister(const uint8_t reg, const uint8_t data, const int32_t spi_clock)** Writes register data to the sensor given the register address and the data. The SPI clock speed must be specified if SPI communication is used.

**bool WriteRegister(const uint8_t reg, const uint8_t data)** Overload of the above where I2C communication is used.

**bool ReadRegisters(const uint8_t reg, const uint8_t count, const int32_t spi_clock, uint8_t &ast; const data)** Reads register data from the sensor given the register address, the number of registers to read, the SPI clock, and a pointer to store the data.

**bool ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t &ast; const data)** Overload of the above where I2C communication is used.

# Mpu9250
This class works with the MPU-9250 and MPU-9255 IMUs.

## Methods

**Mpu9250()** Default constructor, requires calling the *Config* method to setup the I2C or SPI bus and I2C address or SPI chip select pin.

**Mpu9250(i2c_t3 &ast;bus, I2cAddr addr)** Creates a Mpu9250 object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with the I2C address of the sensor. The address will be I2C_ADDR_PRIM (0x68) if the AD0 pin is grounded and I2C_ADDR_SEC (0x69) if the AD0 pin is pulled high.

```C++
Mpu9250 mpu9250(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
```

**Mpu9250(SPIClass &ast;bus, uint8_t cs)** Creates a Mpu9250 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
Mpu9250 mpu9250(&SPI, 2);
```

**void Config(TwoWire &ast;bus, const I2cAddr addr)** This is required when using the default constructor and sets up the I2C bus and I2C address. The address will be I2C_ADDR_PRIM (0x68) if the AD0 pin is grounded and I2C_ADDR_SEC (0x69) if the AD0 pin is pulled high.

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**bool Begin()** Initializes communication with the sensor and configures the default sensor ranges, sampling rates, and low pass filter settings. The default accelerometer range is +/- 16g and the default gyro range is +/- 2,000 deg/s. The default sampling rate is 1000 Hz and the low-pass filter is set to a cutoff frequency of 184 Hz. True is returned if communication is able to be established with the sensor and configuration completes successfully, otherwise, false is returned. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
Wire.begin();
Wire.setClock(400000);
bool status = mpu9250.Begin();
if (!status) {
  // ERROR
}
```

**bool EnableDrdyInt()** Enables the data ready interrupt. A 50 us interrupt will be triggered on the MPU-9250 INT pin when IMU data is ready. This interrupt is active high. This method returns true if the interrupt is successfully enabled, otherwise, false is returned.

```C++
bool status = mpu9250.EnableDrdyInt();
if (!status) {
  // ERROR
}
```

**bool DisableDrdyInt()** Disables the data ready interrupt. This method returns true if the interrupt is successfully disabled, otherwise, false is returned.

```C++
bool status = mpu9250.DisableDrdyInt();
if (!status) {
  // ERROR
}
```

**bool ConfigAccelRange(const AccelRange range)** Sets the accelerometer full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 2g | ACCEL_RANGE_2G |
| +/- 4g | ACCEL_RANGE_4G |
| +/- 8g | ACCEL_RANGE_8G |
| +/- 16g | ACCEL_RANGE_16G |

True is returned on succesfully setting the accelerometer range, otherwise, false is returned. The default range is +/-16g.

```C++
bool status = mpu9250.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G);
if (!status) {
  // ERROR
}
```

**AccelRange accel_range()** Returns the current accelerometer range.

```C++
AccelRange range = mpu9250.accel_range();
```

**bool ConfigGyroRange(const GyroRange range)** Sets the gyro full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 250 deg/s | GYRO_RANGE_250DPS |
| +/- 500 deg/s | GYRO_RANGE_500DPS |
| +/- 1000 deg/s | GYRO_RANGE_1000DPS |
| +/- 2000 deg/s | GYRO_RANGE_2000DPS |

True is returned on succesfully setting the gyro range, otherwise, false is returned. The default range is +/-2000 deg/s.

```C++
bool status = mpu9250.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_1000DPS);
if (!status) {
  // ERROR
}
```

**GyroRange gyro_range()** Returns the current gyro range.

```C++
GyroRange range = mpu9250.gyro_range();
```

**bool ConfigSrd(const uint8_t srd)** Sets the sensor sample rate divider. The MPU-9250 samples the accelerometer and gyro at a rate, in Hz, defined by:

```math
rate = 1000 / (srd + 1)
```

A *srd* setting of 0 means the MPU-9250 samples the accelerometer and gyro at 1000 Hz. A *srd* setting of 4 would set the sampling at 200 Hz. The IMU data ready interrupt is tied to the rate defined by the sample rate divider. The magnetometer is sampled at 100 Hz for sample rate divider values corresponding to 100 Hz or greater. Otherwise, the magnetometer is sampled at 8 Hz.

True is returned on succesfully setting the sample rate divider, otherwise, false is returned. The default sample rate divider value is 0, resulting in a 1000 Hz sample rate.

```C++
/* Set sample rate divider for 50 Hz */
bool status = mpu9250.sample_rate_divider(19);
if (!status) {
  // ERROR
}
```

**uint8_t srd()** Returns the current sample rate divider value.

```C++
uint8_t srd = mpu9250.srd();
```

**bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf)** Sets the cutoff frequency of the digital low pass filter for the accelerometer, gyro, and temperature sensor. Available bandwidths are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 184 Hz | DLPF_BANDWIDTH_184HZ |
| 92 Hz | DLPF_BANDWIDTH_92HZ |
| 41 Hz | DLPF_BANDWIDTH_41HZ |
| 20 Hz | DLPF_BANDWIDTH_20HZ |
| 10 Hz | DLPF_BANDWIDTH_10HZ |
| 5 Hz | DLPF_BANDWIDTH_5HZ |

True is returned on succesfully setting the digital low pass filters, otherwise, false is returned. The default bandwidth is 184 Hz.

```C++
bool status = mpu9250.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
if (!status) {
  // ERROR
}
```

**DlpfBandwidth dlpf_bandwidth()** Returns the current digital low pass filter bandwidth setting.

```C++
DlpfBandwidth dlpf = mpu9250.dlpf_bandwidth();
```

**bool EnableWom(int16_t threshold_mg, const WomRate wom_rate)** Enables the Wake-On-Motion interrupt. It places the MPU-9250 into a low power state, waking up at an interval determined by the *WomRate*. If the accelerometer detects motion in excess of the threshold, *threshold_mg*, it generates a 50us pulse from the MPU-9250 interrupt pin. The following enumerated WOM rates are supported:

| WOM Sample Rate |  Enum Value      |
| ---------------- |  ------------------   |
| 0.24 Hz          |  WOM_RATE_0_24HZ  |
| 0.49 Hz          |  WOM_RATE_0_49HZ  |
| 0.98 Hz          |  WOM_RATE_0_98HZ  |
| 1.95 Hz          |  WOM_RATE_1_95HZ  | 
| 3.91 Hz          |  WOM_RATE_3_91HZ  |
| 7.81 Hz          |  WOM_RATE_7_81HZ  |
| 15.63 Hz         |  WOM_RATE_15_63HZ |
| 31.25 Hz         |  WOM_RATE_31_25HZ |
| 62.50 Hz         |  WOM_RATE_62_50HZ |
| 125 Hz           |  WOM_RATE_125HZ   |
| 250 Hz           |  WOM_RATE_250HZ   |
| 500 Hz           |  WOM_RATE_500HZ   |

The motion threshold is given as a value between 4 and 1020 mg, which is internally mapped to a single byte, 1-255 value. This function returns true on successfully enabling Wake On Motion, otherwise returns false. Please see the *wom_i2c* example. The following is an example of enabling the wake on motion with a 40 mg threshold and a ODR of 31.25 Hz.

```C++
imu.EnableWom(40, bfs::Mpu9250::WOM_RATE_31_25HZ);
```

**void Reset()** Resets the MPU-9250.

**bool Read()** Reads data from the MPU-9250 and stores the data in the Mpu9250 object. Returns true if data is successfully read, otherwise, returns false.

```C++
/* Read the IMU data */
if (mpu9250.Read()) {
}
```

**bool new_imu_data()** Returns true if new data was returned from the accelerometer and gyro.

```C++
if (mpu9250.Read()) {
  bool new_data = mpu9250.new_imu_data();
}
```

**bool new_mag_data()** Returns true if new data was returned from the magnetometer. For MPU-9250 sample rates of 100 Hz and higher, the magnetometer is sampled at 100 Hz. For MPU-9250 sample rates less than 100 Hz, the magnetometer is sampled at 8 Hz, so it is not uncommon to receive new IMU data, but not new magnetometer data.

```C++
if (mpu9250.Read()) {
  bool new_mag = mpu9250.new_mag_data();
}
```

**float accel_x_mps2()** Returns the x accelerometer data from the Mpu9250 object in units of m/s/s. Similar methods exist for the y and z axis data.

```C++
/* Read the IMU data */
if (mpu9250.Read()) {
  float ax = mpu9250.accel_x_mps2();
  float ay = mpu9250.accel_y_mps2();
  float az = mpu9250.accel_z_mps2();
}
```

**float gyro_x_radps()** Returns the x gyro data from the Mpu9250 object in units of rad/s. Similar methods exist for the y and z axis data.

```C++
/* Read the IMU data */
if (mpu9250.Read()) {
  float gx = mpu9250.gyro_x_radps();
  float gy = mpu9250.gyro_y_radps();
  float gz = mpu9250.gyro_z_radps();
}
```

**float mag_x_ut()** Returns the x magnetometer data from the Mpu9250 object in units of uT. Similar methods exist for the y and z axis data.

```C++
/* Read the IMU data */
if (mpu9250.Read()) {
  float hx = mpu9250.mag_x_ut();
  float hy = mpu9250.mag_y_ut();
  float hz = mpu9250.mag_z_ut();
}
```

**float die_temp_c()** Returns the die temperature of the sensor in units of C.

```C++
/* Read the IMU data */
if (mpu9250.Read()) {
  float temp = mpu9250.die_temp_c();
}
```

# Mpu6500
This class works with the MPU-6500 sensor.

## Methods

**Mpu6500()** Default constructor, requires calling the *Config* method to setup the I2C or SPI bus and I2C address or SPI chip select pin.

**Mpu6500(i2c_t3 &ast;bus, I2cAddr addr)** Creates a Mpu6500 object. This constructor is used for the I2C communication interface. A pointer to the I2C bus object is passed along with the I2C address of the sensor. The address will be I2C_ADDR_PRIM (0x68) if the AD0 pin is grounded and I2C_ADDR_SEC (0x69) if the AD0 pin is pulled high.

```C++
Mpu6500 mpu6500(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
```

**Mpu6500(SPIClass &ast;bus, uint8_t cs)** Creates a Mpu6500 object. This constructor is used for the SPI communication interface. A pointer to the SPI bus object is passed along with the chip select pin of the sensor. Any pin capable of digital I/O can be used as a chip select pin.

```C++
Mpu6500 mpu6500(&SPI, 2);
```

**void Config(TwoWire &ast;bus, const I2cAddr addr)** This is required when using the default constructor and sets up the I2C bus and I2C address. The address will be I2C_ADDR_PRIM (0x68) if the AD0 pin is grounded and I2C_ADDR_SEC (0x69) if the AD0 pin is pulled high.

**void Config(SPIClass &ast;spi, const uint8_t cs)** This is required when using the default constructor and sets up the SPI bus and chip select pin.

**bool Begin()** Initializes communication with the sensor and configures the default sensor ranges, sampling rates, and low pass filter settings. The default accelerometer range is +/- 16g and the default gyro range is +/- 2,000 deg/s. The default sampling rate is 1000 Hz and the low-pass filter is set to a cutoff frequency of 184 Hz. True is returned if communication is able to be established with the sensor and configuration completes successfully, otherwise, false is returned. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
Wire.begin();
Wire.setClock(400000);
bool status = mpu6500.Begin();
if (!status) {
  // ERROR
}
```

**bool EnableDrdyInt()** Enables the data ready interrupt. A 50 us interrupt will be triggered on the MPU-9250 INT pin when IMU data is ready. This interrupt is active high. This method returns true if the interrupt is successfully enabled, otherwise, false is returned.

```C++
bool status = mpu6500.EnableDrdyInt();
if (!status) {
  // ERROR
}
```

**bool DisableDrdyInt()** Disables the data ready interrupt. This method returns true if the interrupt is successfully disabled, otherwise, false is returned.

```C++
bool status = mpu6500.DisableDrdyInt();
if (!status) {
  // ERROR
}
```

**bool ConfigAccelRange(const AccelRange range)** Sets the accelerometer full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 2g | ACCEL_RANGE_2G |
| +/- 4g | ACCEL_RANGE_4G |
| +/- 8g | ACCEL_RANGE_8G |
| +/- 16g | ACCEL_RANGE_16G |

True is returned on succesfully setting the accelerometer range, otherwise, false is returned. The default range is +/-16g.

```C++
bool status = mpu6500.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G);
if (!status) {
  // ERROR
}
```

**AccelRange accel_range()** Returns the current accelerometer range.

```C++
AccelRange range = mpu6500.accel_range();
```

**bool ConfigGyroRange(const GyroRange range)** Sets the gyro full scale range. Options are:

| Range | Enum Value |
| --- | --- |
| +/- 250 deg/s | GYRO_RANGE_250DPS |
| +/- 500 deg/s | GYRO_RANGE_500DPS |
| +/- 1000 deg/s | GYRO_RANGE_1000DPS |
| +/- 2000 deg/s | GYRO_RANGE_2000DPS |

True is returned on succesfully setting the gyro range, otherwise, false is returned. The default range is +/-2000 deg/s.

```C++
bool status = mpu6500.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_1000DPS);
if (!status) {
  // ERROR
}
```

**GyroRange gyro_range()** Returns the current gyro range.

```C++
GyroRange range = mpu6500.gyro_range();
```

**bool ConfigSrd(const uint8_t srd)** Sets the sensor sample rate divider. The MPU-9250 samples the accelerometer and gyro at a rate, in Hz, defined by:

```math
rate = 1000 / (srd + 1)
```

A *srd* setting of 0 means the MPU-9250 samples the accelerometer and gyro at 1000 Hz. A *srd* setting of 4 would set the sampling at 200 Hz. The IMU data ready interrupt is tied to the rate defined by the sample rate divider. The magnetometer is sampled at 100 Hz for sample rate divider values corresponding to 100 Hz or greater. Otherwise, the magnetometer is sampled at 8 Hz.

True is returned on succesfully setting the sample rate divider, otherwise, false is returned. The default sample rate divider value is 0, resulting in a 1000 Hz sample rate.

```C++
/* Set sample rate divider for 50 Hz */
bool status = mpu6500.sample_rate_divider(19);
if (!status) {
  // ERROR
}
```

**uint8_t srd()** Returns the current sample rate divider value.

```C++
uint8_t srd = mpu6500.srd();
```

**bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf)** Sets the cutoff frequency of the digital low pass filter for the accelerometer, gyro, and temperature sensor. Available bandwidths are:

| DLPF Bandwidth | Enum Value |
| --- | --- |
| 184 Hz | DLPF_BANDWIDTH_184HZ |
| 92 Hz | DLPF_BANDWIDTH_92HZ |
| 41 Hz | DLPF_BANDWIDTH_41HZ |
| 20 Hz | DLPF_BANDWIDTH_20HZ |
| 10 Hz | DLPF_BANDWIDTH_10HZ |
| 5 Hz | DLPF_BANDWIDTH_5HZ |

True is returned on succesfully setting the digital low pass filters, otherwise, false is returned. The default bandwidth is 184 Hz.

```C++
bool status = mpu6500.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ);
if (!status) {
  // ERROR
}
```

**DlpfBandwidth dlpf_bandwidth()** Returns the current digital low pass filter bandwidth setting.

```C++
DlpfBandwidth dlpf = mpu6500.dlpf_bandwidth();
```

**bool Read()** Reads data from the MPU-6500 and stores the data in the Mpu6500 object. Returns true if data is successfully read, otherwise, returns false.

```C++
/* Read the IMU data */
if (mpu6500.Read()) {
}
```

**bool new_imu_data()** Returns true if new data was returned from the accelerometer and gyro.

```C++
if (mpu6500.Read()) {
  bool new_data = mpu6500.new_imu_data();
}
```

**float accel_x_mps2()** Returns the x accelerometer data from the Mpu6500 object in units of m/s/s. Similar methods exist for the y and z axis data.

```C++
/* Read the IMU data */
if (mpu6500.Read()) {
  float ax = mpu6500.accel_x_mps2();
  float ay = mpu6500.accel_y_mps2();
  float az = mpu6500.accel_z_mps2();
}
```

**float gyro_x_radps()** Returns the x gyro data from the Mpu6500 object in units of rad/s. Similar methods exist for the y and z axis data.

```C++
/* Read the IMU data */
if (mpu6500.Read()) {
  float gx = mpu6500.gyro_x_radps();
  float gy = mpu6500.gyro_y_radps();
  float gz = mpu6500.gyro_z_radps();
}
```

**float die_temp_c()** Returns the die temperature of the sensor in units of C.

```C++
/* Read the IMU data */
if (mpu6500.Read()) {
  float temp = mpu6500.die_temp_c();
}
```

## Sensor Orientation
This library transforms all data to a common axis system before it is returned. This axis system is shown below. It is a right handed coordinate system with the z-axis positive down, common in aircraft dynamics.

![MPU-9250 Orientation](docs/MPU-9250-AXIS.png)

**Caution!** This axis system is shown relative to the MPU-6500 and MPU-9250 sensor. The sensor may be rotated relative to the breakout board. 
