/*
Madgwick Fusion Library:
	The MIT License (MIT)
	Copyright (c) 2021 x-io Technologies
BolderFlight invensense-imu library
	The MIT License (MIT)
	Copyright (c) 2022 Bolder Flight Systems Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mpu6050.h"
#define HMC5983A

/* Mpu9250 object */
bfs::Mpu6050 imu(&Wire, bfs::Mpu6050::I2C_ADDR_PRIM);
#if defined(HMC5983A)
#include "HMC5983.h"
HMC5983 mag(&Wire);
#endif

#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>

#include <Wire.h>
#include "Streaming.h"
#include <string>

#include "constants.h" 
#define printf Serial.printf


void gyroCalibration();

void telemetryPortOut();
void print_float_array(float *arr, int size);

bool dataRdy;


// scale factors
float accelScale, gyroScale;
float magScale[3];

FusionOffset offset;
FusionAhrs ahrs;
FusionQuaternion q;
FusionEuler euler;
FusionAhrsFlags flags;

// Define calibration
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//gyroscop offsets and sensitity configured in gyroCalibration()
FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

//Accelerometer calibration configured in getCalIMU();
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};

//Magnetometer calibration configured = motioncal for 
const FusionMatrix softIronMatrix = {0.9825,-0.0144,0.0187,-0.0144,1.0732,0.0098,0.0187,0.0098,0.9490};
const FusionVector hardIronOffset = {1.763,-17.680,-0.378};

  
  //new data available
volatile int newIMUData;
uint32_t lastUpdate, now1;


void setup() {
  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  SerialUSB1.begin(115200);	// USB, communication to PC or Mac


  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }

  cout.println("IMU Connected!");

  /* Set the sample rate divider */
  // rate = 1000 / (srd + 1)
  // = 1000/20 = 50 hz
  // = 100 hz
  if (!imu.ConfigSrd(9)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  /* MAG */
#if defined(HMC5983A)
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with MAG");
    while (1) {}
  }
  mag.setRange(HMC5983_RANGE_8_1GA);
  mag.setMeasurementMode(HMC5983_CONTINOUS);
  mag.setSampleAverages(HMC5983_SAMPLEAVERAGE_8);
  mag.setDataRate(HMC5983_DATARATE_75HZ);
#endif
  Serial.println("Setup Complete");

  SAMPLE_RATE = 0.01f;	//100hz

  imu.getScales(&accelScale, &gyroScale, magScale);
  gyroCalibration();

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
    
  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .convention = FusionConventionNed,
          .gain = 0.5f, //1.5f,
          .gyroscopeRange = 2000.0f, // replace this with actual gyroscope range in degrees/s
          .accelerationRejection = 10.0f,
          .magneticRejection = 10.0f, //0.0f,
          .recoveryTriggerPeriod  = (uint8_t)(5.0f / SAMPLE_RATE ), // 5 seconds
  };

  FusionAhrsSetSettings(&ahrs, &settings);


  cout.println("Ready for commands....");
  menu();
}

void loop() {
    if ( cout.available() ) {
      char rr;
      rr = cout.read();
      switch (rr) {
        case 'g':
          rr = '0';
          gyroCalibration();
          break;
        case 'r':
        {
          rr ='0';
          if(raw_values_on == 1) {
            raw_values_on = 0;
          } else if(raw_values_on == 0) {
            raw_values_on = 1;
          }
        }
          break;
        case 's':
        {
          rr ='0';
          if(serial_values_on == 1) {
            serial_values_on = 0;
          } else if(serial_values_on == 0) {
            serial_values_on = 1;
          }
        }
          break;
        case 'x':
        {
          rr ='0';
          if(x_values_on == 1) {
            x_values_on = 0;
          } else if(serial_values_on == 0) {
            x_values_on = 1;
          }
        }
            break;
        case 't':
        {
          rr ='0';
          if(telem_values_on == 1) {
            telem_values_on = 0;
          } else if(telem_values_on == 0) {
            telem_values_on = 1;
          }
        }
            break;
        case 'f':
        {
          rr = '0';
          if(fusion_on == 1) {
            fusion_on = 0;
          } else if(fusion_on == 0) {
            fusion_on = 1;
          }
        }
          break;
        case '\r':
        case '\n':
        case 'h': menu(); break;
      }
      while (cout.read() != -1) ; // remove rest of characters.
    }

  if(fusion_on == 1) {
    getFusion();
  }
}


void getCalIMU() {
  // read the sensor
  /* Check if data read */
  //NOTE for Fusion gyro data rate is the driver not the magnetomer
  //if (imu.Read_raw(raw_values) & imu.new_imu_data_()) {
  if (imu.Read()) {
    newIMUData = imu.new_imu_data();
    now1= micros(); // This is when the data reported READY

    val[0] = imu.accel_x_mps2() / g;
    val[1] = imu.accel_y_mps2() / g;
    val[2] = imu.accel_z_mps2() / g;
    val[3] = imu.gyro_x_radps() * rads2degs;
    val[4] = imu.gyro_y_radps() * rads2degs;
    val[5] = imu.gyro_z_radps() * rads2degs;

    //appling accel calibration
    val[0] = (val[0] - acc_off[0]) * acc_scale[0] ;
    val[1] = (val[1] - acc_off[1]) * acc_scale[1] ;
    val[2] = (val[2] - acc_off[2]) * acc_scale[2] ;
  #if defined(HMC5983A)
    mag.getMagScaled(mag_val);
    val[6] = -mag_val[0];
    val[7] = mag_val[1];
    val[8] = -mag_val[2];
  #endif
  }
 
}

void getFusion() {
  getCalIMU();

  if(newIMUData) {
    newIMUData = 0;
    dt = ((now1 - lastUpdate) * 0.000001);
    lastUpdate = now1;

    // Acquire latest sensor data
    FusionVector gyroscope = { val[3], val[4], val[5] }; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = { val[0], val[1], val[2] }; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {  val[6], val[7], val[8] }; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Update gyroscope offset correction algorithm
    //gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);

    flags = FusionAhrsGetFlags(&ahrs);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    q = FusionAhrsGetQuaternion(&ahrs);
    fHeading = FusionCompassCalculateHeading(FusionConventionNed, accelerometer, magnetometer);


    //if(dump > 100) {
      if(raw_values_on == 1) {
        print_float_array(val, 9);
        Serial.println();
      }

      if(serial_values_on == 1) {
        printf("Fusion (RPY): %0.1f, %0.1f, %0.1f, %0.1f\n", fHeading, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
      }
  
      if(x_values_on == 1) {
        timestamp = micros();
        char accelgyroBuffer[100];
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f", 'I', timestamp, gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z, accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f", 'M', timestamp, magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
        printf ("%s\n",accelgyroBuffer);
        sprintf(accelgyroBuffer, "%c,%llu,%0.6f,%0.6f,%0.6f,%0.6f", 'Q', timestamp, q.element.w, q.element.x, q.element.y, q.element.z);
        printf ("%s\n",accelgyroBuffer);
      }

      if(telem_values_on == 1) telemetryPortOut();
      //dump = 0;
  }
}

void gyroCalibration() {
  int numSamples = 500;
  gyrox_off = 0;
  gyroy_off = 0;
  gyroz_off = 0;
  //imu.DisableDrdyInt();
  uint16_t icount = 0;
  while(icount < numSamples){
    imu.Read();
    if(imu.new_imu_data()){
      icount += 1;
      gyrox_off += imu.gyro_x_radps() * rads2degs;;
      gyroy_off += imu.gyro_y_radps() * rads2degs;;
      gyroz_off += imu.gyro_z_radps() * rads2degs;;
    }
  }
  gyrox_off = gyrox_off / numSamples;
  gyroy_off = gyroy_off / numSamples;
  gyroz_off = gyroz_off / numSamples;

  printf("%f, %f, %f\n", gyrox_off, gyroy_off,gyroz_off );
  gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
  gyroscopeOffset = {gyrox_off, gyroy_off, gyroz_off};
}

void menu()
{
  cout.println();
  cout.println("Menu Options:");
  cout.println("========================================");
  cout.println("\tx - x-IMU3 GUI Output");
  cout.println("\tt - Telemetry Viewer Output");
  cout.println("\ts - Serial Print Output (Euler Angles)");
  cout.println("\tf - Fusion On");
  cout.println("\tr - Print Values");
  cout.println("========================================");
  cout.println("\tg - Zero Gyroscope");

  cout.println("========================================");
  cout.println("\th - Menu");
  cout.println("========================================");
  cout.println();
}
