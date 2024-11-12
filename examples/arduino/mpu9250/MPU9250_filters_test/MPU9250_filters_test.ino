
//#define mpu9250
#define m5stack

#if defined(mpu9250)
#include "mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;
#elif defined(m5stack)
#include "BMI270_AUX_BMM150.h"
#endif

float val[10];
int16_t raw_val[10];

float quant[4], ypr[3], _dt, q[4];
float grx, gry, grz;

// gyro bias estimation
float gyrox_off, gyroy_off, gyroz_off;
float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1;

void gyroCalibration();

//Set print rate, sincePrint > printRate
#define printRate      100  //milliseconds
bool execFilter = false;
volatile int newIMUData;
unsigned long lastUpdate, now1;  // sample period expressed in milliseconds for filters

#if defined(m5stack)
uint32_t sample_rate = 0;
uint8_t new_mag_data = 0;
uint8_t new_accel_data = 0;
uint8_t new_gyro_data = 0;
#endif

elapsedMillis sincePrint;

//trig conversions
float rad2deg = 180.0f / PI;
float deg2rad = PI / 180.0f;
//standard gravity in m/s/s
#define gravConst  9.80665

//////////////////////////////////////////////////////////////////////////////
//
//   FILTER SELECTION
//////////////////////////////////////////////////////////////////////////////
//---- Filter Selection ------
// Set filter type: 
//  0 = Madgwick implementation of Mahoney DCM
//  1 = Madgwick Gradient Descent, in Quaternion form
//  2 = xxxxxxxxxx
//  3 = Madwick Original Paper AHRS 
//  4 = DCM Implementation - miniIMU-9 implementation
//  5 = Complimentary Filter
//  6 = uNavAHRS
//  7 = DCM Algorithm - 
//  8 = PJRC NXPMotionSense
// Select the filter type here
#define MARG 3

// Uncomment this line to disable the magnetometer in the 
// sensor fusion algorithm
//#define DISABLE_MAGN 

// Magnetic declination. If used, must be selected for your location.
// Note: May not be used in this sketch yet, needs to be checked.
//#define MAG_DEC 12.83      //degrees for Flushing, NY
#define MAG_DEC 0

/*
 * https://arduino.stackexchange.com/questions/34095/how-do-i-configure-the-arduino-ide-to-look-for-source-code-in-a-subdirectory-wit#:~:text=In%20the%20Arduino%20IDE%20the%20answer%20is%20simple%3A,they%20are.INO%20or.PDE%29%20and%20then%20compiles%20them%20there.
*/
#if MARG == 0
  #include "src/Filters/AHRS.h"
  #include "src/Filters/auxFuncs.h"

#elif MARG == 1
  #include "src/Filters/MadgwickAHRS.h"
  #include "src/Filters/auxFuncs.h"

#elif MARG == 2
  #include "src/Filters/MahonyAHRS.h"
  #include "src/Filters/auxFuncs.h"

#elif MARG == 3
  #include "src/Filters/MargUpdateFilter.h"
  #include "src/Filters/auxFuncs.h"

#elif MARG == 4
  #include "src/Filters/DCM.h"
  DCM dcm;
  #define use_DCM
  #include "src/Filters/auxFuncs.h"

#elif MARG == 5
  #include "src/Filters/compFlt.h"
  #include "src/Filters/auxFuncs.h"

#elif MARG == 6
  #include "src/Filters/uNavAHRS.h"
  uNavAHRS Filter;
  #define use_unavahrs
  #include "src/Filters/auxFuncs.h"

#elif MARG == 7
  #include "src/Filters/DcmAlgorithm.h"  //https://github.com/alrevuelta/dcm-algorithm-cpp
  DcmAlgorithm dcmAlgorithm;
  #define use_dcmalgo
  #include "src/Filters/auxFuncs.h"

#elif MARG == 8
  #include "src/Filters/NXPSensorFusion.h"
  NXPSensorFusion filter;
  
#endif


#if defined(mpu9250)
#define use_freeimu
//Freeimu calibration
int cal_acc_off[] = {13, 43, 74};
int cal_acc_scale[]  = {2030, 2069, 2090};
int cal_magn_off[]  = {73, 168, -190};
int cal_magn_scale[]  = {226, 250, 248};

//Min - max
float acc_off[] = {0.192162, 0.192162, -0.384299};
float acc_scale[]  = {0.998882, 1.002871, 0.986058};
float magn_off[]  = {13.829204, 27.536438, -32.913139};
float magn_scale[]  = {0.966554, 1.038114, 0.997893};

#elif defined(m5stack)
//Min - max
float acc_off[] = {-0.039800, -0.039800, 0.226332};
float acc_scale[]  = {0.813310, 0.812575, 0.806854};
float magn_off[]  = {-8.328297, 23.305244, -9.240059};
float magn_scale[]  = {0.991641, 0.990220, 1.018647};
#endif

// scale factors
float accelScale, gyroScale;
float magScale[3];


void setup() {
  /* Serial to display data */
  while(!Serial && millis() < 5000) {}
  Serial.begin(115200);
  
  // If Teensy 4.x fails print Crashreport to serial monitor
  if (CrashReport) {
      Serial.print(CrashReport);
      Serial.println("Press any key to continue");
      while (Serial.read() != -1) {
      }
      while (Serial.read() == -1) {
      }
      while (Serial.read() != -1) {
      }
  }

  /* Start the I2C bus */
  #if defined(mpu9250)
  Wire2.begin();
  Wire2.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire2, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }

  /* Set the sample rate divider */
  // rate = 1000 / (srd + 1)
  // = 1000/20 = 50 hz
  // = 100 hz
  if (!imu.ConfigSrd(9)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  //Get MPU sensitivity values
  imu.getScales(&accelScale, &gyroScale, magScale);
  
  gyroCalibration();
  #elif defined(m5stack)
  IMU.setAcellConfig(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_16G, BMI2_ACC_NORMAL_AVG4);
  IMU.setGyroConfig(BMI2_GYR_ODR_200HZ, BMI2_GYR_RANGE_2000, BMI2_GYR_NORMAL_MODE);
  IMU.setMagConfig(BMM150_POWERMODE_NORMAL, BMM150_PRESETMODE_REGULAR);

  Serial.println("Beginning IMU Initialization...");

  if (!IMU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }
  Serial.println("IMU Initialization Complete");
  Serial.println(" ");

  Serial.println("IMU Connected!");
#endif

  Serial.printf("Using MARG: %d\n", MARG);

  #if (MARG == 8)
    filter.begin(100);  //100 hertz
  #endif
  
}

void loop() {
  //Option to recal gyro
  char ch = '9';

  if (Serial.available()) {
    ch = Serial.read();
    if ( ch == 'g' ) gyroCalibration();
    if ( ch == 'e' ) execFilter = true;
    if ( ch == 'x' ) execFilter = false;
  }
  

  if ((execFilter == true)) {
    newIMUData = 0;
    getIMU();

    if( (sincePrint > printRate)){
      //digitalWrite(13, HIGH);
      sincePrint = 0;
      #if (MARG == 5)
        //compFilter(gx1, gy1, gz1, ax1, ay1, az1, 0, 0, 0);    //yaw based on filter alone
        compFilter(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1, _dt);//yaw calculated with magnetometer
      #elif(MARG == 4)
        dcm.getEulerDeg(ypr);
      #elif (MARG == 6)
        ypr[0] = Filter.getHeading_rad()*180.0f/PI;
        ypr[1] = Filter.getPitch_rad()*180.0f/PI;
        ypr[2] = Filter.getRoll_rad()*180.0f/PI;
      #elif MARG == 7
        ypr[0] = dcmAlgorithm.getYaw();
        ypr[1] = dcmAlgorithm.getPitch();
        ypr[2] = dcmAlgorithm.getRoll();
      #elif MARG == 8
        ypr[0] = 360.0f - filter.getYaw();
        ypr[1] = filter.getRoll();
        ypr[2] = filter.getPitch();
      #elif MARG != 5
        getYawPitchRoll(ypr);
     #endif
      
      //#if defined(m5stack)
      //  ypr[0] = -ypr[0];
      //#endif
      Serial.printf("%f, %f, %f\n", ypr[0], ypr[1], ypr[2]);
      //Serial.printf("%f, %f\n",Filter.getLongitude_rad()*180.0f/PI, Filter.getLatitude_rad()*180.0f/PI);
            
      //digitalWrite(13, LOW);
    } //end sincePrint
  } //end newIMUdata

  
}

#if defined(mpu9250)
void getCalIMU() {
  // read the sensor
  /* Check if data read */
  //NOTE for Fusion gyro data rate is the driver not the magnetomer
  if (imu.Read_raw(raw_val)) {
  //if (imu.Read_raw(raw_val)) {
    newIMUData = imu.new_imu_data();
    now1= micros(); // This is when the data reported READY
    //Serial.printf("%d, %d, %d, %d, %d, %d, %d, %df, %d\n", raw_val[0], raw_val[1], raw_val[2], raw_val[3], raw_val[4], raw_val[5], raw_val[6], raw_val[7], raw_val[8]);
    val[4] = ((float)(raw_val[3] * gyroScale) - gyrox_off) * deg2rad;
    val[3] = ((float)(raw_val[4] * gyroScale) - gyroy_off) * deg2rad;
    val[5] = -((float)(raw_val[5] * gyroScale) - gyroz_off) * deg2rad;
#if defined(use_freeimu)
    val[0] = ((float)(raw_val[1] - cal_acc_off[1]) / (float)cal_acc_scale[1]) ;
    val[1] = ((float)(raw_val[0] - cal_acc_off[0]) / (float)cal_acc_scale[0])  ; 
    val[2] = -((float)(raw_val[2] - cal_acc_off[2])  / (float)cal_acc_scale[2]) ;
    val[6] = ((float)(raw_val[6] - cal_magn_off[0]) / (float)cal_magn_scale[0]);
    val[7] =  ((float)(raw_val[7] - cal_magn_off[1]) / (float)cal_magn_scale[1]) ;
    val[8] = ((float)(raw_val[8] - cal_magn_off[2])  / (float)cal_magn_scale[2]) ;
#else
    val[6] = (((float)raw_val[6] * magScale[0]) - magn_off[0]) * magn_scale[0];
    val[7] = (((float)raw_val[7] * magScale[1]) - magn_off[1]) * magn_scale[1];
    val[8] = (((float)raw_val[8] * magScale[2]) - magn_off[2]) * magn_scale[2];
    val[0] = (((float)raw_val[1] * accelScale) - acc_off[0]/gravConst) * acc_scale[0] ;
    val[1] = (((float)raw_val[0] * accelScale) - acc_off[1]/gravConst) * acc_scale[1] ;
    val[2] = ((-(float)raw_val[2] * accelScale) - acc_off[2]/gravConst) * acc_scale[2];
#endif
  }
}

void gyroCalibration() {
  int numSamples = 500;
  gyrox_off = 0;
  gyroy_off = 0;
  gyroz_off = 0;
  //imu.DisableDrdyInt();
  for(int i = 0; i < numSamples; i++){
    imu.Read_raw(raw_val);
    gyrox_off += raw_val[3] * gyroScale;
    gyroy_off += raw_val[4] * gyroScale;
    gyroz_off += raw_val[5] * gyroScale;
  }
  gyrox_off = gyrox_off / numSamples;
  gyroy_off = gyroy_off / numSamples;
  gyroz_off = gyroz_off / numSamples;
  //imu.EnableDrdyInt();
  Serial.printf("%f, %f, %f\n", gyrox_off, gyroy_off,gyroz_off );
}

#elif defined(m5stack)
void getCalIMU() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  new_mag_data = 0;
  new_accel_data = 0;
  new_gyro_data = 0;

  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    new_accel_data = 1;
  }
  val[0] = (ax - acc_off[0]/gravConst) * acc_scale[0] ;
  val[1] = -(ay - acc_off[1]/gravConst) * acc_scale[1] ;
  val[2] = -(az - acc_off[2]/gravConst) * acc_scale[2] ;

  if(IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    new_gyro_data = 1;
  }
  val[3] = gx * deg2rad;
  val[4] = -gy * deg2rad;
  val[5] = -gz * deg2rad;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    new_mag_data = 1;
  }
  val[6] = (mx - magn_off[0]) * magn_scale[0];
  val[7] = (my - magn_off[1]) * magn_scale[1];
  val[8] = (mz - magn_off[2]) * magn_scale[2];

  float norm_mag = sqrt(val[6]*val[6] + val[7]*val[7] + val[8]*val[8]);
  val[6] /= norm_mag;
  val[7] /= norm_mag;
  val[8] /= norm_mag;

  if(new_accel_data && new_gyro_data) {
    now1= micros(); // This is when the data reported READY
  //  Serial.printf(" %f, %f, %f, %f, %f, %f, %f, %f, %f\n", val[0], val[1], 
  //              val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
 }

}

void gyroCalibration() {
  gyrox_off = 0;
  gyroy_off = 0;
  gyroz_off = 0;
}

#endif

void getIMU(){ 
  // read the sensor
  getCalIMU();
  // display the data
#if defined(mpu9250)
  if(newIMUData) {
#elif defined(m5stack)
  if(new_accel_data && new_gyro_data) {
#endif
    // update the filter
    gx1 = val[3]; gy1 = val[4]; gz1 = val[5];
    ax1 = val[0]; ay1 = val[1]; az1 = val[2];
    mx1 = val[6]; my1 = val[7]; mz1 = val[8];  
    //Serial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1);
    _dt = ((now1 - lastUpdate) * 0.000001);   //micros converted to seconds
    //Serial.println(_dt, 6);
    #if(MARG == 0 || MARG == 1 || MARG == 2 || MARG == 3)  
      #if (not defined(DISABLE_MAGN))
        #if MARG == 0
            AHRSupdate(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1, _dt);
        #elif MARG == 1
            MadgwickAHRSupdate(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1, _dt);        
        #elif MARG == 2
            //MahonyAHRSupdate(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1);
        #else
            MARGUpdateFilter(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1, _dt);
        #endif
      #else
        #if MARG == 0
            AHRSupdateIMU(gx1, gy1, gz1, ax1, ay1, az1, _dt);
        #elif MARG == 1
            MadgwickAHRSupdateIMU(gx1, gy1, gz1, ax1, ay1, az1, _dt);
        #elif MARG == 3
            MARGUpdateFilterIMU(gx1, gy1, gz1, ax1, ay1, az1, _dt);
        #endif
      #endif
    #elif (MARG == 4)
        //val[9] = maghead.iheading(1, 0, 0, val[0], val[1], val[2], val[6], val[7], val[8]);
        float magHead = calcMagHeading(quant[0], quant[1], quant[2], quant[3], mx1, my1, mz1);
        dcm.setSensorVals(gx1*rad2deg, gy1*rad2deg, gz1*rad2deg, ax1, ay1, az1, mx1*51.02, my1*51.02, mz1*51.02, magHead);
        dcm.G_Dt = _dt;
        dcm.calDCM();
        dcm.getDCM2Q(quant);
    #elif (MARG == 5)
        //compFilter(gx1, gy1, gz1, ax1, ay1, az1, 0, 0, 0);    //yaw based on filter alone
        compFilter(gx1, gy1, gz1, ax1, ay1, az1, mx1, my1, mz1, _dt);//yaw calculated with magnetometer
    #elif MARG == 6
      Filter.update(gx1*deg2rad, gy1*deg2rad, gz1*deg2rad, ax1*gravConst, ay1*gravConst, az1*gravConst, mx1, my1, mz1, _dt);
    #elif MARG == 7
  /*
   * Note that this library only provides the algorithm to perform the Direction
   * Cosine Matrix calculations. This algorithm estimates with drift correction
   * the pitch, roll and yaw (also known as Euler Angles).
   * It has to be feed with data read from a gyroscope, magnetometer and accelerometer,
   * with the following units:
   *    -Gyroscopes:    Deg/s
   *    -Magnetometer:  Gauss
   *    -Accelerometer: G
   */
        dcmAlgorithm.update(_dt, gx1, gy1*rad2deg, gz1*rad2deg, ax1, ay1, az1, mx1*51.02, my1*51.02, mz1*51.02);
    #elif MARG == 8
        filter.update(gy1*rad2deg, gx1*rad2deg, -gz1*rad2deg, ay1, ax1, -az1, my1, mx1, -mz1); 
    #endif
    lastUpdate = micros();
  }
}
