#define cout SerialUSB1

//elapsedMillis dump;

float SAMPLE_RATE; // replace this with actual sample RATE in Hz
float g = 9.80665;
float rads2degs = 57.296;


float fHeading = 0;

float val[10];
int16_t raw_val[10];
float mag_val[3];

uint64_t timestamp;
uint8_t raw_values_on = 0;
uint8_t x_values_on = 0;
uint8_t serial_values_on = 0;
uint8_t telem_values_on = 0;
uint8_t fusion_on = 0;


// gyro bias estimation
float gyrox_off, gyroy_off, gyroz_off;


float mag_norm;

double dt; long t_old;


//Min - max  accel valid for MPU6050
float acc_off[] = {0.011864, 0.011864, 0.133371};
float acc_scale[]  = {1.000154, 1.002913,  1.001943};


//Motion cal valid for HMC5983
float MagOffset1[3] = {0.645,-21.162,30.333}; 
float mCal1[3][3] = 
{
  { 0.9782, 0.0085, 0.0045},
  { 0.0085, 1.0210, 0.0103},
  { 0.0045, 0.0103, 1.0014}
};
