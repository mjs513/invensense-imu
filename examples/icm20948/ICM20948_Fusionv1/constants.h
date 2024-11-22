#define cout SerialUSB1

//elapsedMillis dump;

float SAMPLE_RATE; // replace this with actual sample RATE in Hz
float g = 9.80665;
float rads2degs = 57.296;


float fHeading = 0;

float val[10];
int16_t raw_val[10];

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

//Freeimu calibration
int cal_acc_off[] = {13, 43, 74};
int cal_acc_scale[]  = {2030, 2069, 2090};
int cal_magn_off[]  = {73, 168, -190};
int cal_magn_scale[]  = {226, 250, 248};

//Min - max
float acc_off[] = {-0.148610, -0.148610, 0.053655};
float acc_scale[]  = {1.000679, 0.999199,  0.988149};
float magn_off[]  = {0.975862, -21.344090, 31.705265};
float magn_scale[]  = {1.037051, 1.014182, 1.010868};

//Magneto 1.2 Calibration 
float MagOffset[3] = {13.090573, 30.546150, -32.757856}; 
float mCal[3][3] = 
{
  {1.016476, -0.011290, 0.00759},
  {-0.011290, 1.004871, 0.062744},
  {0.00759, 0.062744, 1.032701}
};

//Motion cal
float MagOffset1[3] = {0.645,-21.162,30.333}; 
float mCal1[3][3] = 
{
  0.9782,0.0085,0.0045},
  {0.0085,1.0210,0.0103},
  {0.0045,0.0103,1.0014}
};
