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

//Min - max
float acc_off[] = {-0.151651, -0.151651, 0.073431};
float acc_scale[]  = {1.000082, 0.997899, 0.992887};
float magn_off[]  = {0.960785, -22.553501, 31.364185};
float magn_scale[]  = {1.079792, 1.053037, 0.990540};


