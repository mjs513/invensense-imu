// gyro bias estimation
size_t _numSamples = 100;
double _gxbD, _gybD, _gzbD;
float _gxb, _gyb, _gzb;
// accel bias and scale factor estimation
double _axbD, _aybD, _azbD;
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;
float _axb, _ayb, _azb;
float _axs = 1.0f;
float _ays = 1.0f;
float _azs = 1.0f;
// magnetometer bias and scale factor estimation
uint16_t _maxCounts = 1000;
float _deltaThresh = 0.3f;
uint8_t _coeff = 8;
uint16_t _counter;
float _framedelta, _delta;
float _hxfilt, _hyfilt, _hzfilt;
float _hxmax, _hymax, _hzmax;
float _hxmin, _hymin, _hzmin;
float _hxb, _hyb, _hzb;
float _hxs = 1.0f;
float _hys = 1.0f;
float _hzs = 1.0f;
float _avgs;

#define G 9.80665f
    
/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int calibrateAccel() {
  // set the range, bandwidth, and srd
/*  if (Imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_2G) != 1) {
    return -1;
  
  if (Imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ) != 1) {
    return -2;
  
  if (Imu.ConfigSrd(19) != 1) {
    return -3;
  }
*/
  // take samples and find min / max 
  _axbD = 0;
  _aybD = 0;
  _azbD = 0;
  _axs = 1; _ays = 1;  _azs = 1; 
  _axb = 0; _ayb = 0;  _azb = 0; 
  uint16_t icount = 0;
  //for (size_t i=0; i < _numSamples; i++) {
  while(icount < _numSamples) {
    if(Imu.Read()){
      if(Imu.new_imu_data()){
        icount += 1;
        _axbD += (Imu.accel_x_mps2()/_axs + _axb)/((double)_numSamples);
        _aybD += (Imu.accel_y_mps2()/_ays + _ayb)/((double)_numSamples);
        _azbD += (Imu.accel_z_mps2()/_azs + _azb)/((double)_numSamples);
      }
    }
  }

  Serial.printf("%f, %f, %f\n",_axbD, _aybD, _azbD );

  if (_axbD > 9.0f) {
    _axmax = (float)_axbD;
  }
  if (_aybD > 9.0f) {
    _aymax = (float)_aybD;
  }
  if (_azbD > 9.0f) {
    _azmax = (float)_azbD;
  }
  if (_axbD < -9.0f) {
    _axmin = (float)_axbD;
  }
  if (_aybD < -9.0f) {
    _aymin = (float)_aybD;
  }
  if (_azbD < -9.0f) {
    _azmin = (float)_azbD;
  }

  // find bias and scale factor
  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_axmin + _axmax) / 2.0f;
    _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  Serial.printf("Max: %f, %f, %f\n",_axmax,_aymax,_azmax );
  Serial.printf("Max: %f, %f, %f\n",_axmin,_aymin,_azmin );
  Serial.printf("%f, %f, %f\n",_axb,_ayb,_azb );
  // set the range, bandwidth, and srd back to what they were
/*  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  */
  return 1;  
}


/* estimates the gyro biases */
int calibrateGyro() {
  // set the range, bandwidth, and srd
/*  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }
*/
  // take samples and find bias
  _gxbD = 0;
  _gybD = 0;
  _gzbD = 0;
  uint16_t icount = 0;
  //for (size_t i=0; i < _numSamples; i++) {
  while(icount < _numSamples) {
    if(Imu.Read()){
    icount += 1;

      if(Imu.new_imu_data()){
        icount += 1;
        _gxbD += (Imu.gyro_x_radps() + _gxb)/((double)_numSamples);
        _gybD += (Imu.gyro_y_radps() + _gyb)/((double)_numSamples);
        _gzbD += (Imu.gyro_z_radps() + _gzb)/((double)_numSamples);
      }
    }
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;
/*
  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  */
  return 1;
}


/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
#if defined(MPU9250) || defined(ICM20948) || defined(EXTMAG)

int calibrateMag() {
  // set the srd
/*  if (setSrd(19) < 0) {
    return -1;
  }
*/
  // get a starting set of data
  #if defined(HMC5983A)
  float mag_val[3];
  mag.getMagScaled(mag_val);
  _hxmax = -mag_val[0];
  _hxmin = -mag_val[0];
  _hymax = mag_val[1];
  _hymin = mag_val[1];
  _hzmax = -mag_val[2];
  _hzmin = -mag_val[2];

  #elif defined(LIS3MDLA)
  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  mag.getEvent(&event);
  _hxmax = event.magnetic.x;
  _hxmin = event.magnetic.x;
  _hymax = event.magnetic.y;
  _hymin = event.magnetic.y;
  _hzmax = event.magnetic.z;
  _hzmin = event.magnetic.z;

  #elif defined(ICM20948)
  float mag_val[3];
  mag.Read();
  mag_val[0] = mag.mag_x_ut();
  mag_val[1] = mag.mag_y_ut();
  mag_val[02] = mag.mag_z_ut();
  _hxmax = mag_val[0];
  _hxmin = mag_val[0];
  _hymax = mag_val[1];
  _hymin = mag_val[1];
  _hzmax = mag_val[2];
  _hzmin = mag_val[2];

  #else 
  Imu.Read();
  _hxmax = Imu.mag_x_ut();
  _hxmin = Imu.mag_x_ut();
  _hymax = Imu.mag_y_ut();
  _hymin = Imu.mag_y_ut();
  _hzmax = Imu.mag_z_ut();
  _hzmin = Imu.mag_z_ut();
  #endif
  // collect data to find max / min in each channel
  _counter = 0;
  while (_counter < _maxCounts) {
    _delta = 0.0f;
    _framedelta = 0.0f;

    #if defined(ICM20948)
    {
    if(mag.Read()){
        _hxfilt = (_hxfilt*((float)_coeff-1)+(mag.mag_x_ut()/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(mag.mag_y_ut()/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(mag.mag_z_ut()/_hzs+_hzb))/((float)_coeff);

    #elif defined(HMC5983A)
    {
      {
        mag.getMagScaled(mag_val);
        //Serial.println(_counter);
        _hxfilt = (_hxfilt*((float)_coeff-1)+(-mag_val[0]/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(mag_val[1]/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(-mag_val[2]/_hzs+_hzb))/((float)_coeff);
        delay(50);

    #elif defined(LIS3MDLA)
    // Using the Adafruit LIS3mdl and ICM20649 the axis are aligned on the bread board
    // so have to use the same axis rotation for NED coordinates as the ICM20649 in the
    // library
    {
      {
        sensors_event_t event; 
        mag.getEvent(&event);
                //Serial.println(_counter);
        _hxfilt = (_hxfilt*((float)_coeff-1)+(event.magnetic.x/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(event.magnetic.y/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(event.magnetic.z/_hzs+_hzb))/((float)_coeff);
        delay(50);
    #else
    if(Imu.Read()){
      if(Imu.new_imu_data()){
        //Serial.println(_counter);
        _hxfilt = (_hxfilt*((float)_coeff-1)+(Imu.mag_x_ut()/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(Imu.mag_y_ut()/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(Imu.mag_z_ut()/_hzs+_hzb))/((float)_coeff);
    #endif
        if (_hxfilt > _hxmax) {
          _delta = _hxfilt - _hxmax;
          _hxmax = _hxfilt;
        }

        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hyfilt > _hymax) {
          _delta = _hyfilt - _hymax;
          _hymax = _hyfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hzfilt > _hzmax) {
          _delta = _hzfilt - _hzmax;
          _hzmax = _hzfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hxfilt < _hxmin) {
          _delta = abs(_hxfilt - _hxmin);
          _hxmin = _hxfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hyfilt < _hymin) {
          _delta = abs(_hyfilt - _hymin);
          _hymin = _hyfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_hzfilt < _hzmin) {
          _delta = abs(_hzfilt - _hzmin);
          _hzmin = _hzfilt;
        }
        if (_delta > _framedelta) {
          _framedelta = _delta;
        }
        if (_framedelta > _deltaThresh) {
          _counter = 0;
        } else {
          _counter++;
        }
      }
    }
  }

  // find the magnetometer bias
  _hxb = (_hxmax + _hxmin) / 2.0f;
  _hyb = (_hymax + _hymin) / 2.0f;
  _hzb = (_hzmax + _hzmin) / 2.0f;

  // find the magnetometer scale factor
  _hxs = (_hxmax - _hxmin) / 2.0f;
  _hys = (_hymax - _hymin) / 2.0f;
  _hzs = (_hzmax - _hzmin) / 2.0f;
  _avgs = (_hxs + _hys + _hzs) / 3.0f;
  _hxs = _avgs/_hxs;
  _hys = _avgs/_hys;
  _hzs = _avgs/_hzs;

  // set the srd back to what it was
/*  if (setSrd(_srd) < 0) {
    return -2;
  }
  */
  return 1;
}
#else
int calibrateMag() {
  _hxmax = 0;
  _hxmin = 0;
  _hymax = 0;
  _hymin = 0;
  _hzmax = 0;
  _hzmin = 0;
  _hxs = 1;
  _hys = 1;
  _hzs = 1;

  return 1;
}
#endif

/* returns the gyro bias in the X direction, rad/s */
float getGyroBiasX_rads() {
  return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float getGyroBiasY_rads() {
  return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float getGyroBiasZ_rads() {
  return _gzb;
}


/* returns the accelerometer bias in the X direction, m/s/s */
float getAccelBiasX_mss() {
  return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float getAccelScaleFactorX() {
  return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float getAccelBiasY_mss() {
  return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float getAccelScaleFactorY() {
  return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float getAccelBiasZ_mss() {
  return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float getAccelScaleFactorZ() {
  return _azs;
}


/* returns the magnetometer bias in the X direction, uT */
float getMagBiasX_uT() {
  return _hxb;
}

/* returns the magnetometer scale factor in the X direction */
float getMagScaleFactorX() {
  return _hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float getMagBiasY_uT() {
  return _hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float getMagScaleFactorY() {
  return _hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float getMagBiasZ_uT() {
  return _hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float getMagScaleFactorZ() {
  return _hzs;
}
