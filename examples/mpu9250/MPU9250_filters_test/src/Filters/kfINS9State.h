#ifndef _KFINS9STATE_H_
#define _KFINS9STATE_H_


//XXXXXXXXXXXXXXXXX  EIGEN SETUP FOR KF  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// state vectors
Eigen::Matrix<double, 9, 1> xk = Eigen::Matrix<double, 9, 1>::Zero();
Eigen::Matrix<double, 9, 1> xk1 = Eigen::Matrix<double, 9, 1>::Zero();
Eigen::Matrix<double, 9, 1> xk1Minus = Eigen::Matrix<double, 9, 1>::Zero();
// temp vectors
Eigen::Matrix<double, 3, 1> xECEF = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> RefLLA = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> accECEFk = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> accECEFk1 = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> accNEDk1 = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> velNEDk1 = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> velECEFk1 = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 3, 1> posECEFk1 = Eigen::Matrix<double, 3, 1>::Zero();
// state transition and observability matrices
Eigen::Matrix<double, 9, 9> fMatrix = Eigen::Matrix<double, 9, 9>::Zero();
Eigen::Matrix<double, 9, 3> gMatrix = Eigen::Matrix<double, 9, 3>::Zero();
Eigen::Matrix<double, 6, 9> hMatrix = Eigen::Matrix<double, 6, 9>::Zero();
// state covariance matrices
Eigen::Matrix<double, 9, 9> pk = Eigen::Matrix<double, 9, 9>::Zero();
Eigen::Matrix<double, 9, 9> pk1 = Eigen::Matrix<double, 9, 9>::Zero();
Eigen::Matrix<double, 9, 9> pk1Minus = Eigen::Matrix<double, 9, 9>::Zero();
// control and measurement vectors
Eigen::Matrix<double, 6, 1> zk1 = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::Matrix<double, 3, 1> uk1 = Eigen::Matrix<double, 3, 1>::Zero();
// noise covariance matrices
Eigen::Matrix<double, 9, 9> qMatrix = Eigen::Matrix<double, 9, 9>::Zero();
Eigen::Matrix<double, 6, 6> rMatrix = Eigen::Matrix<double, 6, 6>::Zero();
// kalman gain and innovation matrices
Eigen::Matrix<double, 9, 6> kk1 = Eigen::Matrix<double, 9, 6>::Zero();
Eigen::Matrix<double, 6, 1> nuk1 = Eigen::Matrix<double, 6, 1>::Zero();

// Kalman filter parameters
// Initial error levels for covariance, P
double initPosnVar = 10.0;
double initVelVar  = 5.0;
double initAccVar  = 0.5;

// Initial noise error levels for Q
int qModel = 2;                   // Select 1,2, or 3
double qAccSigma  = .05;          // Model 1 noise
double qAccPwrSpecDensity  = 5.0; // Model 2 noise
double qJerkSigma  = 0.05;         // Model 3 noise
// Initial noise error levels for R
double rPosnSigma = 5.0;
double rVelSigma  = 0.5;
// Define flag that tells kfReset routine if kf update was done
bool updateFlag = false;
bool kfInitFlag = false;

// Initialization of parameters for IMU data buffering and lineFit
// Note: Must set row length in aMatrices, xArray, and yArray to be
// equal to numPoints
bool enableLineFitSmoothing = false;
int numPoints = 20; // number of points to be smoothed in lineFit
bool lineFitFlag = false;
bool imuBufferFull = false;
int imuBufferCount = 0;
int lineFitOrder = 2;
float R2;
bool lineFitFlagArray[3];
Eigen::Matrix<double, 20, 4> imuBuffer = Eigen::Matrix<double, 20, 4>::Zero();
Eigen::Matrix<double, 3, 1> r2Array = Eigen::Matrix<double, 3, 1>::Zero();
Eigen::Matrix<double, 20, 1> xArray = Eigen::Matrix<double, 20, 1>::Zero();
Eigen::Matrix<double, 20, 1> yArray = Eigen::Matrix<double, 20, 1>::Zero();
Eigen::Matrix<double, 20, 2> a1Matrix = Eigen::Matrix<double, 20, 2>::Ones();
Eigen::Matrix<double, 20, 3> a2Matrix = Eigen::Matrix<double, 20, 3>::Ones();
Eigen::Matrix<double, 2, 1> c1Array = Eigen::Matrix<double, 2, 1>::Zero();
Eigen::Matrix<double, 3, 1> c2Array = Eigen::Matrix<double, 3, 1>::Zero();




///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//
// lineFit and loadIMUBuffer routines, lineFit.ino
///////////////////////////////////////////////////////////////////////////////////////

void loadIMUBuffer() {
  // This routine builds a numPoints (10) row buffer of the last numPoints
  // of accel values. There are four columns, time and the three accel
  // values.

  // Load time and accel data into imuBuffer array
  // First move all data down one row, effectively purging
  // the oldest row of data. Start from top.
  // May be able to do this faster with eigen blocks
  for (int ii = 0; ii <= numPoints - 2; ii++) {
    imuBuffer(ii, 0) = imuBuffer(ii + 1, 0);
    imuBuffer(ii, 1) = imuBuffer(ii + 1, 1);
    imuBuffer(ii, 2) = imuBuffer(ii + 1, 2);
    imuBuffer(ii, 3) = imuBuffer(ii + 1, 3);
  }

  // Load in new data to last row
  imuBuffer(numPoints - 1, 0) = now * 0.000001;
  imuBuffer(numPoints - 1, 1) = ax1;
  imuBuffer(numPoints - 1, 2) = ay1;
  imuBuffer(numPoints - 1, 3) = az1;

  imuBufferCount += 1;
  if (imuBufferCount == numPoints) {
    imuBufferFull = true;
    //Serial.print("IMU Buffer Loaded");
  }
}

void lineFit() {

  // Define temp variables for R2 calculation
  Eigen::Matrix<double, 1, 1> temp11 = Eigen::Matrix<double, 1, 1>::Zero();
  Eigen::Matrix<double, 1, 1> temp12 = Eigen::Matrix<double, 1, 1>::Zero();
  Eigen::Matrix<double, 20, 1> temp13 = Eigen::Matrix<double, 20, 1>::Zero();
  Eigen::Matrix<double, 1, 1> temp14 = Eigen::Matrix<double, 1, 1>::Zero();
  //Eigen::Matrix<double,1,1> temp15 = Eigen::Matrix<double,1,1>::Zero();

  // Create the A matrix

  if (lineFitOrder == 1) {
    a1Matrix.block(0, 1, numPoints, 1) = xArray;
  }
  if (lineFitOrder == 2) {
    a2Matrix.block(0, 1, numPoints, 1) = xArray;
    for (int j = 0; j < numPoints; j++) {
      a2Matrix(j, 2) = powf(xArray(j, 0), 2.0);
    }
  }

  // Solve for c array
  if (lineFitOrder == 1) {
    c1Array = (a1Matrix.transpose() * a1Matrix).inverse() * a1Matrix.transpose() * yArray;
  }
  if (lineFitOrder == 2) {
    c2Array = (a2Matrix.transpose() * a2Matrix).inverse() * a2Matrix.transpose() * yArray;
  }

  // Solve for R2
  if (lineFitOrder == 1) {
    // Calculate intermediate terms
    temp11(0, 0) = (yArray - a1Matrix * c1Array).norm();
    temp12(0, 0) = yArray.mean();
    for (int j = 0; j < numPoints; j++) {
      temp13(j, 0) = yArray(j, 0) - temp12(0, 0);
    }
    temp14(0, 0) = temp13.norm();
    // Calculate R2
    R2 = 1.0 - temp11(0, 0) / temp14(0, 0);
  }

  if (lineFitOrder == 2) {
    // Calculate intermediate terms
    temp11(0, 0) = (yArray - a2Matrix * c2Array).norm();
    temp12(0, 0) = yArray.mean();
    for (int j = 0; j < numPoints; j++) {
      temp13(j, 0) = yArray(j, 0) - temp12(0, 0);
    }
    temp14(0, 0) = temp13.norm();
    // Calculate R2
    R2 = 1.0 - temp11(0, 0) / temp14(0, 0);
  }

  // Set lineFitFlag to true if fit is acceptable
  if (R2 > 0.75) {
    lineFitFlag = true;
  } else {
    lineFitFlag = false;
  }
}


void getSmoothedAccel() {
  // This routine calls lineFit three times to get the three smoothed
  // accel values. It smooths the accel data stored in the imuBuffer
  // variable. The smoothed result is timetagged to the final time
  // per column 1 of the imuBuffer, but selectable time is
  // implementable.

  // Get smoothed value for ax1.

  // Step 1. Assign xArray (time) and yArray (data) relating to ax1.
  // Place imuBuffer time values into the time (x) array.
  // Normalize (zero-set) the time variable x
  xArray = imuBuffer.block(0, 0, numPoints, 1);
  double timeOffset = imuBuffer(0, 0);
  for (int k = 0; k < numPoints; k++) {
    xArray(k, 0) = xArray(k, 0) - timeOffset;
  }

  // Place imuBuffer data into the data (y) array.
  yArray = imuBuffer.block(0, 1, numPoints, 1);

  // Step 2. Call lineFit to get cArray and R2.
  lineFit();

  // Step 3. Use cArray to get ax1Smoothed.
  // Solve for estimate of data at the end time point.
  if (lineFitOrder == 1) {
    ax1Smooth = c1Array(0, 0) + c1Array(1, 0) * xArray(numPoints - 1, 0);
  }
  if (lineFitOrder == 2) {
    ax1Smooth = c2Array(0, 0) + c2Array(1, 0) * xArray(numPoints - 1, 0) +
                c2Array(2, 0) * powf(xArray(numPoints - 1, 0), 2.0);
  }

  // Step 4. Load R2 into r2Array.
  r2Array(0, 0) = R2;
  lineFitFlagArray[0] = lineFitFlag;

  // Get smoothed value for ay1.

  // Step 1. Assign yArray relating to ay1.
  // Place imuBuffer data into the data (y) array.
  yArray = imuBuffer.block(0, 2, numPoints, 1);

  // Step 2. Call lineFit to get cArray and R2.
  lineFit();

  // Step 3. Use cArray to get ax1Smoothed.
  // Solve for estimate of data at the end time point.
  if (lineFitOrder == 1) {
    ay1Smooth = c1Array(0, 0) + c1Array(1, 0) * xArray(numPoints - 1, 0);
  }
  if (lineFitOrder == 2) {
    ay1Smooth = c2Array(0, 0) + c2Array(1, 0) * xArray(numPoints - 1, 0) +
                c2Array(2, 0) * powf(xArray(numPoints - 1, 0), 2.0);
  }

  // Step 4. Load R2 into r2Array.
  r2Array(1, 0) = R2;
  lineFitFlagArray[1] = lineFitFlag;

  // Get smoothed value for az1.

  // Step 1. Assign yArray relating to az1.
  // Place imuBuffer data into the data (y) array.
  yArray = imuBuffer.block(0, 3, numPoints, 1);

  // Step 2. Call lineFit to get cArray and R2.
  lineFit();

  // Step 3. Use cArray to get ax1Smoothed.
  // Solve for estimate of data at the end time point.
  if (lineFitOrder == 1) {
    az1Smooth = c1Array(0, 0) + c1Array(1, 0) * xArray(numPoints - 1, 0);
  }
  if (lineFitOrder == 2) {
    az1Smooth = c2Array(0, 0) + c2Array(1, 0) * xArray(numPoints - 1, 0) +
                c2Array(2, 0) * powf(xArray(numPoints - 1, 0), 2.0);
  }

  // Step 4. Load R2 into r2Array.
  r2Array(2, 0) = R2;
  lineFitFlagArray[2] = lineFitFlag;

} // End





//////////////////////////////////////////////////////////////////////
//
// Routines to Convert NED-Body to NED-Inertial, GetACCNEDInertial.ino
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//
// Direction Cosine Matrix routine
//
// DCM direction definition:
//    aNED = dcm * aBody
//    aBody = dcm.transpose() * aNED
//
// NED reference frames assumed for both body and inertial.
//
//////////////////////////////////////////////////////////////////////

void body2inertial() {

//#include "Eigen.h"
//#include <Eigen/Dense>

  Eigen::Matrix<float, 3, 1> aBody = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1>  aNED = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 3>   dcm = Eigen::Matrix<float, 3, 3>::Zero();

  float rR, pR, yR;

// Load in accel and orientation angles
//const float gConstant = -9.80665;  // Gravity, m/s^2

  if (enableLineFitSmoothing) {
    aBody(0, 0) = ax1Smooth;
    aBody(1, 0) = ay1Smooth;
    aBody(2, 0) = az1Smooth;
  } else {
    aBody(0, 0) = ax1;
    aBody(1, 0) = ay1;
    aBody(2, 0) = az1;
  }
  pR = ypr[1] * deg2rad;
  rR = ypr[2] * deg2rad;
  yR = ypr[0] * deg2rad;

//Define DCM
  dcm(0, 0) =  cos(pR) * cos(yR);
  dcm(0, 1) =  cos(pR) * sin(yR);
  dcm(0, 2) = -sin(pR);

  dcm(1, 0) = -cos(rR) * sin(yR) + sin(rR) * sin(pR) * cos(yR) ;
  dcm(1, 1) =  cos(rR) * cos(yR) + sin(rR) * sin(pR) * sin(yR);
  dcm(1, 2) =  sin(rR) * cos(pR);

  dcm(2, 0) =  sin(rR) * sin(yR) + cos(rR) * sin(pR) * cos(yR);
  dcm(2, 1) = -sin(rR) * cos(yR) + cos(rR) * sin(pR) * sin(yR) ;
  dcm(2, 2) =  cos(rR) * cos(pR);

// Perform transformation
  aNED = dcm.transpose() * aBody;

// Assign accel NED elements to global variables
  aNEDx1 = aNED(0, 0);
  aNEDy1 = aNED(1, 0);
  aNEDz1 = aNED(2, 0);

// Remove gravity from Z component
  aNEDz1 += gravConst;
}




///////////////////////////////////////////////////////////////////////////////////////
//
// Coordinate Transformation Functions
///////////////////////////////////////////////////////////////////////////////////////

/*
cTrans.h

Transformation Functions:
  sk: gives a skew symmetric matrix from a given vector w
  llarate: rate of change of latitude, longitude, and altitude
  lla2ecef: LLA to ECEF
  ecef2lla: ECEF to LLA
  body2ned: IMU body to NED inertial
  ecef2ned: ECEF to NED
  quat2dcm: quaternions to DCM
  ned2ecef: NED to ECEF
  qmult: quaternion multiply
  constrainAngle180: constrains angle to +- 180 degrees
  constrainAngle360: constrains angle from 0 to 360 degrees

Original Author:

Brian R Taylor
brian.taylor@bolderflight.com
2017-12-20
Bolder Flight Systems
Copyright 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



// acceleration due to gravity
const double gravityConst = 9.80665f;
// major eccentricity squared
const double ECC2 = 0.0066943799901;
// earth semi-major axis radius (m)
const double EARTH_RADIUS = 6378137.0;

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix<double, 3, 3> sk(Eigen::Matrix<double, 3, 1> w) {
  Eigen::Matrix<double, 3, 3> C;
  C(0, 0) = 0.0f;    C(0, 1) = -w(2, 0); C(0, 2) = w(1, 0);
  C(1, 0) = w(2, 0);  C(1, 1) = 0.0f;    C(1, 2) = -w(0, 0);
  C(2, 0) = -w(1, 0); C(2, 1) = w(0, 0);  C(2, 2) = 0.0f;
  return C;
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double, 3, 1> llarate(Eigen::Matrix<double, 3, 1> V, Eigen::Matrix<double, 3, 1> lla) {
  double Rew, Rns, denom;
  Eigen::Matrix<double, 3, 1> lla_dot;

  denom = (1.0 - (ECC2 * pow(sin(lla(0, 0)), 2.0)));
  denom = sqrt(denom * denom);

  Rew = EARTH_RADIUS / sqrt(denom);
  Rns = EARTH_RADIUS * (1.0 - ECC2) / denom * sqrt(denom);

  lla_dot(0, 0) = V(0, 0) / (Rns + lla(2, 0));
  lla_dot(1, 0) = V(1, 0) / ((Rew + lla(2, 0)) * cos(lla(0, 0)));
  lla_dot(2, 0) = -V(2, 0);

  return lla_dot;
}

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
Eigen::Matrix<double, 3, 1> lla2ecef(Eigen::Matrix<double, 3, 1> lla) {
  double Rew, denom;
  Eigen::Matrix<double, 3, 1> ecef;

  denom = (1.0 - (ECC2 * pow(sin(lla(0, 0)), 2.0)));
  denom = sqrt(denom * denom);

  Rew = EARTH_RADIUS / sqrt(denom);

  ecef(0, 0) = (Rew + lla(2, 0)) * cos(lla(0, 0)) * cos(lla(1, 0));
  ecef(1, 0) = (Rew + lla(2, 0)) * cos(lla(0, 0)) * sin(lla(1, 0));
  ecef(2, 0) = (Rew * (1.0 - ECC2) + lla(2, 0)) * sin(lla(0, 0));

  return ecef;
}

// This function calculates the Latitude, Longitude and Altitude given ECEF Coordinates.
Eigen::Matrix<double, 3, 1> ecef2lla(Eigen::Matrix<double, 3, 1> ecef) {

  double a, f, b, e, ePrime, n, lat, lon, alt, p, theta, x, y, z;
  Eigen::Matrix<double, 3, 1> lla;

  // WGS84 parameters
  //a = 6378137.0;
  a = EARTH_RADIUS;
  f = 1. / 298.257223563;
  b = a * (1. - f);
  e = sqrt( (a * a - b * b) / (a * a) );
  ePrime = sqrt( (a * a - b * b) / (b * b) );

  // local variables
  x = ecef(0, 0);
  y = ecef(1, 0);
  z = ecef(2, 0);

  // calculations
  lon = atan2(y, x);
  p = sqrt(x * x + y * y);
  theta = atan2(a * z, b * p);
  lat = atan2((z + ePrime * ePrime * b * pow(sin(theta), 3.0)), \
              (p - e * e * a * pow(cos(theta), 3.0)));
  n = a / (sqrt(1. - e * e * pow(sin(lat), 2.0)));
  alt = (p / (cos(lat))) - n;

  // correct for numerical instability near poles
  //k = abs(x)<1 & abs(y)<1;
  //alt(k) = abs(z(k))-b;

  if (lon > PI) {
    lon = lon - 2 * PI;
  }

  lla(0, 0) = lat;
  lla(1, 0) = lon;
  lla(2, 0) = alt;

  return lla;
}

// This function calculates the NED vector given body vector.
//   ypr is a vector of yaw, pitch, and roll in radians
Eigen::Matrix<double, 3, 1> body2ned(Eigen::Matrix<double, 3, 1> ypr, Eigen::Matrix<double, 3, 1> ecef) {

  double pR, rR, yR;
  Eigen::Matrix<double, 3, 1> ned;
  Eigen::Matrix<double, 3, 3> dcm;

  // temp variables
  pR = ypr(1, 0);
  rR = ypr(2, 0);
  yR = ypr(0, 0);

  // define DCM
  dcm(0, 0) =  cos(pR) * cos(yR);
  dcm(0, 1) =  cos(pR) * sin(yR);
  dcm(0, 2) = -sin(pR);

  dcm(1, 0) = -cos(rR) * sin(yR) + sin(rR) * sin(pR) * cos(yR) ;
  dcm(1, 1) =  cos(rR) * cos(yR) + sin(rR) * sin(pR) * sin(yR);
  dcm(1, 2) =  sin(rR) * cos(pR);

  dcm(2, 0) =  sin(rR) * sin(yR) + cos(rR) * sin(pR) * cos(yR);
  dcm(2, 1) = -sin(rR) * cos(yR) + cos(rR) * sin(pR) * sin(yR) ;
  dcm(2, 2) =  cos(rR) * cos(pR);

  // Perform transformation
  ned = dcm.transpose() * ecef;

  // Remove gravity from Z component
  ned(2, 0) = ned(2, 0) + gravityConst;

  return ned;
}

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
Eigen::Matrix<double, 3, 1> ecef2ned(Eigen::Matrix<double, 3, 1> ecef, Eigen::Matrix<double, 3, 1> pos_ref) {
  Eigen::Matrix<double, 3, 1> ned;
  ned(2, 0) = -cos(pos_ref(0, 0)) * cos(pos_ref(1, 0)) * ecef(0, 0) - cos(pos_ref(0, 0)) * sin(pos_ref(1, 0)) * ecef(1, 0) - sin(pos_ref(0, 0)) * ecef(2, 0);
  ned(1, 0) = -sin(pos_ref(1, 0)) * ecef(0, 0) + cos(pos_ref(1, 0)) * ecef(1, 0);
  ned(0, 0) = -sin(pos_ref(0, 0)) * cos(pos_ref(1, 0)) * ecef(0, 0) - sin(pos_ref(0, 0)) * sin(pos_ref(1, 0)) * ecef(1, 0) + cos(pos_ref(0, 0)) * ecef(2, 0);
  return ned;
}

// quaternion to dcm
Eigen::Matrix<double, 3, 3> quat2dcm(Eigen::Matrix<double, 4, 1> q) {
  Eigen::Matrix<double, 3, 3> C_N2B;
  C_N2B(0, 0) = 2.0f * powf(q(0, 0), 2.0f) - 1.0f + 2.0f * powf(q(1, 0), 2.0f);
  C_N2B(1, 1) = 2.0f * powf(q(0, 0), 2.0f) - 1.0f + 2.0f * powf(q(2, 0), 2.0f);
  C_N2B(2, 2) = 2.0f * powf(q(0, 0), 2.0f) - 1.0f + 2.0f * powf(q(3, 0), 2.0f);

  C_N2B(0, 1) = 2.0f * q(1, 0) * q(2, 0) + 2.0f * q(0, 0) * q(3, 0);
  C_N2B(0, 2) = 2.0f * q(1, 0) * q(3, 0) - 2.0f * q(0, 0) * q(2, 0);

  C_N2B(1, 0) = 2.0f * q(1, 0) * q(2, 0) - 2.0f * q(0, 0) * q(3, 0);
  C_N2B(1, 2) = 2.0f * q(2, 0) * q(3, 0) + 2.0f * q(0, 0) * q(1, 0);

  C_N2B(2, 0) = 2.0f * q(1, 0) * q(3, 0) + 2.0f * q(0, 0) * q(2, 0);
  C_N2B(2, 1) = 2.0f * q(2, 0) * q(3, 0) - 2.0f * q(0, 0) * q(1, 0);
  return C_N2B;
}

// ned to ecef inertial
//   ned is the input NED vector
//   refLLA is vector of lat (rad), lon (rad), alt (m above ellipsoid)
Eigen::Matrix<double, 3, 1> ned2ecef(Eigen::Matrix<double, 3, 1> refLLA, Eigen::Matrix<double, 3, 1> ned) {

  Eigen::Matrix<double, 3, 3> C_N2I;
  Eigen::Matrix<double, 3, 1> ecef;
  double clat, slat, clon, slon;

  // Temporary variables
  clat = cos(refLLA(0, 0));
  slat = sin(refLLA(0, 0));
  clon = cos(refLLA(1, 0));
  slon = sin(refLLA(1, 0));

  C_N2I(0, 0) = -clon * slat;
  C_N2I(0, 1) = -slon;
  C_N2I(0, 2) =  clon * clat;

  C_N2I(1, 0) = -slon * slat;
  C_N2I(1, 1) =  clon;
  C_N2I(1, 2) = -slon * clat;

  C_N2I(2, 0) =  clat;
  C_N2I(2, 1) =  0.0f;
  C_N2I(2, 2) = -slat;

  ecef = C_N2I * ned;
  return ecef;
}

// quaternion multiplication
Eigen::Matrix<double, 4, 1> qmult(Eigen::Matrix<double, 4, 1> p, Eigen::Matrix<double, 4, 1> q) {
  Eigen::Matrix<double, 4, 1> r;
  r(0, 0) = p(0, 0) * q(0, 0) - (p(1, 0) * q(1, 0) + p(2, 0) * q(2, 0) + p(3, 0) * q(3, 0));
  r(1, 0) = p(0, 0) * q(1, 0) + q(0, 0) * p(1, 0) + p(2, 0) * q(3, 0) - p(3, 0) * q(2, 0);
  r(2, 0) = p(0, 0) * q(2, 0) + q(0, 0) * p(2, 0) + p(3, 0) * q(1, 0) - p(1, 0) * q(3, 0);
  r(3, 0) = p(0, 0) * q(3, 0) + q(0, 0) * p(3, 0) + p(1, 0) * q(2, 0) - p(2, 0) * q(1, 0);
  return r;
}

// bound yaw angle between -180 and 180
double constrainAngle180(double dta) {
  if (dta >  PI) dta -= (PI * 2.0f);
  if (dta < -PI) dta += (PI * 2.0f);
  return dta;
}

// bound heading angle between 0 and 360
double constrainAngle360(double dta) {
  dta = fmod(dta, 2.0f * PI);
  if (dta < 0)
    dta += 2.0f * PI;
  return dta;
}



///////////////////////////////////////////////////////////////////////////////////////
// kfProcessNoise.ino
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
//
// Kalman filter Process Noise models
//  Reference: Labbe's Kalman Filter Book
///////////////////////////////////////////////////////////////////////////////////////

void calculateQ() {

  if (qModel == 1) {
    // Model 1. Modeling noise directly in accel terms only
    qMatrix(6, 6) = pow(qAccSigma, 2.0);
    qMatrix(7, 7) = pow(qAccSigma, 2.0);
    qMatrix(8, 8) = pow(qAccSigma, 2.0);
  }

  if (qModel == 2) {
    // Model 2. Power spectral noise model
    qMatrix(0, 0) = (pow(_dt, 5.0)) / 20.0;
    qMatrix(0, 3) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(0, 6) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(3, 0) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(3, 3) = (pow(_dt, 3.0)) / 3.0;
    qMatrix(3, 6) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(6, 0) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(6, 3) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(6, 6) = _dt;

    qMatrix(1, 1) = (pow(_dt, 5.0)) / 20.0;
    qMatrix(1, 4) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(1, 7) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(4, 1) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(4, 4) = (pow(_dt, 3.0)) / 3.0;
    qMatrix(4, 7) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(7, 1) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(7, 4) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(7, 7) = _dt;

    qMatrix(2, 2) = (pow(_dt, 5.0)) / 20.0;
    qMatrix(2, 5) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(2, 8) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(5, 2) = (pow(_dt, 4.0)) / 8.0;
    qMatrix(5, 5) = (pow(_dt, 3.0)) / 3.0;
    qMatrix(5, 8) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(8, 2) = (pow(_dt, 3.0)) / 6.0;
    qMatrix(8, 5) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(8, 8) = _dt;

    // Multiply by power spectral density
    qMatrix = qMatrix * qAccPwrSpecDensity;
  }

  if (qModel == 3) {
    // Model 3. Jerk noise model
    qMatrix(0, 0) = (pow(_dt, 4.0)) / 4.0;
    qMatrix(0, 3) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(0, 6) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(3, 0) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(3, 3) = (pow(_dt, 2.0));
    qMatrix(3, 6) = _dt;
    qMatrix(6, 0) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(6, 3) = _dt;
    qMatrix(6, 6) = 1.0;

    qMatrix(1, 1) = (pow(_dt, 4.0)) / 4.0;
    qMatrix(1, 4) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(1, 7) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(4, 1) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(4, 4) = (pow(_dt, 2.0));
    qMatrix(4, 7) = _dt;
    qMatrix(7, 1) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(7, 4) = _dt;
    qMatrix(7, 7) = 1.0;

    qMatrix(2, 2) = (pow(_dt, 4.0)) / 4.0;
    qMatrix(2, 5) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(2, 8) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(5, 2) = (pow(_dt, 3.0)) / 2.0;
    qMatrix(5, 5) = (pow(_dt, 2.0));
    qMatrix(5, 8) = _dt;
    qMatrix(8, 2) = (pow(_dt, 2.0)) / 2.0;
    qMatrix(8, 5) = _dt;
    qMatrix(8, 8) = 1.0;

    // Multiply by jerk variance
    qMatrix = qMatrix * pow(qJerkSigma, 2.0);
  }

} // end void




///////////////////////////////////////////////////////////////////////////////////////
// kfINS9State.ino
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//
// Kalman filter INS routines
///////////////////////////////////////////////////////////////////////////////////////

// Kalman filter initialization
void kfInit() {

  // Initialize R noise covariance
  rMatrix(0, 0) = pow(rPosnSigma, 2.0);
  rMatrix(1, 1) = pow(rPosnSigma, 2.0);
  rMatrix(2, 2) = pow(rPosnSigma, 2.0);
  rMatrix(3, 3) = pow(rVelSigma, 2.0);
  rMatrix(4, 4) = pow(rVelSigma, 2.0);
  rMatrix(5, 5) = pow(rVelSigma, 2.0);

  // Initialize state and covariance, x and P
  // Use first LLA GPS reading to initialize x

  pk(0, 0) = pow(initPosnVar, 2.0);
  pk(1, 1) = pow(initPosnVar, 2.0);
  pk(2, 2) = pow(initPosnVar, 2.0);
  pk(3, 3) = pow(initVelVar, 2.0);
  pk(4, 4) = pow(initVelVar, 2.0);
  pk(5, 5) = pow(initVelVar, 2.0);
  pk(6, 6) = pow(initAccVar, 2.0);
  pk(7, 7) = pow(initAccVar, 2.0);
  pk(8, 8) = pow(initAccVar, 2.0);

  RefLLA(0, 0) = double(rtk.lat1 * deg2rad);
  RefLLA(1, 0) = double(rtk.lon1 * deg2rad);
  RefLLA(2, 0) = double(rtk.hMSL);
  xECEF = lla2ecef(RefLLA);
  xk(0, 0) = xECEF(0, 0);
  xk(1, 0) = xECEF(1, 0);
  xk(2, 0) = xECEF(2, 0);

  // Initialize appropriate H, F, and G elements to identity
  hMatrix(0, 0) = 1.0;
  hMatrix(1, 1) = 1.0;
  hMatrix(2, 2) = 1.0;
  hMatrix(3, 3) = 1.0;
  hMatrix(4, 4) = 1.0;
  hMatrix(5, 5) = 1.0;
  fMatrix = Eigen::Matrix<double, 9, 9>::Identity();
  gMatrix(6, 0) = 1.0;
  gMatrix(7, 1) = 1.0;
  gMatrix(8, 2) = 1.0;

  // Print out initial GPS LLA, filter LLA, filter ECEF
  cout.print("uBlox LLA: ");
  cout.print(rtk.lat1, 7);
  cout.print(",  ");
  cout.print(rtk.lon1, 7);
  cout.print(",  ");
  cout.println(rtk.hMSL, 4);

  cout.print("RefLLA Initial: ");
  cout.print(RefLLA(0, 0)*rad2deg);
  cout.print(",  ");
  cout.print(RefLLA(1, 0)*rad2deg);
  cout.print(",  ");
  cout.println(RefLLA(2, 0));

  cout.print("xk Initial: ");
  cout.print(xk(0, 0));
  cout.print(",  ");
  cout.print(xk(1, 0));
  cout.print(",  ");
  cout.println(xk(2, 0));
  cout.println(" ");
}

// Kalman filter propagation
void kfProp() {

  // Convert IMU accel from body local to NED inertial
  body2inertial();  // Convert body to NED inertial

  // Convert NED inertial to ECEF
  accNEDk1(0, 0) = aNEDx1;
  accNEDk1(1, 0) = aNEDy1;
  accNEDk1(2, 0) = aNEDz1;
  RefLLA(0, 0) = double(rtk.lat1 * deg2rad);
  RefLLA(1, 0) = double(rtk.lon1 * deg2rad);
  RefLLA(2, 0) = double(rtk.hMSL);
  accECEFk1 = ned2ecef(RefLLA, accNEDk1);

  // Use timestep to update F, G, and Q
  fMatrix(0, 3) = _dt;
  fMatrix(1, 4) = _dt;
  fMatrix(2, 5) = _dt;
  fMatrix(3, 6) = _dt;
  fMatrix(4, 7) = _dt;
  fMatrix(5, 8) = _dt;
  fMatrix(0, 6) = 0.5 * _dt * _dt;
  fMatrix(1, 7) = 0.5 * _dt * _dt;
  fMatrix(2, 8) = 0.5 * _dt * _dt;

  gMatrix(0, 0) = 0.5 * _dt * _dt;
  gMatrix(1, 1) = 0.5 * _dt * _dt;
  gMatrix(2, 2) = 0.5 * _dt * _dt;
  gMatrix(3, 0) = _dt;
  gMatrix(4, 1) = _dt;
  gMatrix(5, 2) = _dt;

  calculateQ();

  // Perform Kalman propagation.
  //   Use first line to include IMU accel as control inputs
  //   Use second line to not include IMU accel as control inputs
  // Note that we use the previous accel (time k) to propagate
  // the state to time k+1.
  xk1Minus = fMatrix * xk + gMatrix * accECEFk;
  //xk1Minus = fMatrix * xk;
  pk1Minus = fMatrix * pk * fMatrix.transpose() + qMatrix;

  // reset accel for next kfProp call
  accECEFk = accECEFk1;
}

// Kalman filter measurement update
void kfUpdate() {
  // Calculate Kalman gain
  kk1 = pk1Minus * hMatrix.transpose() * (hMatrix * pk1Minus * hMatrix.transpose() + rMatrix).inverse();

  // read in GPS position and velocity measurements
  RefLLA(0, 0) = double(rtk.lat1 * deg2rad);
  RefLLA(1, 0) = double(rtk.lon1 * deg2rad);
  RefLLA(2, 0) = double(rtk.hMSL);
  velNEDk1(0, 0) = rtk.velN;
  velNEDk1(1, 0) = rtk.velE;
  velNEDk1(2, 0) = rtk.velD;

  // Convert measurements to ECEF
  posECEFk1 = lla2ecef(RefLLA);
  velECEFk1 = ned2ecef(RefLLA, velNEDk1);

  // Load measurements into measurement vector zk1
  zk1(0, 0) = posECEFk1(0, 0);
  zk1(1, 0) = posECEFk1(1, 0);
  zk1(2, 0) = posECEFk1(2, 0);
  zk1(3, 0) = velECEFk1(0, 0);
  zk1(4, 0) = velECEFk1(1, 0);
  zk1(5, 0) = velECEFk1(2, 0);

  // Calculate innovations
  nuk1 = zk1 - hMatrix * xk1Minus;

  // Perform Kalman update
  xk1 = xk1Minus + kk1 * nuk1;
  pk1 = (Eigen::Matrix<double, 9, 9>::Identity() - kk1 * hMatrix) * pk1Minus;

  // Set updateFlag to true so kfReset knows a KF update occurred.
  updateFlag = true;
}

// Reset certain KF terms for next iteration
void kfReset() {
  // if KF update was performed, reset xk and pk to k1 terms
  if (updateFlag) {
    xk = xk1;
    pk = pk1;
    // reset updateFlag to false
    updateFlag = false;
    // else reset xk and pk to k1Minus terms
  } else {
    xk1 = xk1Minus;
    xk = xk1Minus;
    pk1 = pk1Minus;
    pk = pk1;
  }
}



#endif


