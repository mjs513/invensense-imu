///////////////////////////////////////////////////////////////////////////////////////
// 
// Implementation of Mahony's Original AHRS Algorithm
///////////////////////////////////////////////////////////////////////////////////////

// Implementation of Mahony filter as implemented by Kris Winer, where he references Madgwick as well.
// See http://www.x-io.co.uk/category/open-source/ for examples and more details. Fuses acceleration, 
// rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. 
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering 
// algorithms but is much less computationally intensive. 

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between 
// estimated reference vectors and measured ones. 

// These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional 
// feedback, Ki for integral
#ifndef _MahoneyAHRS_
#define _MahoneyAHRS_

#define Kp 2.0f * 5.0f 
#define Ki 0.0f
// Vector to hold integral error for Mahony method
float eInt[3] = {0.0f, 0.0f, 0.0f};       
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

float invSqrt(float x);

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;   

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * _dt);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * _dt);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * _dt);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * _dt);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm; 
}

void getQ(float *quant1, float *quant2, float *quant3, float *quant4)
{
  *quant1 = q0;
  *quant2 = q1;
  *quant3 = q2;
  *quant4 = q3;
}

// Inverse square root function for MARG Filter (Madgwick and Mahoney)
float invSqrt(float x) {
  int instability_fix = 1;
  if (instability_fix == 0)
  {
    union {
      float f;
      int32_t i;
    } y;

    y.f = x;
    y.i = 0x5f375a86 - (y.i >> 1);
    y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
    return y.f;
  }
  else if (instability_fix == 1)
  {
    /* close-to-optimal  method with low cost from
    http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
    uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
    float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  }
  else
  {
    /* optimal but expensive method: */
    return 1.0f / sqrt(x);
  }
}

#endif