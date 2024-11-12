///////////////////////////////////////////////////////////////////////////////////////
// 
// Mahony Quaternion Implementation of the DCM Filter
///////////////////////////////////////////////////////////////////////////////////////

#ifndef _AHRS_
#define _AHRS_

/**
 * Quaternion implementation of the 'DCM filter' [Ma_yhony et al].  Incorporates the magnetic distortion
 * compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
 * a_xis only.
 * 
 * @see: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/
//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Ma_yhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

  #define twoKpDef  (2.0f * 5.0f) // was 0.95
  #define twoKiDef  (2.0f * 0.0f) // was 0.05 
//-------------------------------------------------------------------------
  volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
  volatile float integralFBx,  integralFBy, integralFBz;
  
  volatile float twoKp = twoKpDef;             // 2 * proportional gain (Kp)
  volatile float twoKi = twoKiDef;             // 2 * integral gain (Ki)
//-----------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
float invSqrt(float x);

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, 
				float mx, float my, float mz, float deltat)
{
  float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;   
        
  if ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) {
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;  
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (my * halfwz - mz * halfwy);
    halfey += (mz * halfwx - mx * halfwz);
    halfez += (mx * halfwy - my * halfwx);
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if( (ax != 0.0f) || (ay != 0.0f) || (az != 0.0f) ) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;     

    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }
        
    //if there is a valid correction vector
    if( (halfex != 0.0f) || (halfey != 0.0f) || (halfez != 0.0f) ) {
      
      // Compute and apply integral feedback if enabled
      if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * (deltat);  // integral error scaled by Ki
        integralFBy += twoKi * halfey * (deltat);
        integralFBz += twoKi * halfez * (deltat);
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
      }
      else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
      }

      // Apply proportional feedback
      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (deltat));   // pre-multiply common factors
  gy *= (0.5f * (deltat));
  gz *= (0.5f * (deltat));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void AHRSupdateIMU(float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float deltat)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
		a_x *= recipNorm;
		a_y *= recipNorm;
		a_z *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (a_y * halfvz - a_z * halfvy);
		halfey = (a_z * halfvx - a_x * halfvz);
		halfez = (a_x * halfvy - a_y * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (deltat);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (deltat);
			integralFBz += twoKi * halfez * (deltat);
			g_x += integralFBx;	// apply integral feedback
			g_y += integralFBy;
			g_z += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		g_x += twoKp * halfex;
		g_y += twoKp * halfey;
		g_z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	g_x *= (0.5f * (deltat));		// pre-multiply common factors
	g_y *= (0.5f * (deltat));
	g_z *= (0.5f * (deltat));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * g_x - qc * g_y - q3 * g_z);
	q1 += (qa * g_x + qc * g_z - q3 * g_y);
	q2 += (qa * g_y - qb * g_z + q3 * g_x);
	q3 += (qa * g_z + qb * g_y - qc * g_x); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
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

//---------------------------------------------------------------------------------------------------

#endif 


