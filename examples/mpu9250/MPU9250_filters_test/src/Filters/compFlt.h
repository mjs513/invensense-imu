///////////////////////////////////////////////////////////////////////////////////////
// 
// Implementation of Complementary Filter AHRS Algorithm
///////////////////////////////////////////////////////////////////////////////////////

#ifndef COMPFLT_H
#define COMPFLT_H

//
// Based on Joop Brooking complementary filter as modified by Don Kelly
//

double gyro_pitch, gyro_roll, gyro_yaw, acc_total_vector;
double rad2Deg = 180.0f/PI;
double deg2Rad = PI/180.0f;

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float angle_roll_gyro, angle_pitch_gyro, angle_yaw_gyro;
float angle_pitch_yawRot, angle_roll_yawRot;

// Complementary filter parameters
float gainK1 = 0.985f;      // Gain on gyro influence
float gainK2 = 1-gainK1;  // Gain on accel correction to drift

void compFilter(float g_x, float g_y, float g_z, 
                float a_x, float a_y, float a_z, 
                float m_x, float m_y, float m_z, float deltat)
{

  // Integrate pitch and roll by gyro motion                                      
  angle_pitch_gyro = angle_pitch + g_y * rad2Deg * deltat;                                    
  angle_roll_gyro  = angle_roll + g_x * rad2Deg * deltat;                                      
  angle_yaw_gyro  += g_z * rad2Deg * deltat;                                      

  // If IMU has yawed, adjust pitch and roll angles appropriately
  angle_pitch_yawRot = -angle_roll_gyro * sin(g_z * deltat);                 
  angle_roll_yawRot = angle_pitch_gyro * sin(g_z * deltat);                  

  // Calculate accelerometer angle calculations.
  // First calculate the total accelerometer vector
  acc_total_vector = sqrt((a_x*a_x)+(a_y*a_y)+(a_z*a_z));       

  // Prevent the asin functions from producing a NaN, then
  // calculate pitch and roll from accelerometers
  if(abs(a_y) < acc_total_vector){                                        
    angle_pitch_acc = asin((float)a_x/acc_total_vector)* rad2Deg;         
  }
  if(abs(a_x) < acc_total_vector){                                        
    angle_roll_acc = -asin((float)a_y/acc_total_vector)* rad2Deg;          
  }
  
  // Place the MPU spirit level and note the values in the following 
  // two lines for calibration. Replace the zeros with the accel
  // calibration value for pitch.
  angle_pitch_acc -= -0.0047 ;                                                   
  angle_roll_acc -= -1.2637;      
  
  // Complementary filter for pitch and roll. Here we correct the 
  // drift of the gyro angles with the accelerometer angle info.
  // Typical weighting is 90/10, but can vary significantly.
  angle_pitch = gainK1 * (angle_pitch_gyro + angle_pitch_yawRot) 
                + gainK2 * angle_pitch_acc;           
  angle_roll = gainK1 * (angle_roll_gyro + angle_roll_yawRot) 
               + gainK2 * angle_roll_acc;

  // Load pitch and roll into ypr array             
  ypr[1] = angle_pitch;
  ypr[2] = angle_roll;      

  // If there is magnetometer data available, use it to calculate 
  // heading 
  if((m_x != 0.0f) || (m_y != 0.0f) || (m_z != 0.0f)){
    float magx = m_z * sin(ypr[2] * deg2Rad) - 
                 m_y * cos(ypr[2] * deg2Rad);
    float magy = m_x * cos(ypr[1] * deg2Rad) + 
                 m_y * sin(ypr[1] * deg2Rad) * sin(ypr[2] * deg2Rad) + 
                 m_z * sin(ypr[1] * deg2Rad) * cos(ypr[2] * deg2Rad);
    ypr[0] = atan(magx/magy) * rad2Deg;   // May want atan2 here
    
  // If no magnetometer data, load yaw into ypr array        
  } else {
    ypr[0] = angle_yaw_gyro;
  }
}

void getQ(float *quant1, float *quant2, float *quant3, float *quant4)
{
    *quant1 = 1;
    *quant2 = 0;
    *quant3 = 0;
    *quant4 = 0;
}

#endif // end compFilter

