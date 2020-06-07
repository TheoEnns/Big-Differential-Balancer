#include "Arduino.h"

#ifndef REFERENCE_CALCULATOR
#define REFERENCE_CALCULATOR

/* Notes: 
 *    - Kalman filter works on roll but produces bad results for pitch
*/

//#define USE_KALMAN 1 //This code works fine without a kalman filter

#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <math.h>

#ifdef USE_KALMAN
#include <Kalman.h>
#endif

static const float radian_to_degree = 180.0/M_PI;

class TwoWheel_TF_Calc
{
  public:
      // Raw IMU reads
      float aix, aiy, aiz; // raw ad value
      float gix, giy, giz; // raw a2d value
      
      // post calibration values
      float ax, ay, az; // in g's
      float gx, gy, gz;  // degrees per second
      
      // filtered and post calibration
      float lax, lay, laz; // in g's
      float lgx, lgy, lgz;  // degrees per second

      // Anguler Reference
      float a_roll, a_pitch, a_yaw; // acceleration based and raw
      float l_roll, l_pitch, l_yaw; // low pass filtered
      float k_roll, k_pitch, k_yaw; // kalman filtered


      // Complementary Filter
      float comp_filter = 0.05;
      float inv_comp_filter = 1.0 - comp_filter;

      // Raw Low Pass Filter
      float raw_low_pass_filter = 1.0; //Works best when smoothing is left to kalman
      float inv_raw_low_pass_filter = 1.0 - raw_low_pass_filter;
      
      // Post Kalman Low Pass Filter
      float kal_low_pass_filter = 1.00; 
      float inv_kal_low_pass_filter = 1.0 - raw_low_pass_filter;

      //Bias Adjustment Filter
      float gyro_auto_bias_rate = 0.005;
      float inverted_gyro_auto_bias_rate = 1.0 - gyro_auto_bias_rate;
      float maxGyroMotion = 10.0;//2.0
      float minAccelMotion = 0.9;//0.95;
      float maxAccelMotion = 1.1;//0.99;
      int stability_counter_threshold = 100;
      int stability_counter = 0;
      
      // Calibration Values
      float scale_ax = 0.98885; // 1/1.01128
      float scale_ay = 0.98885; // 1/1.01128
      float scale_az = 0.98885; // 1/1.01128
      float offset_gx = 0.040; //  +0.080
      float offset_gy = -0.13; //  -0.140
      float offset_gz = 0.29; //  +0.195
      
#ifdef USE_KALMAN
      Kalman kalmanPitch;
      Kalman kalmanRoll;
#endif

      
    void init();    
    
    TwoWheel_TF_Calc();
    bool calculateOrientation();
    float getMyPitch();
    float getMyRoll();
    float getLastTime();
      
  private:
    unsigned long microsNow, microsPerReading, microsPrevious;
    float lastDt = 0.01;
    bool is_initialized = false;
    bool updateRawOrientation();
};

/*Constructor */
TwoWheel_TF_Calc::TwoWheel_TF_Calc()
{
  
}

/*Constructor */
void TwoWheel_TF_Calc::init()
{
  if (!IMU.begin())
    return;
  updateRawOrientation();
  k_pitch = l_pitch = a_pitch;
  k_roll = l_roll = a_roll;
  microsPrevious = micros();

#ifdef USE_KALMAN
  kalmanPitch.setAngle(k_pitch);
  kalmanRoll.setAngle(k_roll);
#endif

  is_initialized = true;
}

bool TwoWheel_TF_Calc::updateRawOrientation(){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(aix, aiy, aiz);
  }else{
    return false;
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gix, giy, giz);
  }else{
    return false;
  }

  // convert from raw data to gravity and degrees/second units
  ax = (aix)*scale_ax; // eg 1/1.01128
  ay = (aiy)*scale_ay; // eg 1/1.01128
  az = (aiz)*scale_az; // eg 1/1.01128
  gx = (gix) - offset_gx; //  eg +0.080
  gy = (giy) - offset_gy; //  eg -0.140
  gz = (giz) - offset_gz; //  eg +0.195

  float deltaAccel = ax*ax + ay*ay + az*az;
  float deltaGyro = max(max(fabs(giy),fabs(gix)), fabs(giz));
  if( deltaAccel < maxAccelMotion &&  deltaAccel > minAccelMotion && deltaGyro < maxGyroMotion){
    if(stability_counter >= stability_counter_threshold){
      offset_gx = offset_gx*inverted_gyro_auto_bias_rate + gix*gyro_auto_bias_rate;
      offset_gy = offset_gy*inverted_gyro_auto_bias_rate + giy*gyro_auto_bias_rate;
      offset_gz = offset_gz*inverted_gyro_auto_bias_rate + giz*gyro_auto_bias_rate;
    } else {
      stability_counter++;
    }
  } else {
      stability_counter = 0;
  }

  // update the filter, which computes orientation
  a_roll = atan2(-ay, az);
  a_pitch = atan2(-ax, sqrt(ay*ay + az*az));
  if (ay * sin(a_roll) + az * cos(a_roll) == 0.0) 
     a_pitch = ax > 0.0 ? M_PI_2 : -M_PI_2;
  else 
     a_pitch = (float)atan(-ax / (ay * sin(a_roll) + az * cos(a_roll)));
  if (a_roll > 0)
    a_roll = - (a_roll - M_PI);
  else
    a_roll = - (a_roll + M_PI);

  a_roll = a_roll*radian_to_degree;
  a_pitch = a_pitch*radian_to_degree;
//    Serial.print(" ");
//    Serial.print(a_pitch);
//    Serial.print(" ");
//    Serial.print(a_roll);
}

bool TwoWheel_TF_Calc::calculateOrientation(){  
  microsNow  = micros();
  float dt = (microsNow - microsPrevious)*0.000001f; // convert 1/1000000.0f to sec;
  if (dt < 0.0){
    dt = lastDt; // first oder attempt at rollover protection
  } else {
    lastDt = dt;
  }
  bool success = updateRawOrientation();
  if(!success)
    return success;
    
  lax = inv_raw_low_pass_filter * (lax) + raw_low_pass_filter * (ax);
  lay = inv_raw_low_pass_filter * (lay) + raw_low_pass_filter * (ay);
  laz = inv_raw_low_pass_filter * (laz) + raw_low_pass_filter * (az);
  lgx = inv_raw_low_pass_filter * (lgx) + raw_low_pass_filter * (gx);
  lgy = inv_raw_low_pass_filter * (lgy) + raw_low_pass_filter * (gy);
  lgz = inv_raw_low_pass_filter * (lgz) + raw_low_pass_filter * (gz);
//    Serial.print(" ");
//    Serial.print(lax);
//    Serial.print(" ");
//    Serial.print(lay);
//    Serial.print(" ");
//    Serial.print(lgx);
//    Serial.print(" ");
//    Serial.print(lgy);

  /*Complementary filters to smooth rough pitch and roll estimates*/
  l_pitch = inv_comp_filter * ( l_pitch + ( lgy * dt ) ) + ( comp_filter * a_pitch );
  l_roll = inv_comp_filter * ( l_roll - ( lgx * dt ) ) + ( comp_filter * a_roll );
//    Serial.print(" ");
//    Serial.print(dt);
//    Serial.print(" ");
//    Serial.print(a_pitch);
//    Serial.print(" ");
//    Serial.print(a_roll);
//    Serial.print(" ");
//    Serial.print(lgy);
//    Serial.print(" ");
//    Serial.print(lgx);
//    Serial.print(" ");
//    Serial.print(l_pitch);
//    Serial.print(" ");
//    Serial.print(l_roll);

//    Serial.print(" ");
//    Serial.print(inv_comp_filter);
//    Serial.print(" ");
//    Serial.print(comp_filter);

  /*Kalman filter for most accurate pitch estimates*/
  k_pitch = l_pitch; 
  k_roll = l_roll; 

  /*Kalman filter for most accurate pitch estimates*/
#ifdef USE_KALMAN
  k_pitch = kalmanPitch.getAngle(k_pitch, lgy, dt);
  k_roll = kalmanRoll.getAngle(k_roll, -lgx, dt);
#endif

  k_pitch = inv_kal_low_pass_filter * (k_pitch) + kal_low_pass_filter * (k_pitch);
  k_roll = inv_kal_low_pass_filter * (k_roll) + kal_low_pass_filter * (k_roll);

  microsPrevious = microsNow;
  return success;
}

float TwoWheel_TF_Calc::getMyPitch(){
  return k_pitch;
}

float TwoWheel_TF_Calc::getMyRoll(){
  return k_roll;
}

float TwoWheel_TF_Calc::getLastTime(){
  return microsPrevious;
}



#endif REFERENCE_CALCULATOR
