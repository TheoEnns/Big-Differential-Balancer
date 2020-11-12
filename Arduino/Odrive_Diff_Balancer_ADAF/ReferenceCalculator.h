#include "Arduino.h"

/**************
 * Notes
 *   - For the ADAF MCU, the floating point math is in fact faster than double
 *     Time to process heading is 4.05ms on average with doubles vs 3.82ms with floats
 */

#ifndef REFERENCE_CALCULATOR
#define REFERENCE_CALCULATOR

#include <Adafruit_LSM6DSOX.h>
#include <math.h>

//#define DEBUG_IMUINIT 1 
//#define DEBUG_IMURAW 1 
//#define DEBUG_IMUFILTERING 1 
//#define DEBUG_IMUFINAL 1
//#define DEBUG_IMUTIME 1

static const float radian_to_degree = 180.0/M_PI;

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11
 
class IMU_TF_Calc
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
      float maxGyroMotion = 2.0;
      float minAccelMotion = 0.95;
      float maxAccelMotion = 0.99;
      int stability_counter_threshold = 100;
      int stability_counter = 0;
      
      // Calibration Values
      float scale_ax = 1.0; // 1/1.01128
      float scale_ay = 1.0; // 1/1.01128
      float scale_az = 1.0; // 1/1.01128
      float offset_gx = 0.0; //  +0.080
      float offset_gy = 0.0; //  -0.140
      float offset_gz = 0.0; //  +0.195

      
    void init();    
    
    IMU_TF_Calc();
    bool calculateOrientation();
    float getMyPitch();
    float getMyRoll();
    float getLastTime();
      
  private:
    Adafruit_LSM6DSOX sox;
    unsigned long microsNow, microsPerReading, microsPrevious;
    float lastDt = 0.01;
    bool is_initialized = false;
    bool updateRawOrientation();
};

/*Constructor */
IMU_TF_Calc::IMU_TF_Calc()
{
  
}

/*Constructor */
void IMU_TF_Calc::init()
{
#ifdef DEBUG_IMUINIT 
  Serial.println("Adafruit LSM6DSOX Init Started!");
#endif  
 
  if (!sox.begin_I2C()) {
    #ifdef DEBUG_IMUINIT 
      Serial.println("Failed to find LSM6DSOX chip");
    #endif 
    return;
  }
#ifdef DEBUG_IMUINIT 
  Serial.println("LSM6DSOX Found!");
#endif  
 
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
//  sox.setAccelDataRate(LSM6DS_RATE_416_HZ);
//  sox.setGyroDataRate(LSM6DS_RATE_416_HZ);
  
#ifdef DEBUG_IMUINIT
  {
    Serial.print("Gyro data rate set to: ");
    switch (sox.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
    }
    Serial.print("Accelerometer data rate set to: ");
    switch (sox.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
    }Serial.print("Gyro range set to: ");
    switch (sox.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCT_GYRO_RANGE_4000_DPS:
      break; // unsupported range for the DSOX
    }Serial.print("Accelerometer range set to: ");
    switch (sox.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
  }
#endif

  updateRawOrientation();
  k_pitch = l_pitch = a_pitch;
  k_roll = l_roll = a_roll;
  microsPrevious = micros();

  is_initialized = true;
}

bool IMU_TF_Calc::updateRawOrientation(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  // convert from raw data to gravity and degrees/second units
  ax = (accel.acceleration.x)*scale_ax; // eg 1/1.01128
  ay = (accel.acceleration.y)*scale_ay; // eg 1/1.01128
  az = (accel.acceleration.z)*scale_az; // eg 1/1.01128
  gx = (gyro.gyro.x) - offset_gx; //  eg +0.080
  gy = (gyro.gyro.y) - offset_gy; //  eg -0.140
  gz = (gyro.gyro.z) - offset_gz; //  eg +0.195

  float deltaAccel = ax*ax + ay*ay + az*az;
  float deltaGyro = max(max(fabs(gyro.gyro.y),fabs(gyro.gyro.x)), fabs(gyro.gyro.z));
  if( deltaAccel < maxAccelMotion &&  deltaAccel > minAccelMotion && deltaGyro < maxGyroMotion){
    if(stability_counter >= stability_counter_threshold){
      offset_gx = offset_gx*inverted_gyro_auto_bias_rate + gyro.gyro.x*gyro_auto_bias_rate;
      offset_gy = offset_gy*inverted_gyro_auto_bias_rate + gyro.gyro.y*gyro_auto_bias_rate;
      offset_gz = offset_gz*inverted_gyro_auto_bias_rate + gyro.gyro.z*gyro_auto_bias_rate;
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
  
#ifdef DEBUG_IMURAW 
{
    Serial.print(" ");
    Serial.print(a_pitch);
    Serial.print(" ");
    Serial.print(a_roll);
    Serial.print(" ");
    Serial.println(a_yaw);
}
#endif

}

bool IMU_TF_Calc::calculateOrientation(){  
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

  /*Complementary filters to smooth rough pitch and roll estimates*/
  l_pitch = inv_comp_filter * ( l_pitch + ( lgy * dt ) ) + ( comp_filter * a_pitch );
  l_roll = inv_comp_filter * ( l_roll - ( lgx * dt ) ) + ( comp_filter * a_roll );

  /*Kalman filter for most accurate pitch estimates*/
  k_pitch = l_pitch; 
  k_roll = l_roll; 

  k_pitch = inv_kal_low_pass_filter * (k_pitch) + kal_low_pass_filter * (k_pitch);
  k_roll = inv_kal_low_pass_filter * (k_roll) + kal_low_pass_filter * (k_roll);

  microsPrevious = microsNow;

#ifdef DEBUG_IMUFILTERING
  {
    Serial.print(" ");
    Serial.print(lax);
    Serial.print(" ");
    Serial.print(lay);
    Serial.print(" ");
    Serial.print(lgx);
    Serial.print(" ");
    Serial.print(lgy);
  
    Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(a_pitch);
    Serial.print(" ");
    Serial.print(a_roll);
    Serial.print(" ");
    Serial.print(lgy);
    Serial.print(" ");
    Serial.print(lgx);
    Serial.print(" ");
    Serial.print(l_pitch);
    Serial.print(" ");
    Serial.print(l_roll);
  
    Serial.print(" ");
    Serial.print(inv_comp_filter);
    Serial.print(" ");
    Serial.print(comp_filter);
  }
#endif

#ifdef DEBUG_IMUFINAL
  {
    Serial.print(" ");
    Serial.print(k_pitch);
    Serial.print(" ");
    Serial.print(k_roll);
  }
#endif

#ifdef DEBUG_IMUTIME
  {
    Serial.print(" ");
    Serial.print(dt);
  }
#endif

  return success;
}

float IMU_TF_Calc::getMyPitch(){
  return k_pitch;
}

float IMU_TF_Calc::getMyRoll(){
  return k_roll;
}

float IMU_TF_Calc::getLastTime(){
  return microsPrevious;
}

#endif REFERENCE_CALCULATOR
