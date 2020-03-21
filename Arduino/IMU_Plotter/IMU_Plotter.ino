#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <math.h>
#include <Kalman.h>

#define VERBOSE_PRINT 1

//------------------
// Control Loop
//------------------
unsigned long microsNow, microsPerReading, microsPrevious;
int frqHz = 120;


//------------------
// IMU Processing 
//------------------
Kalman kalmanPitch;
Kalman kalmanRoll;
float aix, aiy, aiz; // raw ad value
float gix, giy, giz; // raw a2d value
float ax, ay, az; // in g's
float gx, gy, gz;  // degrees per second
float a_roll, a_pitch, a_yaw; // acceleration based and raw
float l_roll, l_pitch, l_yaw; // low pass filtered
float k_roll, k_pitch, k_yaw; // kalman filtered
float low_pass_filter = 0.05;
float inv_low_pass_filter = 1.0 - low_pass_filter;
// filtered and post calibration
float lax, lay, laz; // in g's
float lgx, lgy, lgz;  // degrees per second
// Calibration
float scale_ax = 0.98885; // 1/1.01128
float scale_ay = 0.98885; // 1/1.01128
float scale_az = 0.98885; // 1/1.01128
float offset_gx = 1.340; //  +0.080
float offset_gy = -0.420; //  -0.140
float offset_gz = -1.110; //  +0.195
float gyro_auto_bias_rate = 0.005;
float inverted_gyro_auto_bias_rate = 1.0 - gyro_auto_bias_rate;


//------------------
// Physical Shape 
//------------------
float wheel_angle_to_perimeter = 2*M_PI*82.55/180;

//------------------
// Motor Control 
//------------------
bool fallover = false;
bool runaway = false;
bool traveling = false;
bool steady = false;
double right_encoder = 0;
double left_encoder = 0;
double last_right_encoder = 0;
double last_left_encoder = 0;
double right_distance_error = 0;
double left_distance_error = 0;
double right_target_distance = 0;
double left_target_distance = 0;
double right_target_velocity = 0;
double left_target_velocity = 0;
double right_impetus = 0;
double left_impetus = 0;
double adjusted_pitch = 0;
double impact_p_factor = 1.0;
double impact_d_factor = 0.1;


void setup() {
  Serial.begin(2000000);
  Serial1.begin(115200);
  Serial1.setTimeout(5);

  // start the IMU and filter
  if (!IMU.begin()) //Initialize IMU sensor 
     { Serial.println("Failed to initialize IMU!"); while (1);}

  // start the filter(s)
  updateRawOrientation();
  k_pitch = l_pitch = a_pitch;
  k_roll = l_roll = a_roll;
  kalmanPitch.setAngle(k_pitch);
  kalmanRoll.setAngle(k_roll);


  Serial1.println("w axis0.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");
  Serial1.println("w axis1.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / frqHz;
  microsPrevious = micros();
}

bool encoderFlip = true;
void loop() {
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {  
    double dt = (microsNow - microsPrevious)*0.000001f; // 1/1000000.0f;

    // Read IMU
    updateRawOrientation();
    
    // update the filters, which compute orientation
    updateFilters( dt);

    // print the yaw, pitch and roll
#ifdef VERBOSE_PRINT
//    Serial.print("Time: ");
//    Serial.print((microsNow - microsPrevious)/10000); // blue
    Serial.print(" ");
    Serial.print(offset_gx); // purple
    Serial.print(" ");
    Serial.print(offset_gy); // purple
    Serial.print(" ");
    Serial.print(offset_gz); // purple

    // IMU Sensors
////    Serial.print("\t ax: ");
//    Serial.print(" ");
//    Serial.print(ax);  // light red
////    Serial.print("\t ay: ");
//    Serial.print(" ");
//    Serial.print(ay); // green
////    Serial.print("\t az: ");
//    Serial.print(" ");
//    Serial.print(az); // yellow
//    Serial.print("\t gx: ");
//    Serial.print(" ");
//    Serial.print(gx); // purple
////    Serial.print("\t gy: ");
//    Serial.print(" ");
//    Serial.print(gy); // grey
////    Serial.print("\t gz: ");
//    Serial.print(" ");
//    Serial.print(gz); // light blue

    // 1000* IMU Sensors Time Averaged
////    Serial.print("\t lax: ");
//    Serial.print(" ");
//    Serial.print(lax*1000);
////    Serial.print("\t lay: ");
//    Serial.print(" ");
//    Serial.print(lay*1000);
////    Serial.print("\t laz: ");
//    Serial.print(" ");
//    Serial.print(laz*1000);
////    Serial.print("\t lat: ");
//    Serial.print(" ");
//    Serial.print( sqrt(laz*laz + lay*lay + lax*lax)*1000 );    
////    Serial.print("\t lgx: ");
//    Serial.print(" ");
//    Serial.print(lgx*1000);
////    Serial.print("\t lgy: ");
//    Serial.print(" ");
//    Serial.print(lgy*1000);
////    Serial.print("\t lgz: ");
//    Serial.print(" ");
//    Serial.print(lgz*1000);

    // Bot Angle
////    Serial.print("\t aPitch: ");
//    Serial.print(" ");
//    Serial.print(a_pitch);
////    Serial.print("\t aRoll: ");
//    Serial.print(" ");
//    Serial.print(a_roll);
////    Serial.print("\t lPitch: ");
//    Serial.print(" ");
//    Serial.print(l_pitch);
////    Serial.print("\t lRoll: ");
//    Serial.print(" ");
//    Serial.print(l_roll);
////    Serial.print("\t Pitch: ");
//    Serial.print(" ");
//    Serial.print(k_pitch);
////    Serial.print("\t Roll: ");
//    Serial.print(" ");
//    Serial.print(k_roll);

    
    Serial.println(" ");
    
#endif // VERBOSE_PRINT

    // increment previous time, so we keep proper pace
    microsPrevious = microsNow;
  }
}


//------------------
// IMU Processing 
//------------------
void updateRawOrientation(){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(aix, aiy, aiz);
  }else{
    return;}

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gix, giy, giz);
  }else{
    return;}

  // convert from raw data to gravity and degrees/second units
  ax = (aix)*scale_ax; // 1/1.01128
  ay = (aiy)*scale_ay; // 1/1.01128
  az = (aiz)*scale_az; // 1/1.01128
  gx = (gix) - offset_gx; //  +0.080
  gy = (giy) - offset_gy; //  -0.140
  gz = (giz) - offset_gz; //  +0.195

  float deltaAccel = ax*ax + ay*ay + az*az;
  float deltaGyro = fabs(giy + gix + giz);
  if( deltaAccel < 0.99 &&  deltaAccel > 0.95 && deltaGyro < 0.5){
    offset_gx = offset_gx*inverted_gyro_auto_bias_rate + gix*gyro_auto_bias_rate;
    offset_gy = offset_gy*inverted_gyro_auto_bias_rate + giy*gyro_auto_bias_rate;
    offset_gz = offset_gz*inverted_gyro_auto_bias_rate + giz*gyro_auto_bias_rate;
//    Serial.print(offset_gx);
//    Serial.print(" ");
//    Serial.print(offset_gy);
//    Serial.print(" ");
//    Serial.println(offset_gz);
  } else {
//    Serial.print(deltaAccel);
//    Serial.print(" ");
//    Serial.println(deltaGyro);
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

  a_roll = a_roll*180.0/M_PI;
  a_pitch = a_pitch*180.0/M_PI;
}

void updateFilters(float dt){    
  lax = 0.9 * (lax) + 0.1 * (ax);
  lay = 0.9 * (lay) + 0.1 * (ay);
  laz = 0.9 * (laz) + 0.1 * (az);
  lgx = 0.9 * (lgx) + 0.1 * (gx);
  lgy = 0.9 * (lgy) + 0.1 * (gy);
  lgz = 0.9 * (lgz) + 0.1 * (gz);

  /*Complementary filters to smooth rough pitch and roll estimates*/
  l_pitch = inv_low_pass_filter * ( l_pitch + ( lgy * dt ) ) + ( low_pass_filter * a_pitch );
  l_roll = inv_low_pass_filter * ( l_roll + ( lgx * dt ) ) + ( low_pass_filter * a_roll );

  /*Kalman filter for most accurate pitch estimates*/
  k_pitch = kalmanPitch.getAngle(l_pitch, lgy, dt);
  k_roll = kalmanRoll.getAngle(l_roll, lgx, dt);

//  /*Complementary filters to smooth rough pitch and roll estimates*/
//  l_pitch = inv_low_pass_filter * ( l_pitch + ( gy * dt ) ) + ( low_pass_filter * a_pitch );
//  l_roll = inv_low_pass_filter * ( l_roll + ( gx * dt ) ) + ( low_pass_filter * a_roll );
//
//  /*Kalman filter for most accurate pitch estimates*/
//  k_pitch = kalmanPitch.getAngle(l_pitch, gy, dt);
//  k_roll = kalmanRoll.getAngle(l_roll, gx, dt);
}