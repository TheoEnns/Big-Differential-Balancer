#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <math.h>
#include <Kalman.h>
#include "Generic_PID.h"

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
float lax, lay, laz; // in g's
float lgx, lgy, lgz;  // degrees per second


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
PID right_distancePID(&right_distance_error, &right_target_velocity, &right_target_distance,
        0.0, 0.0, 0.0, //double Kp, double Ki, double Kd,
        DIRECT, -550, 550);
PID left_distancePID(&left_distance_error, &left_target_velocity, &left_target_distance,
        0.0, 0.0, 0.0, //double Kp, double Ki, double Kd,
        DIRECT, -550, 550);
PID right_pitchPID(&adjusted_pitch, &right_impetus, &right_target_velocity,
        impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
        DIRECT, -45, 45);
PID left_pitchPID(&adjusted_pitch, &left_impetus, &left_target_velocity,
        impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
        DIRECT, -45, 45);


void setup() {
  Serial.begin(115200);
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

    //Get Distance Traveled
    read_encoder(dt);
    
    //Update PIDs
    adjusted_pitch = k_pitch;
    update_PID_loops( dt);
    
    //Update Motors
    update_motor_output();

    // print the yaw, pitch and roll
#ifdef VERBOSE_PRINT
    Serial.println(" ");
    Serial.print("Time: ");
    Serial.print(microsNow - microsPrevious);

    // IMU Sensors
//    Serial.print("\t ax: ");
//    Serial.print(ax);
//    Serial.print("\t ay: ");
//    Serial.print(ay);
//    Serial.print("\t az: ");
//    Serial.print(az);
//    Serial.print("\t gx: ");
//    Serial.print(gx);
//    Serial.print("\t gy: ");
//    Serial.print(gy);
//    Serial.print("\t gz: ");
//    Serial.print(gz);

    // 1000* IMU Sensors Time Averaged
//    Serial.print("\t lax: ");
//    Serial.print(lax*1000);
//    Serial.print("\t lay: ");
//    Serial.print(lay*1000);
//    Serial.print("\t laz: ");
//    Serial.print(laz*1000);
//    Serial.print("\t lat: ");
//    Serial.print( sqrt(laz*laz + lay*lay + lax*lax)*1000 );    
//    Serial.print("\t lgx: ");
//    Serial.print(lgx*1000);
//    Serial.print("\t lgy: ");
//    Serial.print(lgy*1000);
//    Serial.print("\t lgz: ");
//    Serial.print(lgz*1000);

    // Bot Angle
//    Serial.print("\t aPitch: ");
//    Serial.print(a_pitch);
//    Serial.print("\t aRoll: ");
//    Serial.print(a_roll);
//    Serial.print("\t lPitch: ");
//    Serial.print(l_pitch);
//    Serial.print("\t lRoll: ");
//    Serial.print(l_roll);
    Serial.print("\t Pitch: ");
    Serial.print(k_pitch);
//    Serial.print("\t Roll: ");
//    Serial.print(k_roll);
    
    // Motion Values
//    Serial.print("\t r_enc: ");
//    Serial.print(right_encoder);
//    Serial.print("\t l_enc: ");
//    Serial.print(left_encoder);

    Serial.print("\t r_t_vel: ");
    Serial.print(right_target_velocity);
    Serial.print("\t l_t_vel: ");
    Serial.print(left_target_velocity);
    
    Serial.print("\t r_dist_err: ");
    Serial.print(right_distance_error);
    Serial.print("\t l_dist_err: ");
    Serial.print(left_distance_error);
    
    Serial.print("\t right_impetus: ");
    Serial.print(right_impetus);
    Serial.print("\t left_impetus: ");
    Serial.print(left_impetus);
    
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
  ax = (aix)*0.98885; // 1/1.01128
  ay = (aiy)*0.98885; // 1/1.01128
  az = (aiz)*0.98885; // 1/1.01128
  gx = (gix) + (0.000); //  +0.080
  gy = (giy) + (0.140); //  -0.140
  gz = (giz) - (0.195); //  +0.195

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

//------------------
// Motor Control 
//------------------
bool read_encoder(double dt){  
  double velocity;
  double old_right_distance_error = right_distance_error;
  double old_left_distance_error = left_distance_error;

  right_encoder = 4.0*Serial1.parseFloat();
  velocity = 4.0*Serial1.parseFloat();
  left_encoder = -4.0*Serial1.parseFloat();
  velocity = -4.0*Serial1.parseFloat();

  if( fabs(right_encoder - last_right_encoder) < 20.0 || fabs(right_encoder - last_right_encoder) < 20.0){
    right_distance_error += wheel_angle_to_perimeter*(right_encoder - last_right_encoder) - right_target_velocity*dt; // Subtract Off Target Velocity
    last_right_encoder = right_encoder;
    
    left_distance_error += wheel_angle_to_perimeter*(left_encoder - last_left_encoder) - left_target_velocity*dt; // Subtract Off Target Velocity
    last_left_encoder = left_encoder;

    right_distance_error = 0.95*old_right_distance_error + 0.05*right_distance_error;
    left_distance_error = 0.95*old_left_distance_error + 0.05*left_distance_error;
    
  } else {
    last_left_encoder = left_encoder;
    last_right_encoder = right_encoder;
  }
  
  while(Serial1.available()) Serial1.read();
  Serial1.println("f 0");
  Serial1.println("f 1");

  return true;
}

bool update_PID_loops(double dt){
  bool success = true;
  success = right_distancePID.Compute(dt) && left_distancePID.Compute(dt);
  return success && right_pitchPID.Compute(dt) && left_pitchPID.Compute(dt);
}

bool update_motor_output(){
  if ( fabs(k_pitch) < 15.0){
    Serial1.print("c 0 ");
    Serial1.print( - (right_impetus*1));
    Serial1.println(" ");
    Serial1.print("c 1 ");
    Serial1.print((left_impetus*1));
    Serial1.println(" ");
    return true;
  }
  Serial1.println("c 0 0 ");
  Serial1.println("c 1 0 ");
  return false;
}
