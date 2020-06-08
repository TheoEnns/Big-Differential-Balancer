#include "Arduino.h"

#ifndef ODRIVE_CONTROL
#define ODRIVE_CONTROL

#include "Generic_PID.h"
#include "ReferenceCalculator.h"

#define VERBOSE_PRINT 1


class ODrive
{
  public:   
    ODrive();
    ~ODrive();
    void init(IMU_TF_Calc *_myTF);   
    void update_motion();  
    bool read_encoder(double dt);  
    bool update_angle_PID_loops(double dt);
    bool update_motor_output();
                        
    //Display functions           

  private: 
    IMU_TF_Calc *myTF;
  
    //------------------
    // Control Loop
    //------------------
    unsigned long odriveMicrosNow, odriveMicrosPerReading, odriveMicrosPrevious;    
    int frqHz = 120;
    
    //------------------
    // Physical Shape 
    //------------------
    float wheel_angle_to_perimeter = 2*M_PI*82.55/180.0;
    int encoder_counter = 0;
    int encoder_read_freq = 40;
    int encoder_read_max_counter = frqHz/encoder_read_freq;
    
    
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
    
    double right_vel_encoder = 0;
    double left_vel_encoder = 0;
    double last_vel_right_encoder = 0;
    double last_vel_left_encoder = 0;
    
    double right_distance= 0;
    double left_distance = 0;
    double right_velocity = 0;
    double left_velocity = 0;
    double right_acceleration = 0;
    double left_acceleration = 0;
    
    double right_target_distance = 0;
    double left_target_distance = 0;
    double right_target_velocity = 0;
    double left_target_velocity = 0;
    double right_target_acceleration = 0;
    double left_target_acceleration = 0;
    
    double right_target_angle = 0;
    double left_target_angle = 0;
    
    double right_impetus = 0;
    double left_impetus = 0;
    double adjusted_pitch = 0;
    
    double dist_p_factor = 0.10;  // 0.10
    double dist_d_factor = 0.010;  // 0.01
    double angle_p_factor = 0.10;  // 0.10
    double angle_d_factor = 0.010;  // 0.01
    double impact_p_factor = 0.60; // 0.50
    double impact_d_factor = 0.035; //0.04 
    PID* right_distancePID;
    PID* left_distancePID;
    PID* right_anglePID;
    PID* left_anglePID;
    PID* right_pitchPID;
    PID* left_pitchPID;
    /* PID right_distancePID(&right_distance, &right_target_velocity, &right_target_distance,
            dist_p_factor, 0.00, dist_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -45, 45);
    PID left_distancePID(&left_distance, &left_target_velocity, &left_target_distance,
            dist_p_factor, 0.00, dist_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -45, 45);
    PID right_anglePID(&right_velocity, &right_target_angle, &right_target_velocity,
            angle_p_factor, 0.00, angle_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -5, 5);
    PID left_anglePID(&left_velocity, &left_target_angle, &left_target_velocity,
            angle_p_factor, 0.00, angle_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -5, 5);
    PID right_pitchPID(&adjusted_pitch, &right_impetus, &right_target_angle, //double* Input, double* Output, double* Setpoint,
            impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,DIRECT, -45, 45);
    PID left_pitchPID(&adjusted_pitch, &left_impetus, &left_target_angle, //double* Input, double* Output, double* Setpoint,
            impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,DIRECT, -45, 45);  */  
};

ODrive::ODrive(){
    right_distancePID = new PID(&right_distance, &right_target_velocity, &right_target_distance,
            dist_p_factor, 0.0, dist_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -45, 45);
    left_distancePID = new PID(&left_distance, &left_target_velocity, &left_target_distance,
            dist_p_factor, 0.0, dist_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -45, 45);
    right_anglePID = new PID(&right_velocity, &right_target_angle, &right_target_velocity,
            angle_p_factor, 0.0, angle_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -5, 5);
    left_anglePID = new PID(&left_velocity, &left_target_angle, &left_target_velocity,
            angle_p_factor, 0.0, angle_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,REVERSE, -5, 5);
    right_pitchPID = new PID(&adjusted_pitch, &right_impetus, &right_target_angle, //double* Input, double* Output, double* Setpoint,
            impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,DIRECT, -45, 45);
    left_pitchPID = new PID(&adjusted_pitch, &left_impetus, &left_target_angle, //double* Input, double* Output, double* Setpoint,
            impact_p_factor, 0.0, impact_d_factor, //double Kp, double Ki, double Kd,
            DIRECT,DIRECT, -45, 45);   
}

ODrive::~ODrive(){
  free(right_distancePID);
  free(left_distancePID);
  free(right_anglePID);
  free(left_anglePID);
  free(right_pitchPID);
  free(left_pitchPID);
}

void ODrive::init(IMU_TF_Calc *_myTF){
  _myTF = _myTF;
  Serial1.begin(115200);
  Serial1.setTimeout(5);
  
  Serial1.println("w axis0.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");
  Serial1.println("w axis1.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");
  
  // initialize variables to pace updates to correct rate
  odriveMicrosPerReading = 1000000 / frqHz;
  odriveMicrosPrevious = micros();
  Serial1.println("f 0");
  Serial1.println("f 1");
}

void ODrive::update_motion() {
  // check if it's time to read data and update the filter
  odriveMicrosNow = micros();
  if (odriveMicrosNow - odriveMicrosPrevious >= odriveMicrosPerReading) {  
    double dt = (odriveMicrosNow - odriveMicrosPrevious)*0.000001f; // 1/1000000.0f;

//    // Read IMU, update the filters, and compute orientation
    myTF->calculateOrientation();

    //Get Distance Traveled
    encoder_counter = (encoder_counter+1)%encoder_read_max_counter;
    if (encoder_counter==0){
      read_encoder(dt);
//      right_distancePID->Compute(dt);
//      left_distancePID->Compute(dt);
      right_anglePID->Compute(dt);
      left_anglePID->Compute(dt);
    }
    
    //Update PIDs
    adjusted_pitch = tan(myTF->getMyPitch()*M_PI/180.0)*180.0/M_PI;//*fabs(myTF->getMyPitch());
//    update_angle_PID_loops( dt);
    right_pitchPID->Compute(dt);
    left_pitchPID->Compute(dt);
    
    //Update Motors
    update_motor_output();

    // print the yaw, pitch and roll
#ifdef VERBOSE_PRINT
    Serial.println(" ");
    Serial.print("Time: ");
    Serial.print(odriveMicrosNow - odriveMicrosPrevious);

    // IMU Sensors
//    Serial.print("\t offset_gx: ");
//    Serial.print(myTF->offset_gx);
//    Serial.print("\t offset_gy: ");
//    Serial.print(myTF->offset_gy);
//    Serial.print("\t offset_gz: ");
//    Serial.print(myTF->offset_gz);
//    Serial.print("\t Pitch: ");
//    Serial.print(myTF->getMyPitch());
//    Serial.print("\t Roll: ");
//    Serial.print(myTF->getMyRoll());
    
    // Motion Values
//    Serial.print("\t r_enc: ");
//    Serial.print(right_encoder);
//    Serial.print("\t l_enc: ");
//    Serial.print(left_encoder);

//    Serial.print("\t r_t_ang: ");
//    Serial.print(right_target_angle);
//    Serial.print("\t l_t_ang: ");
//    Serial.print(left_target_angle);
    
//    Serial.print("\t r_dist_err: ");
//    Serial.print(right_distance_error);
//    Serial.print("\t l_dist_err: ");
//    Serial.print(left_distance_error);
    
//    Serial.print("\t right_impetus: ");
//    Serial.print(right_impetus);
//    Serial.print("\t left_impetus: ");
//    Serial.print(left_impetus);
#endif // VERBOSE_PRINT

    // increment previous time, so we keep proper pace
    odriveMicrosPrevious = odriveMicrosNow;
  }
}

//------------------
// Motor Control 
//------------------
bool ODrive::read_encoder(double dt){  
  double old_right_distance = right_distance;
  double old_left_distance = left_distance;
  double old_right_velocity = right_velocity;
  double old_left_velocity = left_velocity;
  double old_right_acceleration = right_acceleration;
  double old_left_acceleration = left_acceleration;

  right_encoder = -4.0*Serial1.parseFloat();
  right_vel_encoder = -4.0*Serial1.parseFloat();
  left_encoder = 4.0*Serial1.parseFloat();
  left_vel_encoder = 4.0*Serial1.parseFloat();

  if( fabs(right_encoder - last_right_encoder) < 20.0 || fabs(right_encoder - last_right_encoder) < 20.0){
    right_distance = wheel_angle_to_perimeter*right_encoder;
    left_distance = wheel_angle_to_perimeter*left_encoder;
    right_velocity = wheel_angle_to_perimeter*right_vel_encoder;
    left_velocity = wheel_angle_to_perimeter*left_vel_encoder;
    right_acceleration = (right_velocity - old_right_velocity)/dt;
    left_acceleration = (left_velocity - old_left_velocity)/dt;    
  } else {
    right_velocity = right_acceleration*dt + right_velocity;
    left_velocity = left_acceleration*dt + left_velocity;
    right_distance = right_velocity*dt + right_distance;
    left_distance = left_velocity*dt + left_distance;

    right_encoder = last_right_encoder + last_vel_right_encoder*dt;
    left_encoder = last_left_encoder + last_vel_left_encoder*dt;
  }
  last_left_encoder = left_encoder;
  last_right_encoder = right_encoder;
  last_vel_left_encoder = left_vel_encoder;
  last_vel_right_encoder = right_vel_encoder;
  
  while(Serial1.available()) Serial1.read();
  Serial1.println("f 0");
  Serial1.println("f 1");

  return true;
}

bool ODrive::update_angle_PID_loops(double dt){
  bool success = true;
  //success = right_distancePID->Compute(dt) && left_distancePID->Compute(dt);
  return success && right_pitchPID->Compute(dt) && left_pitchPID->Compute(dt);
}

bool ODrive::update_motor_output(){
  if ( fabs(myTF->getMyPitch()) < 15.0){
    Serial1.print("c 0 ");
    Serial1.print( - (right_impetus*1.0));
    Serial1.println(" ");
    Serial1.print("c 1 ");
    Serial1.print((left_impetus*1.0));
    Serial1.println(" ");
    return true;
  }else{
    Serial1.println("c 0 0 ");
    Serial1.println("c 1 0 ");
  }
  return false;
}

#endif
