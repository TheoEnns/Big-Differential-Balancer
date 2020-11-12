#include "Arduino.h"

/**************
 * Notes
 *   - For the ADAF MCU, the floating point math is in fact faster than double
 *     Time to process Motor Control is 625us on average with doubles vs 535us with floats
 */

#ifndef ODRIVE_CONTROL
#define ODRIVE_CONTROL

#include "Generic_PID.h"
#include "ReferenceCalculator.h"

//#define VERBOSE_ODRIVE_PRINT 1


class ODrive
{
  public:   
    ODrive();
    ~ODrive();
    void init(IMU_TF_Calc *_myTF);   
    void update_motion();  
    bool read_encoder(float dt);  
    bool update_angle_PID_loops(float dt);
    bool update_motor_output();
                        
    //Display functions  
    void debug_print();         

  private: 
    IMU_TF_Calc *myTF;
  
    //------------------
    // Control Loop
    //------------------
    unsigned long odriveMicrosNow, odriveMicrosPerReading, odriveMicrosPrevious,elapse;    
    int odriveUpdateFreqHz = 10000; // set high so main loop can control update rate
    
    //------------------
    // Physical Shape 
    //------------------
    float wheel_angle_to_perimeter = 2*M_PI*82.55/180.0;
    int encoder_counter = 0;
    int encoder_read_freq = 40;
    int encoder_read_max_counter = odriveUpdateFreqHz/encoder_read_freq;
    
    
    //------------------
    // Motor Control 
    //------------------
    bool fallover = false;
    bool runaway = false;
    bool traveling = false;
    bool steady = false;
    
    float right_encoder = 0;
    float left_encoder = 0;
    float last_right_encoder = 0;
    float last_left_encoder = 0;
    
    float right_vel_encoder = 0;
    float left_vel_encoder = 0;
    float last_vel_right_encoder = 0;
    float last_vel_left_encoder = 0;
    
    float right_distance= 0;
    float left_distance = 0;
    float right_velocity = 0;
    float left_velocity = 0;
    float right_acceleration = 0;
    float left_acceleration = 0;
    
    float right_target_distance = 0;
    float left_target_distance = 0;
    float right_target_velocity = 0;
    float left_target_velocity = 0;
    float right_target_acceleration = 0;
    float left_target_acceleration = 0;
    
    float right_target_angle = 0;
    float left_target_angle = 0;
    
    float right_impetus = 0;
    float left_impetus = 0;
    float adjusted_pitch = 0;
    
//    float dist_p_factor = 0.10;  // 0.10
//    float dist_d_factor = 0.010;  // 0.01
//    float angle_p_factor = 0.10;  // 0.10
//    float angle_d_factor = 0.010;  // 0.01
//    float pitch_p_factor = 0.60; // 0.50
//    float pitch_d_factor = 0.035; //0.04 

//    float dist_p_factor = 0.10;  // 0.10
//    float dist_d_factor = 0.010;  // 0.01
//    float angle_p_factor = 0.10;  // 0.10
//    float angle_d_factor = 0.010;  // 0.01
//    float pitch_p_factor = 1.60; // 0.650
//    float pitch_d_factor = 0.00; //0.015 
    
    float dist_p_factor = 0.10;  // 0.10
    float dist_d_factor = 0.010;  // 0.01
    float angle_p_factor = 0.10;  // 0.10
    float angle_d_factor = 0.010;  // 0.01
    float pitch_p_factor = 1.9; // 2.0
    float pitch_i_factor = 0.00; // 0.00
    float pitch_d_factor = 0.01; //0.015 
    
    PID* right_distancePID;
    PID* left_distancePID;
    PID* right_anglePID;
    PID* left_anglePID;
    PID* right_pitchPID;
    PID* left_pitchPID;
};

ODrive::ODrive(){
    right_distancePID = new PID(&right_distance, &right_target_velocity, &right_target_distance,
            dist_p_factor, 0.0, dist_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,REVERSE, -45, 45);
    left_distancePID = new PID(&left_distance, &left_target_velocity, &left_target_distance,
            dist_p_factor, 0.0, dist_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,REVERSE, -45, 45);
    right_anglePID = new PID(&right_velocity, &right_target_angle, &right_target_velocity,
            angle_p_factor, 0.0, angle_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,REVERSE, -5, 5);
    left_anglePID = new PID(&left_velocity, &left_target_angle, &left_target_velocity,
            angle_p_factor, 0.0, angle_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,REVERSE, -5, 5);
    right_pitchPID = new PID(&adjusted_pitch, &right_impetus, &right_target_angle, //float* Input, float* Output, float* Setpoint,
            pitch_p_factor, pitch_i_factor, pitch_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,DIRECT, -400, 400);
    left_pitchPID = new PID(&adjusted_pitch, &left_impetus, &left_target_angle, //float* Input, float* Output, float* Setpoint,
            pitch_p_factor, pitch_i_factor, pitch_d_factor, //float Kp, float Ki, float Kd,
            DIRECT,DIRECT, -400, 400);   
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
  myTF = _myTF;
  Serial1.begin(115200);
  Serial1.setTimeout(5);
  
  Serial1.println("w axis0.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");
  Serial1.println("w axis1.controller.config.control_mode CTRL_MODE_CURRENT_CONTROL");
  
  // initialize variables to pace updates to correct rate
  odriveMicrosPerReading = 1000000 / odriveUpdateFreqHz;
  odriveMicrosPrevious = micros();
  Serial1.println("f 0");
  Serial1.println("f 1");

  right_distancePID->init();
  left_distancePID->init();
  right_anglePID->init();
  left_anglePID->init();
  right_pitchPID->init();
  left_pitchPID->init();   
}

void ODrive::update_motion() {
  // check if it's time to read data and update the filter
  odriveMicrosNow = micros();
  if (odriveMicrosNow - odriveMicrosPrevious >= odriveMicrosPerReading) {  
    float dt = (odriveMicrosNow - odriveMicrosPrevious)*0.000001f; // 1/1000000.0f;

    //Get Distance Traveled
    encoder_counter = (encoder_counter+1)%encoder_read_max_counter;
//    if (encoder_counter==0){
//      read_encoder(dt);
//      right_distancePID->Compute(dt);
//      left_distancePID->Compute(dt);
//      right_anglePID->Compute(dt);
//      left_anglePID->Compute(dt);
//    }
    
    //Update PIDs
//    adjusted_pitch = fabs(myTF->getMyPitch())*myTF->getMyPitch();
    adjusted_pitch = tan(myTF->getMyPitch()*M_PI/180.0)*180.0/M_PI;//*fabs(myTF->getMyPitch());
//        adjusted_pitch = myTF->getMyPitch();
    update_angle_PID_loops( dt);
//    right_pitchPID->Compute(dt);
//    left_pitchPID->Compute(dt);
    
    //Update Motors
    update_motor_output();

    // print the yaw, pitch and roll
#ifdef VERBOSE_ODRIVE_PRINT
    debug_print();
#endif // VERBOSE_ODRIVE_PRINT

    // increment previous time, so we keep proper pace
    elapse = odriveMicrosNow - odriveMicrosPrevious;
    odriveMicrosPrevious = odriveMicrosNow;
  }
}

void ODrive::debug_print(){
//    Serial.print("T:");
//    Serial.print(elapse);

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

    Serial.print(" AP:");
    Serial.print(adjusted_pitch);
    
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
    
    Serial.print(" r_o:");
    Serial.print(right_impetus);
    Serial.print(" l_o:");
    Serial.print(left_impetus);
}

//------------------
// Motor Control 
//------------------
bool ODrive::read_encoder(float dt){  
  float old_right_distance = right_distance;
  float old_left_distance = left_distance;
  float old_right_velocity = right_velocity;
  float old_left_velocity = left_velocity;
  float old_right_acceleration = right_acceleration;
  float old_left_acceleration = left_acceleration;

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

bool ODrive::update_angle_PID_loops(float dt){
  bool success = true;
  //success = right_distancePID->Compute(dt) && left_distancePID->Compute(dt);
  bool success1 = right_pitchPID->Compute(dt);
  bool success2 = left_pitchPID->Compute(dt);
  return success1 && success2;
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
  Serial1.flush();
  return false;
}

#endif
