#define Serial SerialUSB
#include "ReferenceCalculator.h"
#include "OdriveControl.h"
#include "Collision_Detection.h"

//------------------
// Debugging
//------------------
#define VERBOSE_PRINT 1
unsigned long elapseIMU = 0;
unsigned long elapseEncoder = 0;
unsigned long elapseBumpers = 0;
unsigned long elapseMotor = 0;
unsigned long elapseDebug = 0;

//------------------
// Control Loop
//------------------
unsigned long microsNow;
unsigned long microsPerReading_IMU, microsPrevious_IMU;
unsigned long microsPerReading_Motor, microsPrevious_Motor;
unsigned long microsPerReading_Debug, microsPrevious_Debug;
unsigned long microsPerReading_Bumpers, microsPrevious_Bumpers;
int freqHz_IMU = 140;
int freqHz_Motor = 140;
int freqHz_Debug = 30;
int freqHz_Bumpers = 30;
IMU_TF_Calc imuTF;
ODrive myDriver;


//------------------
// Control Loop
//------------------
Collision_Detector colDecs;

void setup() {
  Serial.begin(2000000);

  // initialize variables to pace updates to correct rate
  microsPerReading_IMU = (unsigned long)(1000000.0f /freqHz_IMU);
  microsPerReading_Motor = (unsigned long)(1000000.0f /freqHz_Motor);
  microsPerReading_Debug = (unsigned long)(1000000.0f /freqHz_Debug);
  microsPerReading_Bumpers = (unsigned long)(1000000.0f /freqHz_Bumpers);
  
  microsPrevious_IMU = micros();
  microsPrevious_Motor = micros();
  microsPrevious_Debug = micros();
  microsPrevious_Bumpers = micros();
  imuTF.init();
  myDriver.init(&imuTF);
  colDecs.init();
}

void loop() {
  double average_loop_time = 0;
  while(!Serial.available()){
    unsigned long start = micros();
    // check if it's time to read data and update the filter
    microsNow  = micros();
    if (microsNow - microsPrevious_IMU >= microsPerReading_IMU) {  
      imuTF.calculateOrientation();
      // increment previous time, so we keep proper pace
      microsPrevious_IMU = microsNow;
      elapseIMU = micros() - microsNow;
    }
  
    // check if it's time to update motors
    microsNow  = micros();
    if (microsNow - microsPrevious_Motor >= microsPerReading_Motor) {  
      myDriver.update_motion();
      // increment previous time, so we keep proper pace
      microsPrevious_Motor = microsNow;
      elapseMotor = micros() - microsNow;
    }
  
    // check if it's time to update bumpers
    microsNow  = micros();
    if (microsNow - microsPrevious_Bumpers >= microsPerReading_Bumpers) {  
      Collision_T collision_status = colDecs.check_collision();
      microsPrevious_Bumpers = microsNow;
      elapseBumpers = micros() - microsNow;
    }
  
    
      // print the yaw, pitch and roll
  #ifdef VERBOSE_PRINT
    microsNow  = micros();
    if (microsNow - microsPrevious_Debug >= microsPerReading_Debug) { 
//      Serial.print("Time: ");
//      Serial.print((microsNow - start));
//      Serial.print(" ");
      
//      Serial.print(imuTF.getMyPitch());
//      Serial.print(" ");
//      Serial.print(imuTF.getMyRoll());
//      Serial.print(" ");

      myDriver.debug_print();
      
//      Serial.print(" dt(Debug):");
//      Serial.print(elapseDebug);
//      Serial.print(" dt(Motor):");
//      Serial.print(elapseMotor);
//      Serial.print(" dt(IMU):");
//      Serial.print(elapseIMU);
//      Serial.print(" dt(Bumpers):");
//      Serial.print(elapseBumpers);

      Serial.println(" ");

      // increment previous time, so we keep proper pace
      microsPrevious_Debug = microsNow;
      elapseDebug = micros() - microsNow;
    }    
  #endif // VERBOSE_PRINT
  
    //Safety Rollover Catches
    microsNow  = micros();
    if (microsNow < microsPrevious_IMU)
      microsPrevious_IMU = microsNow;
    if (microsNow < microsPrevious_Motor)
      microsPrevious_Motor = microsNow;
    if (microsNow < microsPrevious_Debug)
      microsPrevious_Debug = microsNow;
  }
  while(Serial.available()){}
}
