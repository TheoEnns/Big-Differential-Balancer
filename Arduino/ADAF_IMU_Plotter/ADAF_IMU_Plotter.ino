#define Serial SerialUSB
#include "ReferenceCalculator.h"

#define VERBOSE_PRINT 1

//------------------
// Control Loop
//------------------
unsigned long microsNow, microsPerReading, microsPrevious, microsPerReadingD, microsPreviousD;
int frqHz = 3000;
int frqHzD = 200;
TwoWheel_TF_Calc myTF;

void setup() {
  Serial.begin(2000000);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / frqHz;
  microsPerReadingD = 1000000 / frqHzD;
  microsPrevious = micros();
  microsPreviousD = micros();
  myTF.init();
}

void loop() {
  // check if it's time to read data and update the filter
  microsNow  = micros();
  if (microsNow - microsPrevious >= microsPerReading) {  
    double dt = (microsNow - microsPrevious)*0.000001f; // 1/1000000.0f;
    myTF.calculateOrientation();
    // increment previous time, so we keep proper pace
    microsPrevious = microsNow;
  }
    // print the yaw, pitch and roll
#ifdef VERBOSE_PRINT
  if (microsNow - microsPreviousD >= microsPerReadingD) { 
//    Serial.print("Time: ");
//    Serial.print((microsNowD - microsPreviousD)/10000);
    Serial.print(" ");
    Serial.print(myTF.getMyPitch());
    Serial.print(" ");
    Serial.print(myTF.getMyRoll());
    Serial.println(" ");
    // increment previous time, so we keep proper pace
    microsPreviousD = microsNow;
  }    
#endif // VERBOSE_PRINT
}
