#include "ReferenceCalculator.h"

#define VERBOSE_PRINT 1

//------------------
// Control Loop
//------------------
unsigned long microsNow, microsPerReading, microsPrevious;
int frqHz = 120;
TwoWheel_TF_Calc myTF;

void setup() {
  Serial.begin(2000000);
  Serial1.begin(115200);
  Serial1.setTimeout(5);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / frqHz;
  microsPrevious = micros();
  myTF.init();
}

void loop() {
  // check if it's time to read data and update the filter
  microsNow  = micros();
  if (microsNow - microsPrevious >= microsPerReading) {  
    double dt = (microsNow - microsPrevious)*0.000001f; // 1/1000000.0f;
    myTF.calculateOrientation();
    
    // print the yaw, pitch and roll
#ifdef VERBOSE_PRINT
//    Serial.print("Time: ");
//    Serial.print((microsNow - microsPrevious)/10000);
    Serial.print(" ");
    Serial.print(myTF.getMyPitch());
    Serial.print(" ");
    Serial.print(myTF.getMyRoll());
    
    Serial.println(" ");
    
#endif // VERBOSE_PRINT

    // increment previous time, so we keep proper pace
    microsPrevious = microsNow;
  }
}
