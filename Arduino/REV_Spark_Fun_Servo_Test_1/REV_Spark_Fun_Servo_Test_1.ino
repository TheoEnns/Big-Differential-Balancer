/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservoL;  // create servo object to control a servo
Servo myservoR;  // create servo object to control a servo
// twelve servo objects can be created on most boards

float pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(115200);
  myservoR.attach(8);  
  myservoL.attach(9);  
  // 8 is right; forward < 1500; backwards > 1500
  // 9 is left; forward > 1500; backwards < 1500
}

void loop() {
  for (pos = -100; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    unsigned long times = micros();
    myservoR.write(1500-pos);   
    myservoL.write(1500+pos);   
    times = micros() - times;
    Serial.print(times);
    Serial.print("  ");
    Serial.println(pos);
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 100; pos >= -100; pos -= 1) { // goes from 180 degrees to 0 degrees
    unsigned long times = micros();
    myservoR.write(1500-pos);   
    myservoL.write(1500+pos);  
    times = micros() - times;
    Serial.print(times);
    Serial.print("  ");
    Serial.println(pos);
    delay(10);                       // waits 15ms for the servo to reach the position
  }
}
