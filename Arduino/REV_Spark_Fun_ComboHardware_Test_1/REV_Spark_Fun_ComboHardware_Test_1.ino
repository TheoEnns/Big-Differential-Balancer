#include <Servo.h>
#include <Encoder.h>

#define RIGHT 0
#define LEFT 1

Encoder myEncR(4, 5);  //pins 4 , 5
Encoder myEncL(6, 7);  //pins 6 , 7
Encoder * myEnc[2] = { &myEncR, &myEncL};

Servo myServoL;  
Servo myServoR;  
Servo * myServo[2] = { &myServoR, &myServoL};

long oldEncoder[2] = {-999,-999};

void setup() {
  Serial.begin(115200);
  delay(750);
  
  myServoR.attach(8);  
  myServoL.attach(9);  
  // 8 is right; forward < 1500; backwards > 1500
  // 9 is left; forward > 1500; backwards < 1500
  
  delay(750);
  float pos = 0;    // variable to store the servo position
  for (pos = 0; pos <= -100; pos -= 1) { 
    unsigned long times = micros();
    myServoR.write(1500-pos);   
    myServoL.write(1500+pos);   
    times = micros() - times;
//    Serial.print(times);
//    Serial.print("  ");
//    Serial.println(pos);
    delay(10); 
  }
}

void loop() {
  readEncoder();
  
  float pos = 0;    // variable to store the servo position
  for (pos = -100; pos <= 100; pos += 1) { 
    // in steps of 1 degree
    unsigned long times = micros();
    myServoR.write(1500-pos);   
    myServoL.write(1500+pos);   
    times = micros() - times;
//    Serial.print(times);
//    Serial.print("  ");
//    Serial.println(pos);
    delay(10);      
  }
  for (pos = 100; pos >= -100; pos -= 1) { 
    unsigned long times = micros();
    myServoR.write(1500-pos);   
    myServoL.write(1500+pos);  
    times = micros() - times;
//    Serial.print(times);
//    Serial.print("  ");
//    Serial.println(pos);
    delay(10);      
  }
}

void readEncoder(){
  long newEncoderR = myEncR.read();
  if (newEncoderR != oldEncoder[0]) {
    oldEncoder[0] = newEncoderR;
    SerialUSB.print("RE:");
    SerialUSB.println(newEncoderR);
  }

  long newEncoderL = myEncL.read();
  if (newEncoderL != oldEncoder[1]) {
    oldEncoder[1] = newEncoderL;
    SerialUSB.print("LE:");
    SerialUSB.println(newEncoderL);
  }
}
