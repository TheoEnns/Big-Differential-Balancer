/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder..
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEncR(4, 5);
Encoder myEncL(6, 7);
Encoder * myEnc[2] = { &myEncR, &myEncL};
//   avoid using pins with LEDs attached

void setup() {
  SerialUSB.begin(115200);
  delay(1500);
  SerialUSB.println("Basic Encoder Test:");
}

long oldPosition[2] = {-999,-999};

void loop() {
//  for(int i = 0; i<2;i++){
//    long newPosition = myEnc[i]->read();
//    if (newPosition != oldPosition[i]) {
//      oldPosition[i] = newPosition;
//      SerialUSB.print(i==0?"Right:":"Left:");
//      SerialUSB.println(newPosition);
//    }
//  }

  long newPositionR = myEncR.read();
  if (newPositionR != oldPosition[0]) {
    oldPosition[0] = newPositionR;
    SerialUSB.print("Right:");
    SerialUSB.println(newPositionR);
  }

  long newPositionL = myEncL.read();
  if (newPositionL != oldPosition[1]) {
    oldPosition[1] = newPositionL;
    SerialUSB.print("Left:");
    SerialUSB.println(newPositionL);
  }
}
