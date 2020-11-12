#include "Arduino.h"
#include <SparkFun_Qwiic_Button.h>

#ifndef COLLISION_DET
#define COLLISION_DET

enum Collision_T {
  Collision_No_Event=0,
  Collision_Back=1,
  Collision_Front=2,
  Collision_BR=0x31,
  Collision_BC=0x32,
  Collision_BL=0x33,
  Collision_FR=0x34,
  Collision_FC=0x35,
  Collision_FL=0x36
};

class Collision_Detector
{
  public:
    Collision_Detector();
    Collision_T check_collision();         
    Collision_T get_collision_state();
    void init();   
    
  private:  
    uint8_t brightness = 255;
    QwiicButton button[6];   
    bool button_status[6];   
    int cycle_count;
    Collision_T collisionState = Collision_No_Event;

    Collision_T update_state();
};

/*Constructor */
Collision_Detector::Collision_Detector()
{
  cycle_count = 0;
  for(int i = 0;i<6;i++){
    button_status[i] = false;
  }
}

/*Constructor */
void Collision_Detector::init()
{
  Wire.begin();
  for(int i = 0;i<6;i++){
    if (button[i].begin(i+0x31) == false){
      Serial.print("Button ");
      Serial.print(i+0x31,HEX);
      Serial.println(" did not acknowledge!!!");
    }else{
      Serial.print("Button ");
      Serial.print(i+0x31,HEX);
      Serial.println(" acknowledged.");
      button[i].LEDon(0);
    }
  }
}


/* Compute() */
Collision_T Collision_Detector::check_collision()
{
  if (button[cycle_count].isPressed() == true) {
    button_status[cycle_count] = true;
  }else{
    button_status[cycle_count] = false;
  }
//  if(collisionState == Collision_Front && cycle_count >= 3){
//    button[cycle_count].LEDon(brightness);
//  }else if(collisionState == Collision_Back && cycle_count < 3){
//    button[cycle_count].LEDon(brightness);
//  }else if(collisionState == Collision_No_Event){
//    button[cycle_count].LEDon(0);
//  }
  cycle_count = (cycle_count +1)%6;  
  
  return update_state();
}

Collision_T Collision_Detector::get_collision_state(){
  return collisionState;
}

Collision_T Collision_Detector::update_state(){
  bool contactMade = false;
  for(int i = 0;i<6;i++){
    if(button_status[i]){
      if(i>=3){
        if(collisionState == Collision_No_Event){
          button[3].LEDon(brightness);
          button[4].LEDon(brightness);
          button[5].LEDon(brightness);
          collisionState = Collision_Front;
        }
        contactMade = true;
      }else if(i <3){
        if(collisionState == Collision_No_Event){
          button[0].LEDon(brightness);
          button[1].LEDon(brightness);
          button[2].LEDon(brightness);
          collisionState = Collision_Back;
        }
        contactMade = true;
      }
    }
  }
  if(!contactMade && collisionState != Collision_No_Event){
    button[0].LEDon(0);
    button[1].LEDon(0);
    button[2].LEDon(0);
    button[3].LEDon(0);
    button[4].LEDon(0);
    button[5].LEDon(0);
    collisionState = Collision_No_Event;
  }
  return collisionState;
}

#endif
