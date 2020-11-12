#include "Arduino.h"

#ifndef GENERIC_PID
#define GENERIC_PID

class PID
{
  public:

    //Constants used in some of the functions below
    #define DIRECT  0
    #define REVERSE  1
  
    //commonly used functions **************************************************************************
    PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd,
        int inpolarity, int outpolarity, float _min, float _max);

    bool Compute(float deltaTime);
    void SetLimitations(float _min, float _max); 
    
    //available but not commonly used functions ********************************************************
    void SetTunings(float Kp, float Ki, float Kd);     
  
    void SetInPolarity(int _polarity);     
    void SetOutPolarity(int _polarity);     
                        
    //Display functions 
    float GetKp();              
    float GetKi();              
    float GetKd();              
    int GetInPolarity();            
    int GetOutPolarity();           

    void init();   
    
  private:    
    float kp;           
    float ki;                  
    float kd;                  
    
    int inpolarity;
    int outpolarity;
  
    float *myInput;              
    float *refOutput;             
    float *mySetpoint;      
    
    float outputIntegral, lastInput;
    float minOutput, maxOutput;
};

/*Constructor */
PID::PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd,
        int inpolarity, int outpolarity, float _min, float _max)
{
    refOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;

    PID::SetLimitations(_min, _max); 

    PID::SetInPolarity(inpolarity);
    PID::SetOutPolarity(outpolarity);
    PID::SetTunings(Kp, Ki, Kd);
}


/* Compute() */
bool PID::Compute(float deltaTime)
{
   //float timeChange = deltaTime;

    /*Compute all the working error variables*/
    float input = *myInput;
    if(inpolarity == REVERSE)
    {
      input = - input;
    }
    float error = *mySetpoint - input;
    float dInput = (input - lastInput)/deltaTime;
    outputIntegral+= (ki * error)*deltaTime;

    if(outputIntegral > maxOutput) 
      outputIntegral= maxOutput;
    else if(outputIntegral < minOutput) 
      outputIntegral= minOutput;

    float output;
    output = kp * error;

    /*Compute Rest of PID Output*/
    output += outputIntegral - kd * dInput;

    if(outpolarity == REVERSE)
    {
      output = - output;
    }
    if(output > maxOutput) 
      output = maxOutput;
    else if (output < minOutput) 
      output = minOutput;
    *refOutput = output;

    /*Remember some variables for next time*/
    lastInput = input;
    
    return true;
}

/* SetTunings */
void PID::SetTunings(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

/* SetLimitations */
void PID::SetLimitations(float _min, float _max)
{
   if(_min >= _max) 
     return;
   minOutput = _min;
   maxOutput = _max;

   if(*refOutput > maxOutput) 
      *refOutput = maxOutput;
   else 
      if(*refOutput < minOutput) 
        *refOutput = minOutput;

   if(outputIntegral > maxOutput) 
      outputIntegral= maxOutput;
   else 
      if(outputIntegral < minOutput) 
        outputIntegral= minOutput;
}

/* init() */
void PID::init()
{
   outputIntegral = 0;
   lastInput = *myInput;
   if(inpolarity == REVERSE)
    {
      lastInput = - lastInput;
    }
   if(outputIntegral > maxOutput) 
    outputIntegral = maxOutput;
   else if(outputIntegral < minOutput) 
    outputIntegral = minOutput;
}

/* SetPolarity */
void PID::SetInPolarity(int _polarity)
{
   inpolarity = _polarity;
}

/* SetPolarity */
void PID::SetOutPolarity(int _polarity)
{
  outpolarity = _polarity;
}

/* Getter Accessors */
float PID::GetKp()
{
  return  kp; 
}

float PID::GetKi()
{
  return  ki;
}

float PID::GetKd()
{ 
  return  kd;
}

int PID::GetInPolarity()
{ 
  return inpolarity;
}

int PID::GetOutPolarity()
{ 
  return outpolarity;
}

#endif GENERIC_PID
