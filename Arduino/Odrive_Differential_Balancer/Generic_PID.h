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
    PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd,
        int inpolarity, int outpolarity, double _min, double _max);

    bool Compute(double deltaTime);
    void SetLimitations(double _min, double _max); 
    
    //available but not commonly used functions ********************************************************
    void SetTunings(double Kp, double Ki, double Kd);     
  
    void SetInPolarity(int _polarity);     
    void SetOutPolarity(int _polarity);     
                        
    //Display functions 
    double GetKp();              
    double GetKi();              
    double GetKd();              
    int GetInPolarity();            
    int GetOutPolarity();           

  private:
    void init();       

    double kp;           
    double ki;                  
    double kd;                  
    
    int inpolarity;
    int outpolarity;
  
    double *myInput;              
    double *refOutput;             
    double *mySetpoint;      
    
    double outputIntegral, lastInput;
    double minOutput, maxOutput;
};

/*Constructor */
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd,
        int inpolarity, int outpolarity, double _min, double _max)
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
bool PID::Compute(double deltaTime)
{
   //double timeChange = deltaTime;

    /*Compute all the working error variables*/
    double input = *myInput;
    if(inpolarity == REVERSE)
    {
      input = - input;
    }
    double error = *mySetpoint - input;
    double dInput = (input - lastInput)/deltaTime;
    outputIntegral+= (ki * error)*deltaTime;

    if(outputIntegral > maxOutput) 
      outputIntegral= maxOutput;
    else if(outputIntegral < minOutput) 
      outputIntegral= minOutput;

    double output;
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
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

/* SetLimitations */
void PID::SetLimitations(double _min, double _max)
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
   outputIntegral = *refOutput;
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
double PID::GetKp()
{
  return  kp; 
}

double PID::GetKi()
{
  return  ki;
}

double PID::GetKd()
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
