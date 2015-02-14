/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, direction_t ControllerDirection)
{
	
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
	inAuto = false;
	
	PID::SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetResolution(MILLIS); // Use a resolution of milliseconds by default
    PID::SetTunings(Kp, Ki, Kd);
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = PID::GetTime();
   timeChange = (now - lastTime);
   if(SampleTime == 0 || timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/
	  float input = *myInput;
      float error = *mySetpoint - input;
      float prevError = *mySetpoint - lastInput;

      float dInput;
      float ICurrent = ki * (error + prevError) / 2.0f;
      if (SampleTime > 0) {
        ITerm += ICurrent;
        dInput = (input - lastInput);
      } else {
        ITerm += ICurrent*(((float)timeChange) / secondsDivider);
        dInput = (input - lastInput) / (((float)timeChange) / secondsDivider);
      }

      if(ITerm > outMax)
        ITerm = outMax;
      else
        if(ITerm < outMin)
            ITerm = outMin;
 
      /*Compute PID Output*/
      float output = kp * error + ITerm - kd * dInput;
      
	  if(output > outMax)
        output = outMax;
      else
        if(output < outMin)
            output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp < 0 || Ki < 0 || Kd < 0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   if (SampleTime > 0) {
    float SampleTimeInSec = ((float)SampleTime) / secondsDivider;  
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
   } else {
    kp = Kp;
    ki = Ki;
    kd = Kd;
   }
 
  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 * If it's set to 0 or a negative value it will computer every time the
 * function is called
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio;
      if (SampleTime > 0)
        ratio = (float)NewSampleTime / (float)SampleTime;
      else
        ratio = (float)NewSampleTime / (float)timeChange; // We will assume the user is calling Compute at a regular interval

      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long) NewSampleTime;
   } else
      SampleTime = 0; // We will compute every time the function is called
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm = outMax;
	   else if(ITerm < outMin) ITerm = outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to MANUAL (0) or AUTOMATIC (1)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(mode_t Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(direction_t Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* GetTime()*******************************************************************
 * Will get the current time either by using millis() or micros()
 ******************************************************************************/
unsigned long PID::GetTime()
{
  if (secondsDivider == 1000.0f) return millis();
  return micros();
}

/* SetResolution(...)**********************************************************
 * Will set the resolution of GetTime().
 * MILLIS will set the resolution to milliseconds while
 * MICROS will set the resolution to microseconds.
 ******************************************************************************/
void PID::SetResolution(resolution_t resolution)
{
  if (resolution == MILLIS)
    secondsDivider = 1000.0f;
  else
    secondsDivider = 1000000.0f;
  lastTime = PID::GetTime()-SampleTime; // Update last time variable
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}
mode_t PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL; }
direction_t PID::GetDirection(){ return controllerDirection; }

