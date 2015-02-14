#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.0.0

class PID
{


  public:

    //Parameter types for some of the functions below
    enum mode_t { AUTOMATIC = 1, MANUAL = 0 };
    enum direction_t { DIRECT = 0, REVERSE = 1 };
    enum resolution_t { MILLIS = 0, MICROS = 1 };

    //commonly used functions **************************************************************************
    PID(float*, float*, float*,             // * constructor.  links the PID to the Input, Output, and 
        float, float, float,                //   Setpoint.  Initial tuning parameters are also set here
        direction_t);
    
    void SetMode(mode_t);                   // * sets PID to either MANUAL (0) or AUTOMATIC (1)
    
    bool Compute();                         // * performs the PID calculation.  it should be
                                            //   called every time loop() cycles. ON/OFF and
                                            //   calculation frequency can be set using SetMode
                                            //   SetSampleTime respectively
    
    void SetOutputLimits(float, float);     //clamps the output to a specific range. 0-255 by default, but
                                            //it's likely the user will want to change this depending on
                                            //the application



    //available but not commonly used functions ********************************************************
    void SetTunings(float, float,           // * While most users will set the tunings once in the 
                    float);                 //   constructor, this function gives the user the option
                                            //   of changing tunings during runtime for Adaptive control
    void SetControllerDirection(            // * Sets the Direction, or "Action" of the controller. DIRECT
                    direction_t);           //   means the output will increase when error is positive. REVERSE
                                            //   means the opposite.  it's very unlikely that this will be needed
                                            //   once it is set in the constructor.
    void SetSampleTime(int);                // * sets the frequency, in Milliseconds, with which 
                                            //   the PID calculation is performed.  default is 100
    void SetResolution(resolution_t);       // * Set the resolution of the GetTime() function. 
                                            //   MILLIS sets the resolution to milliseconds.
                                            //   MICROS sets the resolution to microseconds.



    //Display functions ****************************************************************
    float GetKp();                        // These functions query the pid for interal values.
    float GetKi();                        //  they were created mainly for the pid front-end,
    float GetKd();                        // where it's important to know what is actually 
    mode_t GetMode();                     //  inside the PID.
    direction_t GetDirection();           //

  private:
    void Initialize();
    unsigned long GetTime();    // * This will call either millis() or micros()
                                //   depending on the used resolution.
    
    float dispKp;               // * we'll hold on to the tuning parameters in user-entered 
    float dispKi;               //   format for display purposes
    float dispKd;               //
    
    float kp;                   // * (P)roportional Tuning Parameter
    float ki;                   // * (I)ntegral Tuning Parameter
    float kd;                   // * (D)erivative Tuning Parameter

    int controllerDirection;

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                 //   what these values are.  with pointers we'll just know.
              
    unsigned long lastTime;
    float ITerm, lastInput;
    unsigned long timeChange;

    unsigned long SampleTime;
    float secondsDivider;
    float outMin, outMax;
    bool inAuto;
};
#endif

