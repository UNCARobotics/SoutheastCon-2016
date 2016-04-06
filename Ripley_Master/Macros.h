#ifndef Macros_h
#define Macros_h

#include "Arduino.h"

  //LED Panel	
  #define LED_PIN 26
  
  //Gripper
  #define SS_BG1 44
  #define SS_BG2 46
  #define SS_AG1 42 
  #define SS_AG2 48 
  
  //Sides 
  #define SS_P2 30 //Front 
  #define SS_P3 A2 //Back
  #define SS_P4 A3 //Arm
  #define SS_P5 32 //Leg
  
  //Truck IR LimitSwitch
  #define SS_T 40  //Truck Pings
  #define SS_IR 38 //IR slave
  #define SS_LS A6 //Limit Switches
  
  #define ER_ARRAY_SIZE 30
  
  //Motor Drivers
  #define M2_D2 2
  #define M2_IN1 3
  #define M2_IN2 4

  #define M3_D2 5
  #define M3_IN1 6
  #define M3_IN2 7
  
  #define M0_D2 8
  #define M0_IN1 9
  #define M0_IN2 10
  
  #define M1_D2 11 
  #define M1_IN1 12
  #define M1_IN2 13
  
  #define EN_M0_M1 24
  #define EN_M2_M3 22
  
// Steppers
  #define LIL_X 0
  #define BIG_X 1
  #define LIL_Y 2
  #define BIG_Y 3
  #define Z     4
  
  // directions of steppers
  #define UP HIGH // for LIL_Y and BIG_Y
  #define DOWN LOW // for LIL_Y and BIG_Y
  #define RIGHT_LIL_X LOW //  for LIL_X and BIG_X                
  #define LEFT_LIL_X HIGH // for LIL_X and BIG_X                   
  #define RIGHT_BIG_X HIGH // for BIG_Y
  #define LEFT_BIG_X  LOW// for BIG_X
  #define OUT HIGH // for Z                                 
  #define IN LOW  // for Z                                  
 

  // Modes for step size
  #define MODE_2  A14 // all drivers share Mode2, 1, 0, these hold the values for the step size
  #define MODE_1  A13
  #define MODE_0  A12

 // Arm variables for delays between steps
#define LIL_X_DELAY 500
#define BIG_X_DELAY 500
#define LIL_Y_DELAY 1000
#define BIG_Y_DELAY 2000
#define Z_DELAY 1000

// Arm variables used in ramping up stepper motors
#define DELAYED_STEPS 100
#define TOP_SPEED 10 // a shorter delay in between toggling makes it move faster
#define BOTTOM_SPEED 10000 // a faster delay in between toggling makes it move slower


  


#endif
