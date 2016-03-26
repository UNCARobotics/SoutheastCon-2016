#ifndef Alpha_h
#define Alpha_h

#include "Arduino.h"
#include <Servo.h> // for servo
#include <Adafruit_INA219.h> // for current sensing

#define COLOR_ARRAY_SIZE 20  //array for averaging color
#define CUR_ARRAY_SIZE 20    //array for averaging current
#define POS_ARRAY_SIZE 50   //array for averaging Servo Position
#define ER_ARRAY_SIZE 30  //array for averaging error

#define READY_POS 100 //open and ready for picking up
#define COLOR_READ_POS 40 //mostly closed to read the color sensor


// GRIPPER DEFINITION///////////////////////////////////////////////////////////////////////////////////////////
class AlphaGripper {
  public:
    uint8_t Cur_Address; //address of the current sensors
    volatile int relax;  // flag for zero'd finger (no block)
    int ServoPin; //pin each servo communicates on 
    
    
    float setpoint; // current draw desired
    float P;   //controller gains
    float D;
    float Error;  //actual current - desired
    float Error_History[ER_ARRAY_SIZE];
      
    float current_history[CUR_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float Avg_current; 
    
    float Pos[POS_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float NewPos; //after caculation, the new position to write to the servos 
    
    int grabbed; // flag for achieving a PD controlled grip

    Servo gripServo;     // a servo for the grippers
    Adafruit_INA219 ina219; // a current sensor for monitoring servo tourqe 
    
    void initSensors(){  // attaches servo and current sensor to the board
      gripServo.attach(ServoPin);
      ina219.begin(Cur_Address);
    }
    
    float currentSense() {                //take current reading and add it to a running average 
      float current; // current measured
      float Avg_Cur;
      // get current from corresponding sensor
       current = ina219.getCurrent_mA();
       Avg_Cur = runningAvg(current, CUR_ARRAY_SIZE, current_history);
       return Avg_Cur;
      
    }

    float Gripper_PD() { // PD caculation for translation
      float Prev_Error, Avg_Error, Addition, NewPos;

      grabbed = (Avg_Error < 50) ? 1 : 0; //check avg to see if holding block
        
      Prev_Error = Error;
      Error = currentSense() - setpoint;
      Avg_Error = runningAvg(Error, ER_ARRAY_SIZE, Error_History);
      Addition = P * (Error) + D * (Error - Prev_Error); //how many degrees added to servo pos
      return Addition;
    }
    
    float holdBlocks() {
      if (relax == 0) { //if not flagged

        NewPos += this->Gripper_PD(); //change in posistion

        NewPos = (NewPos < 0) ? 0 : NewPos; //NewPos should be positive;
        relax = this->zeroCheck(NewPos); // check the zero case
      }
      else NewPos = 100; //if no block, open back up
      return NewPos;
    }
    
    float CloseToRead() {
      for(int i=0;i<3;i++){ //close on blocks to read color
        this->gripServo.write(COLOR_READ_POS);   
       } 
    }

    int zeroCheck(float ThisPos) { // check to see if the finger is zero'd out (no block)
      int flag = 0;
      float Avg_Pos = 0;
      
      Avg_Pos = runningAvg(ThisPos, POS_ARRAY_SIZE, Pos);
      flag = (Avg_Pos) ? 0 : 1;
      return flag;
    }
    
    float runningAvg(float Var, int Size, float *Array) {
      float Avg_Var;
      for (int j = Size - 1; j > 0; j--) { //Shifts
        Array[j] = Array[j - 1];
      }
      Array[0] = Var;

      Avg_Var = 0;                                           //Sums/w abs() and finds Average
      for (int k = 0; k < Size; k++) {
        Avg_Var += Array[k];
      }
      Avg_Var = Avg_Var / Size;
      return Avg_Var;
    }

    void fill() {               //init's arrays each reset
      for (int i = 0; i < POS_ARRAY_SIZE; i++) {
        Pos[i] = 90;
      } 
      for (int i = 0; i < ER_ARRAY_SIZE; i++) {
        Error_History[i] = 500;
      } 
      for (int i = 0; i < CUR_ARRAY_SIZE; i++) {
        current_history[i] = setpoint;
      }
    }
};// END OF CLASS DEF/////////////////////

#endif
