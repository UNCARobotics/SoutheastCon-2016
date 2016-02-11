
/* Github Yeh Yeh
 *  This code is the coneptualization of a navigation function that provides setpoints to a robot. 
 * The robot uses these setpoints in conjuction with sonar and a PD controller 
 * to navigate directly to the given (X,Z) coordinate.
 * 
 * The 2 noteable sections: 
 * 
 * Class "side" allows for the creation of objects representing each side of the robot, containing it's sensor 
 * retreval method and the PD controllor methods. (this accounts for translation, and correctional rotation)
 * 
 * The Navigation function takes a setpoint for each side of the robot, then asks each side object to 
 * sense and caculate it's own needed adjustments. It then sums adjustments needed from each side and 
 * directs the motors accordingly. 
 */
// include the library code:
#include <LiquidCrystal.h> 
#include <NewPing.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(41, 43, 45, 47, 49, 51);

// initialize Sonar with NewPing Library
NewPing sonar[8] = {
  NewPing(22, 22),
  NewPing(24, 24),
  NewPing(26, 26),
  NewPing(28, 28),
  NewPing(30, 30),
  NewPing(32, 32),
  NewPing(34, 34),
  NewPing(36, 36)
};

float BaseSpeed = 0;

struct Motors {   //Motor Values
  int Enable;     
  int Direction;  //high = backwards
  int Speed;      //High = forwards
  
  int enablePin;  //Motor Pins
  int directionPin;
  int speedPin;
};

//Motor initilization (Picture Below shows motor numbers);
//        F
//    ||-2 1-|| 
// A  ||-3 0-||   L
//        B

Motors Motor[4] ={
  { HIGH, LOW, HIGH, 5, 6, 7},
  { HIGH, LOW, HIGH, 8, 9, 10},
  { HIGH, LOW, HIGH, 11, 12, 13},
  { HIGH, LOW, HIGH, 2, 3, 4}
}; 
////////////////////////////////////////////END OF INITIALIZATION///////////////////////////////////////////////////




class Side { /////////////////////////////SIDE CLASS DEFINITION/////////////////////////////////////////////////////
public:
    //initialized values
    int PingPin1;   //sensor number
    int PingPin2;
    
    float P_T;      // PD constants
    float D_T;
    float P_R;
    float D_R;
    
    // found values
    float Ping1;  //for storing distance
    float Ping2;

                        //sense distance for each side
    void sensePings() {                       
      Ping1 = sonar[PingPin1].ping_median();  // send multiple pulses, return median distance
      Ping2 = sonar[PingPin2].ping_median();  
      Ping1 = Ping1/29/2;                     // converts to cm
      Ping2 = Ping2/29/2;
    }

    
    float PD_T(float setpoint){ // PD caculation for translation
      float Error, Prev_Error, Correction, NewSpeed;
      
      Prev_Error = Error;
      Error = (Ping1 + Ping2)/2 - setpoint; 
      Correction = P_T*(Error) + D_T*(Error - Prev_Error);
      NewSpeed = BaseSpeed + Correction;
      return NewSpeed;
    }

    
    float PD_R(){ // PD caculation for rotation
      float Error, Prev_Error, Correction, NewSpeed;

      Prev_Error = Error; 
      Error = Ping1 - Ping2;
      Correction = P_R*(Error) + D_R*(Error - Prev_Error);
      NewSpeed = BaseSpeed + Correction; 
      return NewSpeed; //Positive is ClockWise
    }
};   
 
//intitiate sides:////////////////////////////////////////////////////
       //Pin1,Pin2, PT, DT, PR, DR, Dist1, Dist2
Side Front {0, 1, .1, .05, 0.08 , 0.08, 0, 0};
Side Back  {2, 3, 0, 0, 0, 0, 0, 0};
Side Arm   {4, 5, 1, .1, 0.1, 0.1, 0, 0};
Side Leg   {6, 7, 0, 0, 0, 0, 0, 0};

////////////////////////////////////////////////SETUP///////////////////////////////////////////////////////////////

void setup() {
  
  lcd.begin(16, 2); // set up the LCD's number of columns and rows
  lcd.print("IEEE TESTING");   // Print a message to the LCD.
  delay(2000);
  lcd.clear();
 
  for(int i=0;i<4;i++){  //configure pin modes
    pinMode (Motor[i].enablePin, OUTPUT);
    pinMode (Motor[i].directionPin, OUTPUT);
    pinMode (Motor[i].speedPin, OUTPUT);  
  }
}

 ////////////////////////////////////////////////////LOOP/////////////////////////////////////////////////////// 
void loop(){ 

  
Nav(30,0,30,0); //Navigate to (Front=30cm, Arm=30cm) and maintain 


}
///////////////////////////////////////////////////NAVIGATION/////////////////////////////////////////////////////////

void Nav(float F, float B, float A, float L){
  float numParameters = 0;     //number of non-Zero parameters 
  float Ns_Tfb = 0;           //new speed for front/back translation
  float Ns_Tal = 0;          //new speed for arm/leg translation
  float Ns_R = 0;          //new speed for all translation
  
while (1){ 
    if(F != 0){               // if passed a setpoint for Frontside
      numParameters++;
      Front.sensePings();
      Ns_Tfb = Front.PD_T(F);
      Ns_R = Front.PD_R();
       
    }
    
    else if(B != 0){        // if passed a setpoint for Backside
      numParameters++;
      Back.sensePings();
      Ns_Tfb = -Back.PD_T(B);
      Ns_R = -Back.PD_R();
    }
    
    if(A != 0){             // if passed a setpoint for Armside
      numParameters++;
      Arm.sensePings();
      Ns_Tal = Arm.PD_T(A);
      Ns_R += Arm.PD_R();
    }
    
    else if(L != 0){        // if passed a setpoint for Legside
      numParameters++;
      Leg.sensePings();
      Ns_Tal = -Leg.PD_T(L);
      Ns_R += -Leg.PD_R();
    }
    
  // Sum all speed caculations in proper orintation for mecanum drive
    Motor[0].Speed = Ns_Tfb + Ns_Tal - (Ns_R/numParameters);
    Motor[1].Speed = Ns_Tfb - Ns_Tal - (Ns_R/numParameters);
    Motor[2].Speed = Ns_Tfb - Ns_Tal + (Ns_R/numParameters);
    Motor[3].Speed = Ns_Tfb + Ns_Tal + (Ns_R/numParameters);
    
 // Exit condition
    if((abs(Ns_Tfb) + abs(Ns_Tal) + abs(Ns_R/numParameters)) <=20){
       stopDrive();
       printReadings();
       return;
    }
  
  // if given a final negative speed, make the final speed positive and reverse the wheel direction for each motor
    for(int i=0;i<4;i++){
      if (Motor[i].Speed < 0) {
        Motor[i].Direction = -Motor[i].Speed;
        Motor[i].Speed = 0;
      }
      else {
        Motor[i].Direction = LOW;
      }
    }
  
    // constrain the speeds of the motors to a value in between 0 and 255, the faster the motor can go is 255
    // if the speed is < 0 it gets set as 0, if the speed is > 255 it gets set as 255
    for(int i=0;i<4;i++){
      Motor[i].Speed = constrain(Motor[i].Speed,0,255);
      Motor[i].Direction = constrain(Motor[i].Direction,0,255);
    }
    printReadings(); //print the Sonar readings to the LCD screen
    setDrive(); //everything so far stored motor change data, now tell the motors to use that data
   
  }
}

 //////////////////////////////////////////////////Set/stop Drive/////////////////////////////////////////////////
void setDrive(){
  for(int i=0;i<4;i++){
     digitalWrite(Motor[i].enablePin, HIGH);
    digitalWrite(Motor[i].directionPin, Motor[i].Direction);
    digitalWrite(Motor[i].speedPin, Motor[i].Speed); 
  }
}

void stopDrive(){
   for(int i=0;i<4;i++){
    digitalWrite(Motor[i].enablePin, LOW);
  }
}

///////////////////////////////////////////////Print Sonar Readings////////////////////////////////////////////////
void printReadings() {
  //Function for debugging. It prints Front and Arm sonar reading to the LCD. 
  lcd.setCursor(0, 0);
  lcd.print("F  ");   
  lcd.print(Front.Ping1);
  
  lcd.setCursor(7, 0);
  lcd.print(" ");
  lcd.print(Front.Ping2);

  lcd.setCursor(0, 1);
  lcd.print("A  ");   
  lcd.print(Arm.Ping1);
  
  lcd.setCursor(7, 1);
  lcd.print(" "); 
  lcd.print(Arm.Ping2);
}



  

