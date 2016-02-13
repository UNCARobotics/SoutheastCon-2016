
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
Side Back  {2, 3, .1, .05, 0.08 , 0.08, 0, 0};
Side Arm   {4, 5, 1, .1, 0.1, 0.1, 0, 0};
Side Leg   {6, 7, 1, .1, 0.1, 0.1, 0, 0};







void setup() {
  
  lcd.begin(16, 2); // set up the LCD's number of columns and rows
  lcd.print("IEEE TESTING");   // Print a message to the LCD.
  delay(2000);
  lcd.clear();
 
  for(int i=0;i<4;i++){  //configure pin modes
    pinMode (Motor[i].enablePin, OUTPUT);
    pinMode (Motor[i].backwardSpeedPin, OUTPUT);
    pinMode (Motor[i].forwardSpeedPin, OUTPUT);  
  }
}

void loop() {
  // put your main code here, to run repeatedly:
Front.sensePings();
printReadings();


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
