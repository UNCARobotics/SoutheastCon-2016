  #include <Servo.h> // for servo
  #include <Wire.h> // for current sensing
  #include <Adafruit_INA219.h> // for current sensing
  
  #define CUR_ARRAY_SIZE 20
  #define POS_ARRAY_SIZE 50
  
  //making instances of each current sensor
  Adafruit_INA219 ina219_A;
  Adafruit_INA219 ina219_B(0x41);
  Adafruit_INA219 ina219_C(0x44);
  
  //instance of each servo
  Servo gripServo[3];
  
  class Fingers {
    public:
      int fingerNum;   // identifies which current sensor to use 
      
      float setpoint; // current draw desired
      float P;   //controller gains
      float D;
      
      
      float current_history[CUR_ARRAY_SIZE]; //current is bouncy, averaging is needed
      float Pos[POS_ARRAY_SIZE]; //current is bouncy, averaging is needed
      float Avg_current; 
  
      float currentSense(){                 //get recent avg current
          float current; // current measured 
          
          // get current from corresponding sensor 
          if (fingerNum==0) current = ina219_A.getCurrent_mA();
          else if (fingerNum == 1) current = ina219_B.getCurrent_mA();
          else current = ina219_C.getCurrent_mA();
          
          //Average current with past currents
          for(int i=CUR_ARRAY_SIZE-1; i>0; i--){   //Shifts        
                  current_history[i] = current_history[i-1];
          }
                current_history[0] = current;
                                                   //Sums/w abs() and finds Average
          for(int k=0;k<CUR_ARRAY_SIZE;k++){
                  Avg_current += abs(current_history[k]);
           }
                Avg_current = Avg_current/CUR_ARRAY_SIZE;
          return Avg_current;
      }
      
      float Gripper_PD(){ // PD caculation for translation
          float Error, Prev_Error, Addition, NewPos;
          
          Prev_Error = Error;
          Error = currentSense() - setpoint;
          Addition = P*(Error) + D*(Error - Prev_Error);
          return Addition;
      }

      int zeroCheck(float ThisPos){ // check to see if the finger is zero'd out (no block)
        float Avg_Pos;
        int flag = 0;
        
          for(int i=POS_ARRAY_SIZE-1; i>0; i--){   //Shifts        
              Pos[i] = Pos[i-1];
          }
          Pos[0] = ThisPos;
                                                   //Sums/w abs() and finds Average
          for(int k=0;k<POS_ARRAY_SIZE;k++){
                  Avg_Pos += abs(Pos[k]);
           }
           Avg_Pos = Avg_Pos/POS_ARRAY_SIZE;   
           flag = (Avg_Pos) ? 0 : 1;
           return flag;     
      }
      
       void fill(){                //sets the error array very high
            for(int i=0;i<POS_ARRAY_SIZE;i++){
               current_history[i]=setpoint;
               Pos[i] = 90;
            }
       }
       
                                      

  };/////////////////END OF CLASS DEF//////////////////////////////////////
  // creating 3 fingers for the gripper 
  Fingers Finger[3] = {
//  num, setpt,  P,  D     
    {0,  300,  .001, 0, {}, {}, 0},
    {1,  300,  .001, 0, {}, {}, 0},
    {2,  300,  .001, 0, {}, {}, 0}
  };
  
  void setup() {
    gripServo[0].attach(9); 
    gripServo[1].attach(6);  // attaches the servo on pin 9 to the servo object
    gripServo[2].attach(5);
    
    Serial.begin(115200);
    Serial.println("Hello!");
                          
   // Initialize the INA219. By default, largest range = (32V, 2A). Can change with setCalibration function
   ina219_A.begin();
   ina219_B.begin();
   ina219_C.begin();
  }
  
  void loop() {
      for(int i=0;i<3;i++){ //open for block placement 
      gripServo[i].write(90);   
      } 
      delay(5000);
      
      holdBlocks();
  }
  
  void holdBlocks(){
    float NewPos[3] = {90, 90 , 90}; // var for change in servo pos
    int relax[3] = {0};  // flag for zero'd finger (no block)
    
    fill_current_arrays(); // reset avg arrays
    
    while(1){
        for(int i=0;i<3;i++){   //for each finger run PD caculation
          if(relax[i] == 0){  //if not flagged
            
            NewPos[i] += Finger[i].Gripper_PD(); //change in posistion
            
            NewPos[i] = (NewPos[i]<0) ? 0 : NewPos[i]; //NewPos should be positive;
            relax[i] = Finger[i].zeroCheck(NewPos[i]); // check the zero case
          }
          else NewPos[i] = 100; //if no block, open back up                           
        }
                                       
        for(int i=0;i<3;i++){
          gripServo[i].write(NewPos[i]);   // apply caculations
        }
    }
  }
    
 void fill_current_arrays(){
    Finger[0].fill();
    Finger[1].fill();
    Finger[2].fill();
  }



  
//                                      Serial.print(i);  
//                                      Serial.print("  OldPos: ");
//                                      Serial.print(NewPos[i]);  
//                                      Serial.print("  NewPos: "); 
//                                      Serial.print(NewPos[i]);
//                                      Serial.print("  Avg Current: "); 
//                                      Serial.print(Finger[i].currentSense());
//                                      Serial.println();
