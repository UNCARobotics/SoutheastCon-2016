#ifndef Sides_h
#define Sides_h

#include "Arduino.h"

float BaseSpeed = 0;

class Side { 
    public:
        
        float P_T;      // PD constants
        float D_T;
        float P_R;
        float D_R;
        
        // found values
        float Ping1;  //for storing distance
        float Ping2;
        
        float Error_T;
        float Error_R;
        
        float Error_T_History[ER_ARRAY_SIZE];
        float Error_R_History[ER_ARRAY_SIZE];
        float Avg_ErT;
        float Avg_ErR;

        int SSL;
        //////////////////////////////////////////////////
                            //sense distance for each side
        void sensePings() {                
         digitalWrite(SSL, LOW);  // open communication (direct slave into interupt routine)
           transferAndWait ('p');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (2);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           Ping1 = ((int)transferAndWait(3) << 8) | (int)transferAndWait(4); 
           Ping2 = ((int)transferAndWait(0) << 8) | (int)transferAndWait(0); 
         digitalWrite(SSL, HIGH); // close communication, but pings will continue to read
        }
        
        void senseTruckPings(byte x) {                 
         digitalWrite(SSL, LOW);  // open communication (direct slave into interupt routine)
         if (x = 'r'){
           transferAndWait ('r');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (2);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           Ping1 = ((int)transferAndWait(3) << 8) | (int)transferAndWait(4);  
           Ping2 = ((int)transferAndWait(0) << 8) | (int)transferAndWait(0); 
         }
         if (x = 'l'){
           transferAndWait ('l');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (6);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           Ping1 = ((int)transferAndWait(7) << 8) | (int)transferAndWait(8);  
           Ping2 = ((int)transferAndWait(0) << 8) | (int)transferAndWait(0); 
         }
         digitalWrite(SSL, HIGH); // close communication, but pings will continue to read
        }
        
        void stopPings(){
          digitalWrite(SSL, LOW);   
          transferAndWait ('q');  // add command 
          transferAndWait (2);  // add command
          digitalWrite(SSL, HIGH);
          
        }
       byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
      } 
        
        float PD_T(float setpoint){ // PD caculation for translation
          float Prev_Error, Correction, NewSpeed;
          int T = 1; // condition for averaging translation in Avg_error function
          
          Prev_Error = Error_T;
          Error_T = (Ping1 + Ping2)/2 - setpoint; 
          Avg_Error(Error_T, T);
          Correction = P_T*(Error_T) + D_T*(Error_T - Prev_Error);
          NewSpeed = BaseSpeed + Correction;
         
          return NewSpeed;
        }

        float Truck_PD(float setpoint){
          float Prev_Error, Correction, NewSpeed;
          int T = 1; // condition for averaging translation in Avg_error function
          
          Prev_Error = Error_T;
          Error_T = (Ping2 - Ping1) - setpoint; 
          Avg_Error(Error_T, T);
          Correction = P_T*(Error_T) + D_T*(Error_T - Prev_Error);
          NewSpeed = BaseSpeed + Correction;
         
          return NewSpeed;
        }

        float Truck_Arm_PD(bool mirror){
          float setpoint = 400;
          float Prev_Error, Correction, NewSpeed;
          int T = 1; // condition for averaging translation in Avg_error function
          
          Prev_Error = Error_T;
          if (mirror == 1) Error_T = Ping2 - setpoint; 
          else Error_T = Ping1 - setpoint;
          Avg_Error(Error_T, T);
          Correction = P_T*(Error_T) + D_T*(Error_T - Prev_Error);
          NewSpeed = BaseSpeed + Correction;
         
          return NewSpeed;
        }
    
        float PD_R(){ // PD caculation for rotation
          float Prev_Error, Correction, NewSpeed;
          int R = 0;  // condition for averaging Rotation in Avg_error function
    
          Prev_Error = Error_R; 
          Error_R = -(Ping1 - Ping2);
          Avg_Error(Error_R, R);
          Correction = P_R*(Error_R) + D_R*(Error_R - Prev_Error);
          NewSpeed = BaseSpeed + Correction; 
          return NewSpeed; //Positive is ClockWise
        }


        //This is for the exit conditions.
        void Avg_Error(float Er, int type){    //Shifts the array of errors then adds the most recent to the 0 spot
          if (type == 1){                      //Translation                                        
            for(int i=ER_ARRAY_SIZE-1; i>0; i--){   //Shifts        
              Error_T_History[i] = Error_T_History[i-1];
            }
            Error_T_History[0] = Er;
                                               //Sums/w abs() and finds Average
            for(int k=0;k<ER_ARRAY_SIZE;k++){
              Avg_ErT += abs(Error_T_History[k]);
            }
            Avg_ErT = Avg_ErT/ER_ARRAY_SIZE;
          }
          else if(type==0){                   //Rotation
            for(int i=ER_ARRAY_SIZE-1; i>0; i--){  //Shifts
              Error_R_History[i] = Error_R_History[i-1];
            }
            Error_R_History[0] = Er;
            for(int k=0;k<ER_ARRAY_SIZE;k++){       //Sums/w abs() and finds Average
              Avg_ErR += abs(Error_T_History[k]);
            }
            Avg_ErR = Avg_ErR/ER_ARRAY_SIZE;
          }
        }

        float getAvg_ErT(){     //needed to send Error Avg because PD function can't return 2 things
          return Avg_ErT;
        }
        float getAvg_ErR(){     //needed to send Error Avg because PD function can't return 2 things
          return Avg_ErR;
        }
        
        void fill(){                //sets the error array very high
          for(int i=0;i<ER_ARRAY_SIZE;i++){
             Error_T_History[i]=100;
             Error_R_History[i]=100;
          }
        }

  };  

  #endif
