#ifndef Gripperset_h
#define Gripperset_h

#include "Arduino.h"


 class Gripperset {
   public: 
    int SSL; //Slave select for the Alpha
    int SS_B; //Slave select for it's Beta
    int AlphaColors[3];  //Colors sent from Alpha to display
    int BetaColors[3]; //Beta Colors sent from Alpha. These will be displayed and sent to beta
    byte Alpha; //byte holding encoded colors for Alpha
    byte Beta; //byte holding encoded colors for Beta
    
    byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
    } 
      
    int buttonCheck(){ // Looking for Button on Alpha, telling Beta///////////////////
      byte checkA = 0; //message recieved from Alpha 
      int touchdown = 0;  
        digitalWrite(SSL, LOW);   
          checkA =transferAndWait ('s');  //request button condition
          checkA =transferAndWait ('s');  //recieve button condition 
          checkA =transferAndWait ('s');  //recieve button condition 
          //after frist transfer it could recieve on either transfer
        digitalWrite(SSL, HIGH);
        // Master will recieve B11110000, if button is pressed
        //Next we let the Beta know if the button was pressed
        
        digitalWrite(SS_B, LOW);   
          transferAndWait ('s'); //ready Beta for transfer
          transferAndWait (checkA);  //send it Alpha button condition
        digitalWrite(SS_B, HIGH);
        
        //check if the message meets condition 
        touchdown = (checkA == B11110000) ? 1 : 0;
        touchdown = 1;
        return touchdown; 
     }
    
    void senseColors(){ // Tell alpha to fire color sensors /////////////
        digitalWrite(SSL, LOW);   
          transferAndWait ('c');   //prep Alpha to take color readings
          transferAndWait (0);    //take color readings 
        digitalWrite(SSL, HIGH);
      
    }
    
    void manageColors(){ // Request color readings from Alpha ///////////
       byte decoder = B00110000;
       
        digitalWrite(SSL, LOW);   
          transferAndWait (1);  // request Alpha send packages
          transferAndWait (2);  //Alpha preps package[0]
          Alpha = transferAndWait (0); //recieve alpha colors, prep package[1]
          Beta = transferAndWait (0); // recieve beta colors 
        digitalWrite(SSL, HIGH); 

        
        for (int i=0;i<3;i++){ //unpack 3 alpha colors
          for(int j=4;i<3;j-2){ //change bit shifting by 2 each time
            //fill color array by viewing byte Alpha with the decoder mask
           
              AlphaColors[i] = (int)(Alpha & decoder); 
              AlphaColors[i]>>=j; //shift decoded number until its on 2 bits
              decoder >>= 2;  //shift the mask down to the next 2 bits
              
            }
        }
        decoder = B00110000; // reset decoder
        
        for (int i=0;i<3;i++){ // unpack beta colors as above
          for(int j=4;i<3;j-2){
            BetaColors[i] = Beta & decoder;
            BetaColors[i]>>=j;
            decoder >>= 2;
            
          }
        }
        //send beta colors to the beta slave
        digitalWrite(SS_B, LOW);   
          transferAndWait ('c'); //request to send beta its colors
          transferAndWait (Beta); //beta gets colors 
          transferAndWait (0); //tell beta to move on to managing colors and gripping
        digitalWrite(SS_B, HIGH);  
        
    }
    
    int holdCheck(){
      byte checkA = 0; byte checkB = 0;
      int holding = 0;  
        digitalWrite(SSL, LOW);   
          checkA =transferAndWait ('h');  //request gripping condition from Alpha
        digitalWrite(SSL, HIGH);
        
        digitalWrite(SS_B, LOW);   
          checkB =transferAndWait ('h');  //request gripping condition from Beta 
        digitalWrite(SS_B, HIGH);
        
        //then see if both Alpha and Beta send the "we have them" message
        holding = ((checkA == B00001111) & (checkB == B00001111)) ? 1 : 0;
        return holding;
    }
    
    void dropColor(byte x){ // Transfer a color to the Beta and Alpha //////////////
        digitalWrite(SS_B, LOW);   
          transferAndWait (x);  //send color and prep beta
          transferAndWait (0);  // beta executes drop
        digitalWrite(SS_B, HIGH);
        
        digitalWrite(SSL, LOW);   
          transferAndWait (x); //send color and prep alpha  
          transferAndWait (0);  // alpha executes drop
        digitalWrite(SSL, HIGH);
        
    }
    
    void dropAll(){ // Drops all Blocks (should be done before picking up new blocks)
      digitalWrite(SS_B, LOW);   
        transferAndWait ('a');  //prep beta
        transferAndWait (0);  //execute
      digitalWrite(SS_B, HIGH);
      
      digitalWrite(SSL, LOW);   
        transferAndWait ('a');   //prep alpha
        transferAndWait (0);    //execute
      digitalWrite(SSL, HIGH);
    }

    void expand(){
      digitalWrite(SS_B, LOW);   
        transferAndWait ('E');  //prep beta
        transferAndWait (0);  //execute
      digitalWrite(SS_B, HIGH);
      
      digitalWrite(SSL, LOW);   
        transferAndWait ('E');   //prep alpha
        transferAndWait (0);    //execute
      digitalWrite(SSL, HIGH);
    }

    void collapse(){
      digitalWrite(SS_B, LOW);   
        transferAndWait ('C');  //prep beta
        transferAndWait (0);  //execute
      digitalWrite(SS_B, HIGH);
      
      digitalWrite(SSL, LOW);   
        transferAndWait ('C');   //prep alpha
        transferAndWait (0);    //execute
      digitalWrite(SSL, HIGH);
    }
    
  };
  
  #endif