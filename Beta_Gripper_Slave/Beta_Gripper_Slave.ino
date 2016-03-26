#include <Servo.h> // for servo
#include <Wire.h> // for current sensing
#include "Beta.h"

#define EXPAND_SET 60 //servo position for expanding the gripper outer sliders
#define COLLAPSE_SET 120 //servo position for collapsing the gripper outer sliders

enum {
  RED,
  YELLOW,
  BLUE,
  GREEN
};

BetaGripper Finger[3] = {
// address,relax, Pin, setpt,  P,   D, AvgErr,AvgCur,  ,newPos
  {0x40,   0,     3,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0},
  {0x41,   0,     5,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}, 
  {0x44,   0,     9,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}
};

Servo assemblyServo;

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile byte colors = B11000000; //var recieves colors from master
volatile int task = 0; //which functions should it be completing 
volatile int dropColor; 
volatile int go = 0; // flag for having a grip on the blocks
volatile int expand = 0;
volatile int collapse = 0;

int myColors[3] = {RED, RED, RED};

void setup() {
  Serial.begin(115200);
  
  for(int i=0; i<3; i++){
    Finger[i].initSensors();
  }

  assemblyServo.attach(10);
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE); 
}

// SPI interrupt routine//////////////////////////////////////////////////////////////////////
ISR (SPI_STC_vect)
{
  byte c = SPDR;
 
  switch (command)
  {
  // no command? then this is the command ///////////////////////////
  case 0:
  command = c;
  SPDR = 0;
  break;  

  // Waiting for button pressed signal from alpha///////////////////
  case 's':
    command = 0;
    SPDR = 0;
    task = (c == B11110000) ? 1 : 0;
    break;

 // Get Colors from Alpha ///////////////////////////////////////////
  case 'c': 
    command = 1; // will be 1 next
    SPDR = 0;
    colors = c; 
    break;
 
// Set task to Manage colors    
  case 1: 
    command = 0; // will be 0 next
    SPDR = 0;
    task = 3; //do color tasks, then return to closed position (task 1)
    break;

// Checking for a Good Grip on All Blocks //////////////////////////   
  case 'h':
    command = c;
    SPDR = (go == 3) ? B00001111 : 0;
    break;  

// Tells the gripper assembly to expand //////////////////////////   
  case 'E':
    command = c;
    SPDR = 0;
    expand = 1;
    break;  
    
// Tells the gripper assembly to collapse//////////////////////////  
  case 'C':
    command = c;
    SPDR = 0;
    collapse = 1;
    break;  

 // DROP RED /////////////////////////////////
  case 'r':      
    command = c;
    dropColor = RED;
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    Finger[0].relax = ((myColors[0] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[0].relax = ((myColors[0] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

 // DROP YELLOW /////////////////////////////////      
  case 'y':       
    command = c;
    dropColor = YELLOW;
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    Finger[0].relax = ((myColors[0] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[0].relax = ((myColors[0] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

  // DROP BLUE ///////////////////////////////// 
  case 'b':       
    command = c;
    dropColor = BLUE;
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    Finger[0].relax = ((myColors[0] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[0].relax = ((myColors[0] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

  // DROP GREEN /////////////////////////////////     
  case 'g':        
    command = c;
    dropColor = GREEN;
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    Finger[0].relax = ((myColors[0] == dropColor)&&(myColors[1] == dropColor)) ? 1 : 0; 
    Finger[0].relax = ((myColors[0] == dropColor)&&(Finger[1].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

// DROP ALL ///////////////////////////////// (always do before getting new blocks)  
 case 'a': 
    command = c;
//    for(int i=0;i<3;i++){ //put guts in interrupt without function call
//    Finger[i].relax = 1; //in case hold function finishes running after interrupt
//    Finger[i].gripServo.write(READY_POS); //in case hold function does get to relax, before exiting task loop
//    //need to reset after all the blocks are dropped
//    }
    SPDR = 0;
    task = 0; //I think this is enough to drop all, if not uncomment above
    break;
    
  } // end of switch
} 
// END OF INTERUPT //////////////////////////////////////////////////////////////////


// LOOP////////////////////////////////////////////////////////////////////////////
  void loop() {
    
    
    if (task == 0){  // Defalt when Grippers are not being used ////////////////////////
      //everything is reset     
      for(int i=0;i<3;i++){ 
      Finger[i].relax = 0;
      Finger[i].gripServo.write(READY_POS); //need to reset after all the blocks are dropped
      Finger[i].NewPos = 90;
      Finger[i].grabbed = 0;
      }
      fill_arrays();
      expand_collapse_Check();
    }
  
    
    if(task == 1){ //Close fingers, take color readings and wait/////////////////////////////
      
      for(int i=0;i<3;i++){ 
      Finger[i].CloseToRead();
      }
      while(task == 1){
        //Don't be Rowdy
      } 
    }
    
    
    if(task == 2){ // PD-Controlled carrying blocks//////////////////////////////////////////
      
      for(int i=0;i<3;i++){ 
      Finger[i].holdBlocks();
      }
      go = (Finger[0].grabbed + Finger[1].grabbed + Finger[2].grabbed); // how many blocks grabbed?
      expand_collapse_Check();
    }
    
    if(task == 3){ // Decode the color byte sent from master via ALpha, store in array ///////////
        byte decoder = B00110000; // reset decoder
        
        for (int i=0;i<3;i++){
          for(int j=4;i<3;j-2){
            myColors[i] = (int)(colors & decoder);
            myColors[i]>>=j;
            decoder >>= 2;
          }
        }    
        task = 2; //After managing colors, begin to hold the blocks
    }
      
  }
//////////////////////////////////END OF LOOP/////////////////////////////////////////////////     
void fill_arrays() {
  for(int i=0; i<3; i++){
    Finger[i].fill();
  }
}
void expand_collapse_Check(){
       if (expand == 1){
         for(int pos=90;pos>EXPAND_SET;pos++){
            assemblyServo.write(pos);
            delay(1);
         }   
       }
       expand = 0;
       if (collapse == 1){
         for(int pos=90;pos<COLLAPSE_SET;pos++){
            assemblyServo.write(pos);
            delay(1);
       } 
       collapse = 0;
      }
  
}
//  END///////////



