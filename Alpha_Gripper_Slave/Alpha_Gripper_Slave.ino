#include <Servo.h> // for servo
#include <Wire.h> // for current sensing
#include "Adafruit_TCS34725.h" // for color sensor
#include "Alpha.h"


//create an instance of a color sensor
Adafruit_TCS34725 tcs[6] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); 

enum { //place holder for colors RED = int 0, Yellow = int 1, etc...makes reading code easier
  RED,
  YELLOW,
  BLUE,
  GREEN
};

////////////////////////////////////////


AlphaGripper Finger[3] = {
// address,relax, Pin, setpt,  P,   D, AvgErr,AvgCur,  ,newPos
  {0x40,   0,     3,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0},
  {0x41,   0,     5,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}, 
  {0x44,   0,     6,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}
};

// SETUP//////////////////////////////////////////////////////////////////////////////
volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile int task = 0; //which functions should it be completing 
volatile int dropColor;
volatile int go = 0;

volatile int expand = 0;
volatile int collapse = 0;
bool count = 0;

bool button = HIGH; 
byte package[2] = {0};

int AllColors[6] = {BLUE, RED, BLUE, GREEN, GREEN, YELLOW}; //colors of alpha grippers 0-2, beta colors 3-5
int myColors[3] = {BLUE, RED, BLUE}; // personal colors gripper looks at to drop blocks
// init BLUE for competition (best odds)

int lastButtonReading = HIGH;
long lastDebounceTime = 0;  


  

void setup() {
  Serial.begin(115200);
  Serial.println("Begin /////////////////////////");
  delay(1000);
  for(int i=0; i<3; i++){
    Finger[i].initSensors();
  }
  
  pinMode(A1, INPUT); //button
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT_PULLUP);

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
  // no command? then this is the command
  case 0:
  command = c;
  SPDR = 0;
  break;  

  // Looking for Button///////////////////
  case 's':  
    command = c;
    //SPDR = (button == LOW) ? B11110000 : 0; //if button, send flag
    SPDR = 10101010;
    task = (button == LOW) ? 1 : 0;  //if button, close read
    break;

  // Take Color Readings and Create Packages //
  case 'c': 
    command = c; //will be 1 next
    SPDR = 0;
    task = 3; //do color tasks, then return to closed position (task 1)
    break;

  // Send Alpha Colors to Master ///////////////////////
  case 1: 
    command = c; // will be 2 next
    SPDR = package[0];  //Alpha colors
    break;

// Send Beta Colors to Master //////////////////////////    
  case 2: 
    command = c; // will be 0 next
    SPDR = package[1]; //Beta colors
    task = 2; //After reading and sending colors, Start PD_Gripping //
    break;
    
// Checking for a Good Grip on All Blocks //    
  case 'h':
    command = c;
    SPDR = (go == 3) ? B00001111 : 0; //if all three are flagged, send 'ready'
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
    //Code for NCUR DEMO
      fill_arrays();
      for(int i=0;i<3;i++){ 
        Finger[i].gripServo.write(READY_POS);
      }
      while(1){
        if (button == HIGH){
          
            int reading = digitalRead(A1); //take a button reading
            // Debounce code
            if (reading != lastButtonReading) lastDebounceTime = millis();
            if ((millis() - lastDebounceTime) > 50){
               if (button != reading){ 
                  button = reading;
               }
            }   
            lastButtonReading = reading;
            // End Debounce 
        }
        else{
            for(int i=0;i<3;i++){
              Finger[i].holdBlocks();
            }  
        }
      }
      //END NCUR DEMO
      
//    if (task == 0){  // Defalt when Grippers are not being used ////////////////////////
//      if (button == HIGH){
//        
//          int reading = digitalRead(A1); //take a button reading
//          // Debounce code
//          if (reading != lastButtonReading) lastDebounceTime = millis();
//          if ((millis() - lastDebounceTime) > 50){
//             if (button != reading){ 
//                button = reading;
//             }
//          }   
//          lastButtonReading = reading;
//          // End Debounce 
//      }
//      
//      //everything is reset     
//      for(int i=0;i<3;i++){ 
//      Finger[i].relax = 0;
//      Finger[i].gripServo.write(READY_POS); //need to reset after all the blocks are dropped
//      Finger[i].NewPos = 90;
//      Finger[i].grabbed = 0;
//      }
//      fill_arrays();
//    }
//  
//    
//    if(task == 1){ // Close fingers and wait ///////////////////////
//     
//      
//      Finger[1].CloseToRead();
//      
//      while(task == 1){
//        //Don't be Rowdy
//      } 
//    }
//    
//    
//    if(task == 2){ // PD-Controlled carrying blocks //////////////////////////////////////
//      for(int i=0;i<3;i++){
//        Serial.print(" "); Serial.print(i); Serial.print(":"); 
//        Finger[i].holdBlocks();
//      }
//      Serial.println();
//      button = LOW; // reset button
//      go = (Finger[0].grabbed + Finger[1].grabbed + Finger[2].grabbed); // how many blocks grabbed?
//    }
//    
//    if(task == 3){ // Take Color Readings on ALPHA & BETA, and Package results
//      getColors();
//      task = 1; //After taking a reading, re-ready fingers and wait
//    }
//      
    }
// END OF LOOP////////////////////////////////////////////////////////////////////////////////     
void fill_arrays() { //function resets all arrays for each finger
  for(int i=0; i<3; i++){
    Finger[i].fill();
  }
}

// COLOR FUNCTIONS ///////////////////////////////////////////////////////////////////////////

void getColors(){
  Finger[1].CloseToRead();
  readColors(); //Fills all colors array with new colors
  
  for(int i=0;i<3;i++){  //store colors for this gripper in exclusive array. 
     myColors[i] = AllColors[i];
  }
  
  package[0] = (AllColors[0]<<4) | (AllColors[1]<<2) | (AllColors[2]) | B11000000; //package beta colors
  package[1] = (AllColors[3]<<4) | (AllColors[4]<<2) | (AllColors[5]) | B11000000; //package
  // the leading 2 bits are a tag. After that each 2 bits represents a color
}

void readColors() {
      uint16_t r, g, b, c, colorTemp; // values used by Color sensor

      uint16_t color; //Avg and final color
      uint16_t Color_Hist[COLOR_ARRAY_SIZE];
      int R = 0; int B = 0; int Y = 0; int G = 0; //Counters for readings

      
      for(int j=0; j<6; j++){   //take readings for all six color sensors (3Alpha & 3BETA)
        tcaselect(j); //MUX select function 
        
        //fill array with new readings to get good avg
          for (int i = 0; i < COLOR_ARRAY_SIZE; i++) {
            tcs[j].getRawData(&r, &g, &b, &c);
            colorTemp = tcs[j].calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
          }

        //calculate the mode of 7 blocks
          for (int i = 0; i < COLOR_ARRAY_SIZE; i++) { 
            tcs[j].getRawData(&r, &g, &b, &c);
            colorTemp = tcs[j].calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
            Serial.print(color); Serial.print(" ");
          // count readings as they come in  
            if ((color >= 20000)) R++;
            else if ((color >= 9500) && (color < 20000)) B++;
            else if ((color >= 3500) && (color < 9500)) G++;
            else if ((color > 1000) && ( color < 3500 )) Y++;
          }
          Serial.println();      
          Serial.print("R: "); Serial.print(R); Serial.print(" ");
          Serial.print("B: "); Serial.print(B); Serial.print(" ");
          Serial.print("G: "); Serial.print(G); Serial.print(" ");
          Serial.print("Y: "); Serial.print(Y); Serial.print("... ");

          if (B>=(COLOR_ARRAY_SIZE-5)) Serial.print("Blue Block ");
 
          else if (G>=(COLOR_ARRAY_SIZE-5)) Serial.print("Green Block ");
    
          else if (Y>=(COLOR_ARRAY_SIZE-5)) Serial.print("Yellow Block ");
    
          else Serial.print("Red Block ");

          // make decision based on Mode of Avg readings
          if (B >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] = BLUE; 
          else if (G >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] = GREEN;
          else if (Y >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] =YELLOW;
          else AllColors[j] = RED;
          R=0; Y=0; B=0; G=0;
          Serial.println(j);  
          delay(1000);
           }
       
    } // All colors have been read
    
 float runningAvg(uint16_t Var, int Size, uint16_t *Array) {
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
///////////////////////////////////////////////////////////////////////////////////////////////
void tcaselect(uint8_t i){ //MUX function
  if(i>7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1<<i);
  Wire.endTransmission();
}




