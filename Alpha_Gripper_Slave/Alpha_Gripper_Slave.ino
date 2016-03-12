#include <Servo.h> // for servo
#include <Wire.h> // for current sensing
#include <Adafruit_INA219.h> // for current sensing
#include "Adafruit_TCS34725.h" // for color sensor


//create an instance of a color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); 

#define COLOR_ARRAY_SIZE 20  //array for averaging color
#define CUR_ARRAY_SIZE 20    //array for averaging current
#define POS_ARRAY_SIZE 50   //array for averaging Servo Position
#define READY_POS 100 //open and ready for picking up
#define COLOR_READ_POS 40 //mostly closed to read the color sensor
#define ER_ARRAY_SIZE 30  //array for averaging error

enum { //place holder for colors RED = int 0, Yellow = int 1, etc...makes reading code easier
  RED,
  YELLOW,
  BLUE,
  GREEN
};

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
};// END OF CLASS DEF/////////////////////////////////////////////////////////////


AlphaGripper Finger[3] = {
// address,relax, Pin, setpt,  P,   D, AvgErr,AvgCur,  ,newPos
  {0x40,   0,     9,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0},
  {0x41,   0,     6,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}, 
  {0x44,   0,     5,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}
};

// SETUP//////////////////////////////////////////////////////////////////////////////
volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile int task = 0; //which functions should it be completing 
volatile int dropColor;
volatile int go = 0;
bool button; 
byte package[2] = {0};

int AllColors[6] = {0}; //colors of alpha grippers 0-2, beta colors 3-5
int myColors[3] = {RED, RED, RED}; // personal colors gripper looks at to drop blocks
// init BLUE for competition (best odds)

void setup() {
  Serial.begin(115200);
  
  for(int i=0; i<3; i++){
    Finger[i].initSensors();
  }
  pinMode(A1, INPUT); //button
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
  // no command? then this is the command
  case 0:
  command = c;
  SPDR = 0;
  break;  

  // Looking for Button///////////////////
  case 's':  
    command = c;
    SPDR = (button == HIGH) ? B11110000 : 0; //if button, send flag
    task = (button == HIGH) ? 1 : 0;  //if button, close read
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
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;
    
// DROP YELLOW /////////////////////////////////    
  case 'y':      
    command = c;
    dropColor = YELLOW;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

// DROP BLUE /////////////////////////////////    
  case 'b':         
    command = c;
    dropColor = BLUE;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    SPDR = 0;
    break;

// DROP GREEN /////////////////////////////////    
  case 'g':         
    command = c;
    dropColor = GREEN;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
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
      if (button == LOW){
        button == digitalRead(A1);
      }
      //everything is reset     
      for(int i=0;i<3;i++){ 
      Finger[i].relax = 0;
      Finger[i].gripServo.write(READY_POS); //need to reset after all the blocks are dropped
      Finger[i].NewPos = 90;
      Finger[i].grabbed = 0;
      }
      fill_arrays();
    }
  
    
    if(task == 1){ // Close fingers and wait ///////////////////////
     
      for(int i=0;i<3;i++){ 
      Finger[i].CloseToRead();
      }
      while(task == 1){
        //Don't be Rowdy
      } 
    }
    
    
    if(task == 2){ // PD-Controlled carrying blocks //////////////////////////////////////
      
      for(int i=0;i<3;i++){ 
      Finger[i].holdBlocks();
      button = LOW; // reset button
      }
      go = (Finger[0].grabbed + Finger[1].grabbed + Finger[2].grabbed); // how many blocks grabbed?
    }
    
    if(task == 3){ // Take Color Readings on ALPHA & BETA, and Package results
      getColors();
      task == 1; //After taking a reading, re-ready fingers and wait
    }
      
  }
// END OF LOOP////////////////////////////////////////////////////////////////////////////////     
void fill_arrays() { //function resets all arrays for each finger
  for(int i=0; i<3; i++){
    Finger[i].fill();
  }
}

// COLOR FUNCTIONS ///////////////////////////////////////////////////////////////////////////
void getColors(){
  
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
            tcs.getRawData(&r, &g, &b, &c);
            colorTemp = tcs.calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
          }

        //calculate the mode of 7 blocks
          for (int i = 0; i < COLOR_ARRAY_SIZE; i++) { 
            tcs.getRawData(&r, &g, &b, &c);
            colorTemp = tcs.calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
          
          // count readings as they come in  
            if ((color >= 20000)) R++;
            else if ((color >= 9500) && (color < 20000)) B++;
            else if ((color >= 3500) && (color < 9500)) G++;
            else if ((color > 1000) && ( color < 3500 )) Y++;
          }

          // make decision based on Mode of Avg readings
          if (B >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] = BLUE; 
          else if (G >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] = GREEN;
          else if (Y >= (COLOR_ARRAY_SIZE - 5)) AllColors[j] =YELLOW;
          else AllColors[j] = RED;
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



