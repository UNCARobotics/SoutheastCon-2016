#include <Servo.h> // for servo
#include <Wire.h> // for current sensing
#include <Adafruit_INA219.h> // for current sensing

#define CUR_ARRAY_SIZE 20
#define POS_ARRAY_SIZE 50
#define READY_POS 100
#define COLOR_READ_POS 40
#define ER_ARRAY_SIZE 30

enum {
  RED,
  YELLOW,
  BLUE,
  GREEN
};


class AlphaGripper {
  public:
    uint8_t Cur_Address; //address of the current sensors
    volatile int relax;  // flag for zero'd finger (no block)
    int ServoPin; //pin each servo communicates on 
    
    
    float setpoint; // current draw desired
    float P;   //controller gains
    float D;
    float Error;
    float Error_History[ER_ARRAY_SIZE];
      
    float current_history[CUR_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float Avg_current; 
    
    float Pos[POS_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float NewPos; //after caculation, the new position to write to the servos 
    
    int grabbed;

    Servo gripServo;     // a servo for the grippers
    Adafruit_INA219 ina219; // a current sensor 
    
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

      Prev_Error = Error;
      Error = currentSense() - setpoint;
      Avg_Error = runningAvg(Error, ER_ARRAY_SIZE, Error_History);
      grabbed = (Avg_Error < 50) ? 1 : 0;
      Addition = P * (Error) + D * (Error - Prev_Error);
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
      for(int i=0;i<3;i++){ //open for block placement  
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

    void fill() {               //sets the error array very high
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
};/////////////////END OF CLASS DEF//////////////////////////////////////


AlphaGripper Finger[3] = {
// address,relax, Pin, setpt,  P,   D, AvgErr,AvgCur,  ,newPos
  {0x40,   0,     9,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0},
  {0x41,   0,     6,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}, 
  {0x44,   0,     5,   300,  0.001, 0, 0, {},      {},  0,   {},   90, 0}
};

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile byte colors = B11000000;
volatile int task = 0; //which functions should it be completing 
volatile int dropColor;
volatile int go = 0;
bool button; 
byte package[2] = {0};

int myColors[3] = {RED, RED, RED};

void setup() {
  Serial.begin(115200);
  
  for(int i=0; i<3; i++){
    Finger[i].initSensors();
  }
  pinMode(A1, INPUT);
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
  
  case 's':
    command = c;
    SPDR = (button == HIGH) ? B11110000 : 0; 
    task = (button == HIGH) ? 1 : 0;
    break;
    
  case 'c': //Close and read Color
    command = 1;
    SPDR = 0;
    colors = c; 
    break;
    
  case 1: //Close and read Color
    command = 0;
    SPDR = 0;
    task = 3; 
    break;

  case 'h':
    command = c;
    SPDR = (go == 3) ? B00001111 : 0;
    break;  
 
  case 'r': //DROP RED       
    command = c;
    dropColor = RED;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    break;
    
  case 'y': //DROP YELLOW       
    command = c;
    dropColor = YELLOW;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    break;
    
  case 'b': //DROP BLUE         
    command = c;
    dropColor = BLUE;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    break;
    
  case 'g': //DROP GREEN        
    command = c;
    dropColor = GREEN;
    Finger[0].relax = (myColors[0] == dropColor) ? 1 : 0; //put guts in interrupt without function call
    Finger[1].relax = (myColors[1] == dropColor) ? 1 : 0;
    
    //special condition for if top block is blocked... pun intended?
    Finger[2].relax = ((myColors[2] == dropColor)&&(myColors[0] == dropColor)) ? 1 : 0; 
    Finger[2].relax = ((myColors[2] == dropColor)&&(Finger[0].relax == 1)) ? 1 : 0;
    break;
    
 case 'a': //DROP ALL
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
}  // end of Interupt //////////////////////////////////////////////////////////////////




  void loop() {//////////////LOOOOOOOOOOOOOP///////////////////////////////////////////
    
    //Defalt when Grippers are not being used
    if (task == 0){  
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
  
    //Close fingers, take color readings and wait
    if(task == 1){
      for(int i=0;i<3;i++){ //put guts in interrupt without function call
      Finger[i].CloseToRead();
      }
      while(task == 1){
        //Don't be Rowdy
      } 
    }
    
    // PD-Controlled carrying blocks
    if(task == 2){
      for(int i=0;i<3;i++){ //put guts in interrupt without function call
      Finger[i].holdBlocks();
      button = LOW;
      }
      go = (Finger[0].grabbed + Finger[1].grabbed + Finger[2].grabbed);
    }
    if(task == 3){
        byte decoder = B00110000; // reset decoder
        
        for (int i=0;i<3;i++){
          for(int j=4;i<3;j-2){
            myColors[i] = (int)(colors & decoder);
            myColors[i]>>=j;
            decoder >>= 2;
          }
        }    
      task = 2;
    }
      
  }
//////////////////////////////////END OF LOOP/////////////////////////////////////////////////     
void fill_arrays() {
  for(int i=0; i<3; i++){
    Finger[i].fill();
  }
}





