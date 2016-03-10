#include <Servo.h> // for servo
#include <Wire.h> // for current sensing
#include <Adafruit_INA219.h> // for current sensing
#include "Adafruit_TCS34725.h" // for color sensor
// initialize the library with the numbers of the interface pins

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

#define COLOR_ARRAY_SIZE 20
#define CUR_ARRAY_SIZE 20
#define POS_ARRAY_SIZE 50

enum {
  RED,
  YELLOW,
  BLUE,
  GREEN
};

int BlockColors[6] = {0}; //colors of beta grippers 0-2, alpha colors 3-5

class AlphaGripper {
  public:
    uint8_t Cur_Address; //address of the current sensors
    int relax;  // flag for zero'd finger (no block)
    int ServoPin; //pin each servo communicates on 
    
    float setpoint; // current draw desired
    float P;   //controller gains
    float D;

      
    float current_history[CUR_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float Avg_current; 
    
    float Pos[POS_ARRAY_SIZE]; //current is bouncy, averaging is needed
    float NewPos; //after caculation, the new position to write to the servos 
    


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
      float Error, Prev_Error, Addition, NewPos;

      Prev_Error = Error;
      Error = currentSense() - setpoint;
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
      if (relax == 0) { //if not flagged

        NewPos += this->Gripper_PD(); //change in posistion

        NewPos = (NewPos < 0) ? 0 : NewPos; //NewPos should be positive;
        relax = this->zeroCheck(NewPos); // check the zero case
      }
      else if (this->zeroCheck(NewPos) == 1) NewPos = 100; //if no block, open back up
      else return 500;
      return NewPos;
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
        Avg_Var += abs(Array[k]);
      }
      Avg_Var = Avg_Var / Size;
      return Avg_Var;
    }

    void fill() {               //sets the error array very high
      for (int i = 0; i < POS_ARRAY_SIZE; i++) {
        current_history[i] = setpoint;
        Pos[i] = 90;
      } 
    }
};/////////////////END OF CLASS DEF//////////////////////////////////////


AlphaGripper Finger[3] = {
// address,relax, Pin, setpt,  P,   D,   ,AvgCur,  ,newPos
  {0x40,   0,     9,   300,  0.001, 0, {},  0,   {},   90},
  {0x41,   0,     6,   300,  0.001, 0, {},  0,   {},   90}, 
  {0x44,   0,     5,   300,  0.001, 0, {},  0,   {},   90}
};

void setup() {
  for(int i=0; i<3; i++){
    Finger[i].initSensors();
  }

  Serial.begin(115200);
  
}

void loop() {
  int Position = 90;
  Serial.println("Hello!"); 
  for(int i=0;i<3;i++){ //open for block placement 
  Finger[i].gripServo.write(Position);   
  } 
  delay(5000);
  while(1){
    for(int i=0;i<3;i++){ //open for block placement 
    Position = Finger[i].holdBlocks();  
    Finger[i].gripServo.write(Position);   
    } 
  }
}


void fill_current_arrays() {
  for(int i=0; i<3; i++){
    Finger[i].fill();
  }
}

////////////////////////////////////////////////////////Color///////////////////////////////////////////////////////////////
void getColors(){
void readColors() {
      uint16_t r, g, b, c, colorTemp;
      int R = 0; int B = 0; int Y = 0; int G = 0;
      uint16_t Color_Hist[COLOR_ARRAY_SIZE];
      uint16_t color;
      for(int j=0; j<6; j++){
        tcaselect(j);
          for (int i = 0; i < COLOR_ARRAY_SIZE; i++) { //ramp up to good avg
            tcs.getRawData(&r, &g, &b, &c);
            colorTemp = tcs.calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
          }
    
          for (int i = 0; i < COLOR_ARRAY_SIZE; i++) { //calculate the mode of 7 blocks
            tcs.getRawData(&r, &g, &b, &c);
            colorTemp = tcs.calculateColorTemperature(r, g, b);
            color = runningAvg(colorTemp, COLOR_ARRAY_SIZE, Color_Hist);
            if ((color >= 20000)) R++;
            else if ((color >= 9500) && (color < 20000)) B++;
            else if ((color >= 3500) && (color < 9500)) G++;
            else if ((color > 1000) && ( color < 3500 )) Y++;
          }
    
          if (B >= (COLOR_ARRAY_SIZE - 5)) BlockColors[j] = BLUE; 
    
          else if (G >= (COLOR_ARRAY_SIZE - 5)) BlockColors[j] = GREEN;
    
          else if (Y >= (COLOR_ARRAY_SIZE - 5)) BlockColors[j] =YELLOW;
    
          else BlockColors[j] = RED;
        }
    }
    float runningAvg(uint16_t Var, int Size, uint16_t *Array) {
      float Avg_Var;
      for (int j = Size - 1; j > 0; j--) { //Shifts
        Array[j] = Array[j - 1];
      }
      Array[0] = Var;

      Avg_Var = 0;                                           //Sums/w abs() and finds Average
      for (int k = 0; k < Size; k++) {
        Avg_Var += abs(Array[k]);
      }
      Avg_Var = Avg_Var / Size;
      return Avg_Var;
}

void tcaselect(uint8_t i){
  if(i>7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1<<i);
  Wire.endTransmission();
}



