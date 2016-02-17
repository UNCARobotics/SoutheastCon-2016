// include the library code:
#include <LiquidCrystal.h> // for lcd screen
#include "Adafruit_TCS34725.h" // for color sensing
#include <Wire.h> // for current sensing DO WE NEED THIS? IT COMPILES WITHOUT IT
#include <Adafruit_INA219.h> // for current sensing

Adafruit_INA219 ina219;
#if defined(ARDUINO_ARCH_SAMD)  // for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define Serial SerialUSB
#endif

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

///////////////////////////////////////////END OF INITIALIZATION///////////////////////////////////////////////////

class Grippers {  /////////////////////////GRIPPER CLASS DEFINITION//////////////////////////////////////////////////
  public:
  
  struct Blocks {
    char Color;
    int Address; // address of Flora sensor
    uint16_t colorTemp, r, g, b;
  };


  //Block positions declarations (Picture below shows block numbers)
  // |1|    |3|
  // | |    |2|
  //      bot side of gripper
  
  Blocks Block[3] ={
    {' ', 0 , 0, 0, 0, 0}, // add addresses
    {' ', 0 , 0, 0, 0, 0},
    {' ', 0, 0, 0, 0, 0}
  };  

  void getColor() {
  //take color reading  
    for(int i=0;i<4;i++){
      tcs.getRawData(&Block[i].r, &Block[i].g, &Block[i].b);
      Block[i].colorTemp = tcs.calculateColorTemperature(Block[i].r, Block[i].g, Block[i].b);
        
        //red
        if ( (Block[i].colorTemp > 4500) && (Block[i].colorTemp < 9000) ) {   
        }
        
        //blue
        else if ( (Block[i].colorTemp > 9000) && (Block[i].colorTemp < 13000) ){
        }
        
        //green
        else if ( (Block[i].colorTemp > 3000) && (Block[i].colorTemp < 4500) ){
        }
        
        //yellow
        else if ( (Block[i].colorTemp > 2000) && ( Block[i].colorTemp < 3000 ) ){
        }
        
        //ambient
        else {
         }
    }   
  }

};
//inititate grippers:////////////////////////////////////////////////////////////////////////////////////////

Grippers Gripper[4] ;

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("IEEE TESTING");
  Wire.begin();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}

