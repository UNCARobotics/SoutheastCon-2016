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

  uint16_t r, g, b, c, colorTemp;
   
  struct Blocks {
    char Color;
    int Address; // address of Flora sensor
   
  };

  //Block positions declarations (Picture below shows block numbers)
  // |1|    |3|
  // | |    |2|
  //      bot side of gripper
  
  Blocks Block[3] ={
    {' ', 0}, // add addresses
    {' ', 0},
    {' ', 0}
  };  


//// my idea is you go through each all blocks for each gripper, so you go into first gripper and take colors by stepping through blocks
//// and then go into next gripper
  void getColor() {
  //take color reading  
    for(int i=0;i<3;i++){
      tcs.getRawData(&r, &g, &b, &c);
      
      colorTemp = tcs.calculateColorTemperature(r, g, b);
        
        //red
        if ( (colorTemp > 4500) && (colorTemp < 9000) ) {   
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Red Block");
          Block[i].Color = 'red';
        }
        
        //blue
        else if ( (colorTemp > 9000) && (colorTemp < 13000) ){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Blue Block");
          Block[i].Color = 'blue';
        }
        
        //green
        else if ( (colorTemp > 3000) && (colorTemp < 4500) ){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Green Block");
          Block[i].Color = 'green';
        }
        
        //yellow
        else if ( (colorTemp > 2000) && ( colorTemp < 3000 ) ){
           lcd.clear();
           lcd.setCursor(0,0);
           lcd.print("Yellow Block");
           Block[i].Color = 'yellow';
        }
        
        //ambient
        else {
           lcd.clear();
           lcd.setCursor(0,0);
           lcd.print("Ambient");
         }
    }   
  }

};
//inititate grippers:////////////////////////////////////////////////////////////////////////////////////////
Grippers Gripper[4];

//Gripper positions declarations (Picture below shows gripper numbers, from front of bot)
// |1| |2| |3| |4|
 

////////////////////////////////////////////////SETUP////////////////////////////////////////////////
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

