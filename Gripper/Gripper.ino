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

class Gripper {  /////////////////////////GRIPPER CLASS DEFINITION//////////////////////////////////////////////////
  public:
  char block_1;
  char block_2;
  char block_3;
  uint16_t colorTemp;

  //Block positions declarations (Picture below shows block numbers)
  // |1|    |3|
  // | |    |2|
  //      bot side of gripper

  void getColor() {
  //take color reading  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
    
    //red
    if ((colorTemp > 4500) && (colorTemp < 9000)) {   
    }
    
    //blue
    else if ((colorTemp > 9000) && ( colorTemp < 13000)){
    }
    
    //green
    else if ((colorTemp > 3000) && (colorTemp < 4500)){
    }
    
    //yellow
    else if ( (colorTemp > 2000) && ( colorTemp < 3000 )){
    }
    
    //ambient
    else {
     }
}
  }
};
//inititate grippers:////////////////////////////////////////////////////////////////////////////////////////

Gripper A;
Gripper B;
Gripper C;
Gripper D;

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

