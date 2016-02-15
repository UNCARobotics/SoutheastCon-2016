// include the library code:
#include <LiquidCrystal.h>
#include <Wire.h> 
#include "Adafruit_TCS34725.h"

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

///////////////////////////////////////////END OF INITIALIZATION///////////////////////////////////////////////////

class Gripper {  /////////////////////////GRIPPER CLASS DEFINITION//////////////////////////////////////////////////
  
}
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

