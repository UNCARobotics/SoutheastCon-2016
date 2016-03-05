  
  #include <Wire.h> // enables you to communicate with I2C
  #include "Adafruit_TCS34725.h" // for color sensor
  
  // initialize the library with the numbers of the interface pins
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);
  
  #define COLOR_ARRAY_SIZE 15
  float Color[COLOR_ARRAY_SIZE];
  float ThisColor;
  float Avg_Color;
  
  
  void setup() {
  
    Serial.begin(115200);
    Serial.print("IEEE");
    Serial.println("");
    Wire.begin();
  
  }
  
  void loop()
  {
    uint16_t r, g, b, c, colorTemp, lux;
  
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
  
//    Serial.print(colorTemp);
//    Serial.println("");
  
    for(int i=COLOR_ARRAY_SIZE-1; i>0; i--){
      Color[i] = Color[i-1];
    }
    Color[0] = colorTemp;
    
    Avg_Color = 0;
  
    for(int k=0; k<COLOR_ARRAY_SIZE; k++){
      Avg_Color += abs(Color[k]);
    }
    Avg_Color = Avg_Color/COLOR_ARRAY_SIZE;
  
    Serial.println(Avg_Color);
    
  //
  //  // red
  //  if ((colorTemp > 4500) && (colorTemp < 9000)) {
  //    Serial.print("Red Block");
  //    Serial.println("");
  //  }
  //
  //  // blue
  //  else if ((colorTemp > 9000) && ( colorTemp < 13000)) {
  //    Serial.print("Blue Block");
  //    Serial.println("");
  //  }
  //
  //  // green
  //  else if ((colorTemp > 3000) && (colorTemp < 4500)) {
  //    Serial.print("Green Block \n");
  //  }
  //
  //  // yellow
  //  else if ( (colorTemp > 2000) && ( colorTemp < 3000 )) {
  //    Serial.print("Yellow Block  \n");
  //  }
  //
  //  else {
  //    Serial.print("Ambient \n");
  //  }
  }
  
  

