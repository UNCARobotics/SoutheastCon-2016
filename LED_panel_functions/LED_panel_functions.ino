#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN,
  NEO_MATRIX_LEFT     + NEO_MATRIX_TOP +
  NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);
  
int c = 0;  
uint32_t lastlight = 0;
void setup() {
  matrix.begin();
  matrix.show();
  matrix.setBrightness(20);
}

void loop() {
  
while((millis()>0000) && (millis()<6000)){
  led_Nav(1);
}  

while((millis()>6000) && (millis()<15000)){
   lastlight = led_Rotate(0, lastlight);
}
 
while(millis()>15000){
  led_Nav(0);
}  
  
}

void led_Nav(bool n){
  int x[12] = {0,1,0,7,6,7,0,1,0,7,6,7};
  int y[12] = {0,0,1,0,0,1,7,7,6,7,7,6};
  for(int i = 0; i<12; i++){
    if (n == 1) matrix.drawPixel(x[i], y[i], matrix.Color(0, 255 , 0));
    else matrix.drawPixel(x[i], y[i], matrix.Color(255, 0 , 0));
  }
  matrix.show();
}

uint32_t led_Rotate(bool dir, uint32_t lastlight){
  int x[12] = {0,1,0,7,6,7,0,1,0,7,6,7}; int y[12] = {0,0,1,0,0,1,7,7,6,7,7,6};
  int cwX[4] = {2,7,5,0}; int cwY[4] = {0,2,7,5};
  int ccwX[4] = {5,7,2,0}; int ccwY[4] = {0,5,7,2}; 
  
  for(int i = 0; i<12; i++){
    matrix.drawPixel(x[i], y[i], matrix.Color(255, 150 , 0));
  }
  if (dir == 1){ //CW
    if((millis() - lastlight) > 200){
      for(int k=0;k<=c;k++){
        for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(cwX[i]+k, cwY[i], matrix.Color(0, 200 , 150));
          if(i == 1) matrix.drawPixel(cwX[i], cwY[i]+k, matrix.Color(0, 200 , 150));
          if(i == 2) matrix.drawPixel(cwX[i]-k, cwY[i], matrix.Color(0, 200 , 150));
          if(i == 3) matrix.drawPixel(cwX[i], cwY[i]-k, matrix.Color(0, 200 , 150));       
        }
      }
      c++;
     if(c == 5){
        for(int k=0;k<(c-1);k++){
          for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(cwX[i]+k, cwY[i], matrix.Color(0, 0 , 0));
          if(i == 1) matrix.drawPixel(cwX[i], cwY[i]+k, matrix.Color(0, 0 , 0));
          if(i == 2) matrix.drawPixel(cwX[i]-k, cwY[i], matrix.Color(0, 0 , 0));
          if(i == 3) matrix.drawPixel(cwX[i], cwY[i]-k, matrix.Color(0, 0 , 0));       
        }
      }
      c = 0;
     }
      lastlight = millis();
    }
    
  }
  else { //CCW
        if((millis() - lastlight) > 200){
      for(int k=0;k<=c;k++){
        for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(ccwX[i]-k, ccwY[i], matrix.Color(0, 200 , 150));
          if(i == 1) matrix.drawPixel(ccwX[i], ccwY[i]-k, matrix.Color(0, 200 , 150));
          if(i == 2) matrix.drawPixel(ccwX[i]+k, ccwY[i], matrix.Color(0, 200 , 150));
          if(i == 3) matrix.drawPixel(ccwX[i], ccwY[i]+k, matrix.Color(0, 200 , 150));       
        }
      }
      c++;
     if(c == 5){
        for(int k=0;k<(c-1);k++){
          for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(ccwX[i]-k, ccwY[i], matrix.Color(0, 0 , 0));
          if(i == 1) matrix.drawPixel(ccwX[i], ccwY[i]-k, matrix.Color(0, 0 , 0));
          if(i == 2) matrix.drawPixel(ccwX[i]+k, ccwY[i], matrix.Color(0, 0 , 0));
          if(i == 3) matrix.drawPixel(ccwX[i], ccwY[i]+k, matrix.Color(0, 0 , 0));       
        }
      }
      c = 0;
     }
      lastlight = millis();
    }

  }
  matrix.show();
  return lastlight;
}

