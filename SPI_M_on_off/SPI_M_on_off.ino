#include <SPI.h>
#include <LiquidCrystal.h> 

LiquidCrystal lcd(41, 43, 45, 47, 49, 39);

float Ping1 = 0; float Ping2 = 0;
int P1_lead = 0; int P1_total = 0;
int P2_lead = 0; int P2_total = 0;

void setup() {

  lcd.begin(16, 2); // set up the LCD's number of columns and rows
  
  lcd.print("SPI TESTING");   // Print a message to the LCD.
  
  delay(2000);
  lcd.clear();
  
  
  Serial.begin (115200);
  Serial.println ();
  
  digitalWrite(SS, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
}
byte transferAndWait (const byte what)
{
  byte a = SPI.transfer (what);
  delayMicroseconds (20);
  return a;
} // end of transferAndWait

void loop (void){
  
  for (int i=10;i>0;i--){
      Serial.println(i);
      delay(500);
  }
  
  for(int i=0;i<100;i++){  
    // enable Slave Select
    digitalWrite(SS, LOW);   
    transferAndWait ('f');  // add command 
    transferAndWait (2);  // add command
     P1_lead = transferAndWait (3);
     P1_lead <<= 8; 
     P1_total = transferAndWait (4);
     P1_total = P1_lead | P1_total;
     P2_lead = transferAndWait (0);
     P2_lead <<= 8; 
     P2_total = transferAndWait (0);
     P2_total = P2_lead | P2_total; 
     digitalWrite(SS, HIGH);

   Ping1 = (float)P1_total;
   Ping2 = (float)P2_total;
   
    Serial.println(i);
    Serial.print ("Ping 1: ");
    Serial.print (P1_total);
    Serial.println("mm");
    Serial.print ("Ping 2: ");
    Serial.print(P2_total);
    Serial.println("mm");
    Serial.println();
    
    delay (200);  // 1 second delay 
 }
    digitalWrite(SS, LOW);   
    transferAndWait ('q');  // add command 
    transferAndWait (2);  // add command
    digitalWrite(SS, HIGH);
    
 delay(5000);
}  // end of loop
