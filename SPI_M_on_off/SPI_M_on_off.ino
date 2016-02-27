#include <SPI.h>
#include <LiquidCrystal.h> 
#define SS_1 9   //pin '10' from slave, into this SS_1 on master
LiquidCrystal lcd(41, 43, 45, 47, 49, 39);

float Ping1 = 0; float Ping2 = 0;
int P1_lead = 0; int P1_total = 0;
int P2_lead = 0; int P2_total = 0;

void setup() {

  lcd.begin(16, 2); // set up the LCD's number of columns and rows
  lcd.print("SPI TESTING");   // Print a message to the LCD.
  delay(2000);
  lcd.clear();
  
  
  Serial.begin(115200);
  Serial.println();

  // set slave select and intitialize closed (high)
  pinMode(SS_1, OUTPUT); 
  digitalWrite(SS_1, HIGH);  

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

// loop for testing. Counts down, turns on pings, takes 99 readings, and turns pings off. 
void loop(){
  //count down
  for (int i=10;i>0;i--){        
      Serial.println(i);
      delay(500);
  }
  
  // take 99 readings
  for(int i=0;i<100;i++){      
    getPings();
    SerialPrintReadings(i);
    delay (200);  
  }
  stopPings();  
  delay(5000); 
} 



//////////////////////Tranfer&Wait, getPings, stopPings, printReadings/////////////////////////////////////////////
 
byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
      byte a = SPI.transfer (what);
      delayMicroseconds (20);
      return a;
} 

void getPings(){
     digitalWrite(SS_1, LOW);  // open communication (direct slave into interupt routine)
       transferAndWait ('f');  // asks to turn pings on, and sets up the first byte transfer
       transferAndWait (2);   //pings are on and first request is recieved  
       P1_lead = transferAndWait (3); //first package comes in (leading P1 byte)
       P1_lead <<= 8;                 //shift bits to make room for tailing byte
       P1_total = transferAndWait (4); //second package comes in (trailing P1 byte)
       P1_total = P1_lead | P1_total;   //combine lead and tail into 16bit
       
       //do the same for P2    IMPORTANT to notice that all messages are recieved 2 transfers after called for
       P2_lead = transferAndWait (0);   //that is why there are two dummy commands sent with 0 here. 
       P2_lead <<= 8; 
       P2_total = transferAndWait (0);
       P2_total = P2_lead | P2_total; 
     digitalWrite(SS_1, HIGH); // close communication, but pings will continue to read

   Ping1 = (float)P1_total;
   Ping2 = (float)P2_total;
}
void stopPings(){
    digitalWrite(SS_1, LOW);   
    transferAndWait ('q');  // add command 
    transferAndWait (2);  // add command
    digitalWrite(SS_1, HIGH);
}
void SerialPrintReadings(int i){
    Serial.println(i);
    Serial.print ("Ping 1: ");
    Serial.print (P1_total);
    Serial.println("mm");
    Serial.print ("Ping 2: ");
    Serial.print(P2_total);
    Serial.println("mm");
    Serial.println();
}

