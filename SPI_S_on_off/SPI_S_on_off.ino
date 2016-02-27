// Written by Nick Gammon
// April 2011

#include <NewPing.h>
#define PIN_F1 2
#define PIN_F2 3

NewPing sonar[2] = {
  NewPing(PIN_F1, PIN_F1),
  NewPing(PIN_F2, PIN_F2),
};

// what to do with incoming data
volatile byte command = 0;
byte package[2][2] = {0};
float Ping[2] = {0}; // F1, F2
unsigned int Dist[2] = {0}; 
volatile int on = 0;

void setup (void)
{
  Serial.begin(115200);
  
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

 Serial.println("Begin"); 
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
 
  switch (command)
  {
  // no command? then this is the command
  case 0:
    command = c;
    SPDR = 0;
    on = 1;
    break;
    
  // add to incoming byte, return result
  case 'f':
    command = c;
    SPDR = package[0][0];  // add 15
    break;
    
  // subtract from incoming byte, return result
  case 2:
    command = c;
    SPDR = package[0][1];  // subtract 8
    break;

  // add to incoming byte, return result
  case 3:
    command = c;
    SPDR = package[1][0];  // add 15
    break;
    
  // subtract from incoming byte, return result
  case 4:
    command = c;
    SPDR = package[1][1];  // subtract 8
    break;
 
  case 'q':
    command = c;
    SPDR = package[1][1];  // subtract 8
    on = 0;
    break;
  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect

void loop (void)
{
if (on == 1){
  sensePings();
  displayPackage();
}
  // if SPI not active, clear current command
if (digitalRead (SS) == HIGH)
    command = 0;
}  // end of loop

void sensePings() {  //f & l represent first and last
  
    Ping[0] = (sonar[0].ping_median()/29/2)*10;  // send multiple pulses, return median distance
    Ping[1] = (sonar[1].ping_median()/29/2)*10;
    for(int i=0;i<2;i++){
      Dist[i] = (int)Ping[i]; 
      package[i][0] = Dist[i] >> 8;
      package[i][1] = Dist[i] & 0xFF;
    }  
   
}
void displayPackage(){ //Print to Serial, measurments and how they are packaged
Serial.print("--     ");
Serial.print(Dist[0]);
Serial.print("             ");
Serial.println(Dist[1]);
Serial.print("   ");
Serial.print(package[0][0],BIN);
Serial.print(" ");
Serial.print(package[0][1],BIN);
Serial.print("    "); 
Serial.print(package[1][0],BIN);
Serial.print(" ");
Serial.println(package[1][1],BIN);
}
