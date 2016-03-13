// hunt for blocks
struct IR{
  uint8_t pin;
  uint32_t lastDebounce;
  uint8_t lastRead = 0;
  byte state = 0;
};

struct IR grippers[6];
struct IR frame[6];
  
// IR pins 1, 4, 5, 7, 15, 19 are in the grippers and have 10cm and less range
// IRs 3, 2, 5, 6, 17, 18 are on the frame and have 15cm and less range
// 1 means it doesn't see anything, 0 means it does

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile int on = 0; //turns sensor on and off
byte package[2] = {0};


void setup() {
  Serial.begin(115200);
  Serial.println("IEEE TESTING//////////////////////////////////");


  
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);
  
  // initialize IR pins
  grippers[0].pin = 4;    frame[0].pin = 3;
  grippers[1].pin = 9;    frame[1].pin = 2;
  grippers[2].pin = 7;    frame[2].pin = 5;
  grippers[3].pin = 15;   frame[3].pin = 6;
  grippers[4].pin = 16;   frame[4].pin = 17;
  grippers[5].pin = 19;   frame[5].pin = 18;

  //set IR pins to inputs
  for(int i = 0; i < 6; i++){
    pinMode(grippers[i].pin, INPUT);
    pinMode(frame[i].pin, INPUT);
  }
  

}

// SPI interrupt routine//////////////////////////////////////////////////////////////////////
ISR (SPI_STC_vect)
{
  byte c = SPDR;
 
  switch (command)
  {
  // no command? then this is the command
  case 0:
    command = c;
    SPDR = 0;
    on = 1;     //getting a trasfer? turn on sensors
    break;
    
  case 'i':
    command = c;
    SPDR = package[0];  
    break;
    
  case 2:
    command = c;
    SPDR = package[1]; 
    break;

 
  case 'q':             //reciving 'q' primes the slave to turn off sensors
    command = c;
    SPDR = 0;  // trailing byte of ping 2 on last transmition
    on = 0;               //turn off sensors
    break;
    
  } // end of switch
}  // end of Interupt //////////////////////////////////////////////////////////////////////

void loop(){
  on = 1;
   if (on == 1){       //only if told, turn sensors on
      for(int i = 0; i < 6; i++){
          grippers[i].state = (byte)gripperReading(i);
          frame[i].state = (byte)frameReading(i);
      }
      packageData();
      printDebug();
  }
    // if SPI not active, clear current command
  if (digitalRead (SS) == HIGH){
      command = 0;
  }
}
  
// DEBOUNCE READING FUNCTIONS ///////////////////////////////////////////////////////////////
int gripperReading(int i){
   uint8_t reading;
    reading = digitalRead(grippers[i].pin);
    
    if(grippers[i].lastRead != reading)
      grippers[i].lastDebounce = millis();
      
    if((millis() - grippers[i].lastDebounce) > 50)
    {
      grippers[i].state = reading;
    }
    grippers[i].lastRead = reading;
    return reading;
}

int frameReading(int i){
  uint8_t reading;
    reading = digitalRead(frame[i].pin);
    
    if(frame[i].lastRead != reading)
      frame[i].lastDebounce = millis();
      
    if((millis() - frame[i].lastDebounce) > 50)
    {
      frame[i].state = reading;
    }
    frame[i].lastRead = reading;
    return reading;
}

void packageData(){
  package[0] = B00000110;
  package[1] = B00000110;
  for(int i=0;i<5;i++){
    package[0] |= grippers[i].state;
    package[0] <<= 1;
    package[1] |= frame[i].state;
    package[1] <<= 1;
  }
  package[0] |= grippers[5].state;
  package[1] |= frame[5].state;
}

  void printDebug(){
    Serial.print("00");
    for(int i = 0; i < 6; i++){
          Serial.print(grippers[i].state);  
    }
    Serial.print("                      00");
    for(int i = 0; i < 6; i++){
          Serial.print(frame[i].state);  
    }
    Serial.println();
    Serial.print(package[0], BIN);
    Serial.print(" gripper IR's ... ");
    Serial.print(package[1], BIN);
    Serial.println(" frame IR's");
    Serial.println();
  
  }
  






