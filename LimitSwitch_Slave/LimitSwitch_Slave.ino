// hunt for blocks
struct LimitSwitch{
  uint8_t pin;
  uint32_t lastDebounce;
  bool lastRead = 0;
  byte state = 0;
};

struct LimitSwitch lilx[3];
struct LimitSwitch BIGX[3];
struct LimitSwitch lily[3];
struct LimitSwitch BIGY[3];
struct LimitSwitch Z[2];

  
// IR pins 1, 4, 5, 7, 15, 19 are in the grippers and have 10cm and less range
// IRs 3, 2, 5, 6, 17, 18 are on the frame and have 15cm and less range
// 1 means it doesn't see anything, 0 means it does

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
volatile int on = 0; //turns sensor on and off
byte lilx_package = 'n';
byte lily_package = 'n';
byte BIGX_package = 'n';
byte BIGY_package = 'n';
byte Z_package = 'n';


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
   lilx[0].pin = 2;  BIGX[0].pin = 5; lily[0].pin = 8; BIGY[0].pin = 11; Z[0].pin = 14;
   lilx[1].pin = 3;  BIGX[1].pin = 6; lily[1].pin = 9; BIGY[1].pin = 12; Z[1].pin = 15;
   lilx[2].pin = 4;  BIGX[2].pin = 7; lily[2].pin = 10; BIGY[2].pin = 13; 
  //set IR pins to inputs
  for(int i = 0; i < 3; i++){
    pinMode(lilx[i].pin, INPUT);
    pinMode(BIGX[i].pin, INPUT);
    pinMode(lily[i].pin, INPUT);
    pinMode(BIGY[i].pin, INPUT);
  }
  pinMode(Z[0].pin, INPUT);
  pinMode(Z[1].pin, INPUT);

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
    break;
    
  case 'x':
    command = c;
    SPDR = lilx_package;  
    break;
    
  case 'y':
    command = c;
    SPDR = lily_package;  
    break;

  case 'X':
    command = c;
    SPDR = BIGX_package;  
    break;


  case 'Y':
    command = c;
    SPDR = BIGY_package;  
    break;

  case 'Z':
    command = c;
    SPDR = Z_package;  
    break;
    
  } // end of switch
}  // end of Interupt //////////////////////////////////////////////////////////////////////

void loop(){
       
  for(int i = 0; i < 3; i++){
    lilx[i].state = (byte)limitReading(i, lilx);
    BIGX[i].state = (byte)limitReading(i, BIGX); 
    lily[i].state = (byte)limitReading(i, lily); 
    BIGY[i].state = (byte)limitReading(i, BIGY);
  }
  for(int i = 0; i < 2; i++){
    Z[i].state = (byte)limitReading(i, Z);
  }
  packageData();
  // if SPI not active, clear current command
  if (digitalRead (SS) == HIGH){
      command = 0;
  }
}
  
// DEBOUNCE READING FUNCTIONS ///////////////////////////////////////////////////////////////
int limitReading(int i, LimitSwitch axis[]){
   uint8_t reading;
    reading = digitalRead(axis[i].pin);
    
    if(axis[i].lastRead != reading)
      axis[i].lastDebounce = millis();
      
    if((millis() - axis[i].lastDebounce) > 50)
    {
      axis[i].state = reading;
    }
    axis[i].lastRead = reading;
    return reading;
}


void packageData(){
  if (lilx[0].state) lilx_package = 'L';
  else if (lilx[1].state) lilx_package = 'H';
  else if (lilx[2].state) lilx_package = 'R';
  else lilx_package = 'n';

  if (BIGX[0].state) BIGX_package = 'L';
  else if (BIGX[1].state) BIGX_package = 'H';
  else if (BIGX[2].state) BIGX_package = 'R';
  else BIGX_package = 'n';
  
  if (lily[0].state) lily_package = 'U';
  else if (lily[1].state) lily_package = 'H';
  else if (lily[2].state) lily_package = 'D';
  else lily_package = 'n';

  if (BIGY[0].state) BIGY_package = 'U';
  else if (BIGY[1].state) BIGY_package = 'H';
  else if (BIGY[2].state) BIGY_package = 'D';
  else BIGY_package = 'n';

  if (Z[0].state) lily_package = 'I';
  else if (Z[1].state) lily_package = 'O';
  else Z_package = 'n';
}
  
  






