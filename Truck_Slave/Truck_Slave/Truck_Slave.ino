#include <NewPing.h>
#define PIN_T1 A5
#define PIN_T2 A4
#define PIN_T3 A3
#define PIN_T4 A2
#define HIST_ARRAY_SIZE 5 //array of ping readings to average

//initialize 2 pings
NewPing sonar[4] = {
  NewPing(PIN_T1, PIN_T1),
  NewPing(PIN_T2, PIN_T2),
  NewPing(PIN_T3, PIN_T3),
  NewPing(PIN_T4, PIN_T4),
};

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
byte package[4][2] = {0}; //storage for partitioned int (two bytes)
float Ping[4] = {0}; // T1, T2
unsigned int Dist[4] = {0}; //change ping readings to int's
volatile int on = 0; //turns sensor on and off
float Avg_Ping[4];
float Pings_History[4][HIST_ARRAY_SIZE];


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
    
  case 'f':
    command = c;
    SPDR = package[0][0];  // leading byte of ping 1
    break;
    
  case 2:
    command = c;
    SPDR = package[0][1];  // trailing byte of ping 1
    break;

  case 3:
    command = c;
    SPDR = package[1][0];  // leading byte of ping 2
    break;
    
  case 4:
    command = c;
    SPDR = package[1][1];  // trailing byte of ping 2
    break;
    
  case 5:  
    command = c;
    SPDR = package[2][0];  // leading byte of ping 3
    break;
    
  case 6:
    command = c;
    SPDR = package[2][1];  // trailing byte of ping 3
    break;

  case 7:
    command = c;
    SPDR = package[3][0];  // leading byte of ping 4
    break;
    
  case 8:
    command = c;
    SPDR = package[3][1];  // trailing byte of ping 4
    break;   
 
  case 'q':             //reciving 'q' primes the slave to turn off sensors
    command = c;
    SPDR = 0;
    on = 0;               //turn off sensors
    break;
    
  } // end of switch
}  // end of Interupt //////////////////////////////////////////////////////////////////////



void loop (void){
if (on == 1){       //only if told, turn sensors on
  sensePings(); 
}
  // if SPI not active, clear current command
if (digitalRead (SS) == HIGH)
    command = 0;
}  



//////////////////////////////sense, display/////////////////////////////////////////////////
void sensePings() { 
     
    for(int i=0; i<4;i++){
       Avg_Ping[i] = 0;  //reset Avgs
       Ping[i] = (sonar[i].ping()/29/2)*10;  //take measurments
    }
   

    //shift new measurement into a history of measurments
    for(int i=0;i<4;i++){
      for(int j=HIST_ARRAY_SIZE-1; j>0; j--){   //Shifts        
          Pings_History[i][j] = Pings_History[i][j-1];
      }
          Pings_History[i][0] = Ping[i];
   //take the average of the history including new value                                      
      for(int k=0;k<HIST_ARRAY_SIZE;k++){
          Avg_Ping[i] += Pings_History[i][k];
      }
        Avg_Ping[i] = Avg_Ping[i]/HIST_ARRAY_SIZE;

      //package the average value
      Dist[i] = (int)Avg_Ping[i]; //change float to int
      package[i][0] = Dist[i] >> 8; //chop into leading byte
      package[i][1] = Dist[i] & 0xFF; //chop into trailing byte
    }  
}





