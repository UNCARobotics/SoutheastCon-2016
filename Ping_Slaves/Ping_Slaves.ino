#include <NewPing.h>
#define PIN_F1 5
#define PIN_F2 6
#define HIST_ARRAY_SIZE 1 //array of ping readings to average

//initialize 2 pings
NewPing sonar[2] = {
  NewPing(PIN_F1, PIN_F1),
  NewPing(PIN_F2, PIN_F2),
};

volatile byte command = 0;   // stores value recieved from master, tells slave what case to run
byte package[2][2] = {0}; //storage for partitioned int (two bytes)
float Ping[2] = {0}; // F1, F2
unsigned int Dist[2] = {0}; //change ping readings to int's
volatile int on = 0; //turns sensor on and off
float Avg_Ping[2];
float Pings_History[2][HIST_ARRAY_SIZE];


void setup (void)
{
  Serial.begin(115200);
  digitalWrite(17, LOW); // LED's off
  digitalWrite(3, LOW);
  // have to send on master in, *slave out*
  
  //pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  //SPCR |= _BV(SPE);

  // turn on interrupts
  //SPCR |= _BV(SPIE);

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
    
  case 'p':
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
 
  case 'q':             //reciving 'q' primes the slave to turn off sensors
    command = c;
    SPDR = 0;  // trailing byte of ping 2 on last transmition
    on = 0;               //turn off sensors
    break;
    
  } // end of switch
}  // end of Interupt //////////////////////////////////////////////////////////////////////



void loop (void){

//  if (on == 1){       //only if told, turn sensors on
//    
//    sensePings();
//  }
//    // if SPI not active, clear current command
//  if (digitalRead (SS) == HIGH){
//      digitalWrite(17, LOW);
//      digitalWrite(3, LOW);
//      command = 0;
//  }



//  sensePings();
//  Serial.println(Ping[0]);
//  Serial.println(Ping[1]);
//  Serial.println();
//  delay(500);
sensePings();
Serial.println(Ping[0]);
Serial.println(Ping[1]);
delay(500);


  //if (on == 1){       //only if told, turn sensors on
    
    sensePings();
  
    // if SPI not active, clear current command
  if (digitalRead (SS) == HIGH){
      //digitalWrite(17, LOW);
      //digitalWrite(3, LOW);
      command = 0;
  }

}  



//////////////////////////////sense, display/////////////////////////////////////////////////
void sensePings() { 
     //reset Avgs
    Avg_Ping[0] = 0; 
    Avg_Ping[1] = 0;
    

    
    Ping[0] = (sonar[0].ping()/29/2)*10;  
    //ping_1_check();
    Ping[1] = (sonar[1].ping()/29/2)*10;

    
    //shift new measurement into a history of measurments
    for(int i=0;i<2;i++){
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
void ping_1_check(){
  if (Ping[0] > 0){
   digitalWrite(17, HIGH); //LED ON
  }
}
void ping_2_check(){
  if (Ping[1] > 0){
   digitalWrite(3, HIGH); //LED ON
  }
}

