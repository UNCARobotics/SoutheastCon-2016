
// Define global variables for the pins of the motors
// top left wheel = a, top right wheel = b, bottom left wheel = c, bottom right wheel = d

// Hi this is Steve "The Melon" Moyer
// Hey this is Fiona
// test with Corey

// motor a
#define A_RED_PIN 5
#define A_WHITE_PIN 28
// motor b
#define B_RED_PIN 4
#define B_WHITE_PIN 26
// motor c
#define C_RED_PIN 3
#define C_WHITE_PIN 24
// motor d
#define D_RED_PIN 2
#define D_WHITE_PIN 22

// Define a struct for the motors to store base speed, correct speed, and direction, and initialize
struct Motors {
  int BaseSpeed;
  int CorrectSpeed;
  int Direction;
};

Motors MotorA = { 0, 0, HIGH };
Motors MotorB = { 0, 0, HIGH };
Motors MotorC = { 0, 0, HIGH };
Motors MotorD = { 0, 0, HIGH };

// define the pins for the ping sensors
#define PING_PIN_1 52
#define PING_PIN_2 50
#define PING_PIN_3 48
#define PING_PIN_4 56

// declare a struct for pings to hold variables for duration of the ping sensors and the distance result in centimeters
// Ping_1 is the ping closest to motor c on the left side, Ping_2 is closest to motor a on the left side
// Ping_3 is closest to motor a on the front, Ping_4 is closest to motor b on the front
struct Pings {
  long Duration;
  long Distance;
};

Pings Ping1;
Pings Ping2;
Pings Ping3;
Pings Ping4;
  
// declare variables for translational set points are set desired distances from wall in centimeters, rotational set points are desired angles
// left is the left set of Ping sensors which are on the side of the bot with motors A and C
// front is the front set of Ping sensors which are on the side of the bot with motors A and B 
struct SetPoints{
  int T;
  int R;
};

// Using 5*29*2 so that the convert to cm fuction does not have to be used
SetPoints Left_SetPoint = { 5 * 29 * 2 , 0 };
SetPoints Front_SetPoint = { 5 * 29 * 2  , 0 };

#define LOOP_TIME 0 // this is also a tuning variable to add later, for now we will just leave it 'off'
long long timer_correction;

// declaring structs to store current and previous translational error for left and front sensors
struct Error{
  float T_Current;
  float T_Previous;
  float R_Current;
  float R_Previous;
};

Error Left_Error = {0, 0, 0, 0};
Error Front_Error = {0, 0, 0, 0};

// initialize structs for Trans correction and prop correction for the left sensors;
struct Correct{
  float T;
  float R;
};

Correct Left_Correct = {0, 0};
Correct Front_Correct = {0, 0};

// declare structs for proportional and derivative constants for translational and rotational correction
struct PD_Constants{
  float P_Trans;
  float P_Rot;
  float D_Trans;
  float D_Rot;
};

PD_Constants Left_Const = { 20 , 0 , 0 , 0 };
PD_Constants Right_Const= { 0 , 0 , 0 , 0 };

// Configures the specified pin to behave either as an input or an output.
void setup() {
  //motor a
  pinMode(A_RED_PIN,OUTPUT);
  pinMode(A_WHITE_PIN,OUTPUT);

  //motor b
  pinMode(B_RED_PIN,OUTPUT);
  pinMode(B_WHITE_PIN,OUTPUT);
  
  //motor c
  pinMode(C_RED_PIN,OUTPUT);
  pinMode(C_WHITE_PIN,OUTPUT);

  //motor d
  pinMode(D_RED_PIN,OUTPUT);
  pinMode(D_WHITE_PIN,OUTPUT);
}

/// Paul TEst
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fuction for moving chassis forward
// Fuction for moving chassis forwards, it does not return anything

void Forwards(){
  digitalWrite(A_WHITE_PIN,MotorA.Direction);
  digitalWrite(B_WHITE_PIN,MotorB.Direction);
  digitalWrite(C_WHITE_PIN,MotorC.Direction);
  digitalWrite(D_WHITE_PIN,MotorD.Direction);  
  
  analogWrite(A_RED_PIN, MotorA.CorrectSpeed);
  analogWrite(B_RED_PIN, MotorB.CorrectSpeed);
  analogWrite(C_RED_PIN, MotorC.CorrectSpeed);
  analogWrite(D_RED_PIN, MotorD.CorrectSpeed);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fuction for taking readings of the ping sensors
void takeReading(int side){
  // SETTING UP the first ping on a side
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse
  
  // LEFT SIDE
  if(side==0){
    pinMode(PING_PIN_1, OUTPUT);
    digitalWrite(PING_PIN_1, LOW);
    delayMicroseconds(2);
    digitalWrite(PING_PIN_1, HIGH);
    delayMicroseconds(5);
    digitalWrite(PING_PIN_1, LOW);
    
    pinMode(PING_PIN_1, INPUT);
    Ping1.Distance = pulseIn(PING_PIN_1, HIGH);
  
    // SETTING UP the second ping on a side
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse
    pinMode(PING_PIN_2, OUTPUT);
    digitalWrite(PING_PIN_2, LOW);
    delayMicroseconds(2);
    digitalWrite(PING_PIN_2, HIGH);
    delayMicroseconds(5);
    digitalWrite(PING_PIN_2, LOW);
  
    pinMode(PING_PIN_2, INPUT);
    Ping2.Distance = pulseIn(PING_PIN_2, HIGH);
  }
  // FRONT SIDE
  else if(side==1){}
  // RIGHT SIDE
  else if(side==2){}
  // BACK SIDE
  else{}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // a tuning variable to add later, right now it is 'off'
  timer_correction = millis(); 

  MotorA.Direction = HIGH;
  MotorB.Direction = HIGH;
  MotorC.Direction = HIGH;
  MotorD.Direction = HIGH;

  // save current distance in previous 
  Left_Error.T_Previous = Left_Error.T_Current;
  Left_Error.R_Previous = Left_Error.R_Current;
  Front_Error.T_Previous = Front_Error.T_Current;
  Front_Error.R_Previous = Front_Error.R_Current;

  // call fuction takeReading for finding distances using the ping sensors 1 and 2
  takeReading(0);

  // calculate left translational error
  Left_Error.T_Current = ((float)(Ping1.Distance + Ping2.Distance)/2) - (float)Left_SetPoint.T;

  /*
   * Serial.println(Left_Error.T_Current);
   * Serial.println();
   */
  
  // calculate left translation correction
  Left_Correct.T = (Left_Const.P_Trans * Left_Error.T_Current) + ((Left_Const.D_Trans) * (Left_Error.T_Current - Left_Error.T_Previous));
  // calculate left rotational error
  Left_Error.R_Current = (Ping1.Distance - Ping2.Distance) - Left_SetPoint.R;
  // calcualte left rotational correction
  Left_Correct.R = (Left_Const.P_Rot * Left_Error.R_Current) +((Left_Const.D_Rot) * (Left_Error.R_Current - Left_Error.R_Previous));

  /*
   * Leave all of this stuff commented out if tuning controllers, no need to sample more than ping's 1 & 2
   */
//  
//  // call fuction takeReading for finding distances using the ping sensors 3 and 4
//  takeReading(PING_PIN_3, PING_PIN_4, Ping3.Distance, Ping4.Distance);
//
//  // save distances found from takeReading fuction 
//  Ping3.Distance = Sonar1_Dist;
//  Ping4.Distance = Sonar2_Dist;

  
  // set motor speeds
  // multiplying by (int) casts Tran_Correct_Left into an interger from float
  MotorA.CorrectSpeed = (int) ((float) MotorA.BaseSpeed - (Left_Correct.T) - (Left_Correct.R) + (Front_Correct.T) - (Front_Correct.R) );
  MotorB.CorrectSpeed = (int) ((float) MotorB.BaseSpeed  + (Left_Correct.T) + (Left_Correct.R) - (Front_Correct.T) - (Front_Correct.R) );
  MotorC.CorrectSpeed = (int) ((float) MotorC.BaseSpeed + (Left_Correct.T) - (Left_Correct.R) - (Front_Correct.T) + (Front_Correct.R) );
  MotorD.CorrectSpeed = (int) ((float) MotorD.BaseSpeed - (Left_Correct.T) + (Left_Correct.R) + (Front_Correct.T) + (Front_Correct.R) );

  // if the motor speed is a negative number, make the speed positive and change the direction of the motor
  if(MotorA.CorrectSpeed < 0){
    MotorA.CorrectSpeed = -MotorA.CorrectSpeed;
    MotorA.Direction = LOW;
  }
  
  if(MotorB.CorrectSpeed < 0){
    MotorB.CorrectSpeed = -MotorB.CorrectSpeed;
    MotorB.Direction = LOW;
  }
  
  if(MotorC.CorrectSpeed < 0){
    MotorC.CorrectSpeed = -MotorC.CorrectSpeed;
    MotorC.Direction = LOW;
  }
  
  if(MotorD.CorrectSpeed < 0){
    MotorD.CorrectSpeed = -MotorD.CorrectSpeed;
    MotorD.Direction = LOW;
  }

  // constrain the speeds of the motors to a value in between 0 and 255, the faster the motor can go is 255
  // if the speed is < 0 it gets set as 0, if the speed is > 255 it gets set as 255
  MotorA.CorrectSpeed = constrain(MotorA.CorrectSpeed,0,255);
  MotorB.CorrectSpeed = constrain(MotorB.CorrectSpeed,0,255);
  MotorC.CorrectSpeed = constrain(MotorC.CorrectSpeed,0,255);
  MotorD.CorrectSpeed = constrain(MotorD.CorrectSpeed,0,255);

  // a tuning variable to add later, right now it is 'off'
  while(millis() < (timer_correction + LOOP_TIME)){}

  //call Forwards fuction
  Forwards();

}
