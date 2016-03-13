//right now it is based on distance and not time, the stepNum is divided by 3, this makes the constant slope part go faster


#define CCW HIGH
#define CW LOW

// all drivers use common/same Mode2, 1, 0 and SLEEP which is why they are not included in the Steppers struckt
int Sleep = 6;   // SLEEP must be HIGH to allow stepper to move, LOW turns it "off"
int Mode_2 = 12;
int Mode_1 = 11;
int Mode_0 = 10;

struct Steppers { //Stepper Motor Values
  int Dir;    // CW is 0 (which moves away from stepper), CCW is 1 (which towards  from stepper)
  int Step;

};

// set up pins
Steppers Stepper[6] = {
  {4, 5},
  //  {, },
  //  {, },
  //  {, },
  //  {, },
  //  {, }
};

// to make writing spin in Step fuction easier
//int CW = 0;
//int CCW = 1;


// variables for trapezoidal ramp
int Ramp_Delay = 0;
int Top_Speed = 10; // a shorter delay in between toggling makes it move faster
int Bottom_Speed = 10000; // a faster delay in between toggling makes it move slower
int Step_Amount = 0;
int place = 0; // where you are currently in the ramp

void setup() {

  // initialize global vars
  digitalWrite(Sleep, HIGH);
  digitalWrite(Mode_0, LOW);
  digitalWrite(Mode_1, LOW);
  digitalWrite(Mode_2, LOW);

  for (int i = 0; i < 6; i++) { //configure pin modes
    pinMode(Stepper[i].Dir, OUTPUT);
    pinMode(Stepper[i].Step, OUTPUT);
  }
  
}

void loop() {
  Ramp_Delay = Bottom_Speed;
  place = 0;
  Step(1, CCW, 2000);

  stopStepper();

  delay(5000);

}

void Step( float stepSize, bool spin , int stepNum){

  setModes(stepSize);
  digitalWrite(Sleep, HIGH);
  digitalWrite(Stepper[0].Dir, spin); // sets  direction of stepper
  Step_Amount = stepNum/3;
  
  // ramp up
  while(  (place >= 0)   &&   (place < (Step_Amount))  ){
    Ramp_Delay -= ((Bottom_Speed-Top_Speed)/Step_Amount);
    toggleStep(Ramp_Delay);
    place++;
  }

  // constant slope
  while(  (place >= Step_Amount)   &&   (place < (2*Step_Amount))  ){
    Ramp_Delay = Top_Speed;
    long timerr = millis();
    //while((millis()-timerr)<1000){
    //  toggleStep(Ramp_Delay);
    //}
    place++;
  }
  Ramp_Delay -= ((Bottom_Speed-Top_Speed)/Step_Amount);
  
  // ramp down
  while(  (place >= (2*Step_Amount))   &&   (place < (3*Step_Amount))  ){
     Ramp_Delay += ((Bottom_Speed-Top_Speed)/Step_Amount);
     toggleStep(Ramp_Delay);
     place++;
  }
  
 

}

void setModes( float stepSize){  // assigns the modes with the appropriate boolean value depending on the stepSize

  if (stepSize == 1)   { Mode_2 = 0; Mode_1 = 0; Mode_0 = 0;}; // full step
  if (stepSize == 1/2) { Mode_2 = 0; Mode_1 = 0; Mode_0 = 1;}; // 1/2 step
  if (stepSize == 1/4) { Mode_2 = 0; Mode_1 = 1; Mode_0 = 0;}; // 1/4 step
  if (stepSize == 8 )  { Mode_2 = 0; Mode_1 = 1; Mode_0 = 1;}; // 8 microsteps per step
  if (stepSize == 16)  { Mode_2 = 1; Mode_1 = 0; Mode_0 = 0;}; // 16 microsteps per step
  if (stepSize == 32)  { Mode_2 = 1; Mode_1 = 0; Mode_0 = 1;}; // 32 microsteps per step
}

void stopStepper() {
  digitalWrite(Sleep, LOW);
}
  


void toggleStep(int ramp_Delay){
  //for (int j = 0; j < (stepnum/3); j++){ 
    digitalWrite(Stepper[0].Step, HIGH);
    delayMicroseconds(500); // do not change this time
    digitalWrite(Stepper[0].Step, LOW); 
  //}
  delayMicroseconds(ramp_Delay);
}


