// ARM CODE

#define MIRROR // when MIRROR = 1 bot is on ver 1/A (A is the side we started testing on), when MIRROR = 0 bot is on ver 2/B

//Packages being recieved from IR slave, IR_package1 hold the gripper's IRs, IR_package2 hold the frame's IRs
byte IR_package1 = 0; byte IR_package2 = 0;    // GET RID OF THIS WHEN THIS CODE GOES INTO MASTER CODE!!!!!!

#define MODE_2  12 // all drivers share Mode2, 1, 0, these hold the values for the step size
#define MODE_1  11
#define MODE_0  10
#define UP HIGH // for LIL_Y and BIG_Y
#define DOWN LOW // for LIL_Y and BIG_Y
#define RIGHT HIGH //  for LIL_X, BIG_X, and Z            // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!
#define LEFT LOW // for LIL_X, BIG_X, and Z               // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!
bool mode[3]; // all steppers use the same mode pins 

// stepper motors
#define LIL_X 0
#define BIG_X 1
#define LIL_Y 2
#define BIG_Y 3
#define Z 4

struct Steppers {
  int Dir;
  int Step;
  int Sleep; // active HIGH
  int Place; // current place of stepper, 0 will be left or down most position of lead screw
  int Max; // max is the number of steps on each lead screw, used for fault code inside toggleStep
};

Steppers Stepper[5] = { // sets pins for Dir and Step pins on driver
  {2, 3, 13, 0, 0},
  {4, 5, 14, 0, 0},
  {6, 7, 15, 0, 0},
  {8, 9, 16, 0, 0},
  {10, 11, 17, 0, 0}
};

// limit switches              
#define LIL_X_LEFT   0
#define LIL_X_RIGHT  1
#define LIL_X_HOME   2

#define BIG_X_LEFT   3
#define BIG_X_RIGHT  4
#define BIG_X_HOME   5

#define LIL_Y_MIN    6
#define LIL_Y_MAX    7
#define LIL_Y_TRAIN  8

#define BIG_Y_MIN    9
#define BIG_Y_MAX    10
#define BIG_Y_HOME   11

#define Z_IN         12
#define Z_OUT        13

int Limits[14];

// button
#define L_BUTTON 14
#define R_BUTTON 15

bool flag = LOW; // for fault code inside toggleStep
///////////////////////// END OF DECLARATIONS ///////////////////////////////
void setup() {
  // configure pin modes
  pinMode(MODE_0,OUTPUT);
  pinMode(MODE_1,OUTPUT);
  pinMode(MODE_2,OUTPUT);
  
  for (int i = 0; i < 1; i++) {
    pinMode(Stepper[i].Dir, OUTPUT);
    pinMode(Stepper[i].Step, OUTPUT);
    pinMode(Stepper[i].Sleep, OUTPUT);
  }

  // initialize vars
  digitalWrite(MODE_0, LOW);
  digitalWrite(MODE_1, LOW);
  digitalWrite(MODE_2, LOW);

 for (int i = 0; i < 1; i++) {
    digitalWrite(Stepper[i].Dir, HIGH);
    pinMode(Stepper[i].Step, LOW);
    pinMode(Stepper[i].Sleep, HIGH);
  }
}
//////////////////////// END OF SETUP ////////////////////
void loop() {


}

//////////////////////// END OF LOOP /////////////////////

void IR_Hunt(bool Mirror){ //  moves LIL_X until the IRs on the fram and grippers see the correct IR pattern
//  senseIRs();  // gets packages from slave                                               ADD THIS WHEN THIS CODE GOES INTO MASTER CODE!!!!!!
  if(Mirror == 1) { // side 1/A
    while(IR_package1 != 192 && IR_package2 != 255){ // IRs don't see correct pattern
      toggleStep(LIL_X, LEFT); // move left
    }
  }
  else { // side 2/B
   while(IR_package1 != 192 && IR_package2 != 255){ // IRs don't see correct pattern
      toggleStep(LIL_X, RIGHT); // move right
    }
  }
}

void buttonStep(){ // drop BIG_Y until limit or button, if needed drop LIL_Y until limit or button
  bool pressed;
  while(Limits[BIG_Y_MIN] == 0 || ( (R_BUTTON == 0) && (L_BUTTON == 0) ) ){   
    toggleStep(BIG_Y, DOWN);
    if(R_BUTTON == 1 && L_BUTTON == 0){
      pressed = HIGH;
    } 
  }
  if(!pressed){
    while(Limits[LIL_Y_MIN] == 0 || ( (R_BUTTON == 0) && (L_BUTTON == 0) ) ){
      toggleStep(LIL_Y, DOWN);
    }
  }
  
}

void limitStep(int Stepper_SL, int Limit_SL, bool spin, int stepSize){   // moves the stepper until the limit switch is tripped
  setModes(stepSize);
  digitalWrite(Stepper[Stepper_SL].Dir, spin); // sets direction of stepper
  while( Limits[Limit_SL] == 0){ // while limit switch is not pressed
    toggleStep(Stepper_SL, spin);
  }
}

void Step(int Stepper_SL, int Limit_SL, bool spin, int stepSize, int stepNum){  // moves the stepper for certain amounts of steps unless limit switch is tripped
  int counter = 0;
  setModes(stepSize);
  digitalWrite(Stepper[Stepper_SL].Dir, spin); // sets direction of stepper

  while( Limits[Limit_SL] == 0){    // while limit switch is not pressed
    toggleStep(Stepper_SL, spin);
    counter++;
    if (counter > stepNum){
      break;
    }
  }   
}

void setModes(int stepSize){ // sets the mode pins on the driver given the stepSize that is wanted
  if (stepSize == 1)   { mode[2] = LOW; mode[1] = LOW; mode[2] = LOW;}; // full step
  if (stepSize == 2)   { mode[2] = LOW; mode[1] = LOW; mode[0] = HIGH;}; // 1/2 step
  if (stepSize == 4)   { mode[2] = LOW; mode[1] = HIGH; mode[0] = LOW;}; // 1/4 step
  if (stepSize == 8)   { mode[2] = LOW; mode[1] = HIGH; mode[0] = HIGH;}; // 8 microsteps per step
  if (stepSize == 16)  { mode[2] = HIGH; mode[1] = LOW; mode[0] = LOW;}; // 16 microsteps per step
  if (stepSize == 32)  { mode[2] = HIGH; mode[1] = LOW; mode[0] = HIGH;}; // 32 microsteps per step
  digitalWrite(MODE_0, mode[0]);
  digitalWrite(MODE_1, mode[1]);
  digitalWrite(MODE_2, mode[2]);
}

void toggleStep(int Stepper_SL, bool spin){ // toggles the Step pin on the driver, moves stepper one step
  digitalWrite(Stepper[Stepper_SL].Step, LOW);
  delayMicroseconds(500); 
  digitalWrite(Stepper[Stepper_SL].Step, HIGH);

  delay(1);  // CHANGE LATER! we want this to be the smallest number possible
}

