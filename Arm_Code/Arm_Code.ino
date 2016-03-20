// ARM CODE

#define MIRROR HIGH // when MIRROR = 1 bot is on ver 1/A (A is the side we started testing on), when MIRROR = 0 bot is on ver 2/B

//Packages being recieved from IR slave, IR_package1 hold the gripper's IRs, IR_package2 hold the frame's IRs
byte IR_package1 = 0; byte IR_package2 = 0;    // GET RID OF THIS WHEN THIS CODE GOES INTO MASTER CODE!!!!!!

#define MODE_2  A14 // all drivers share Mode2, 1, 0, these hold the values for the step size
#define MODE_1  A13
#define MODE_0  A12
bool mode[3]; // used to set the modes

// stepper motors
struct Steppers {
  int Dir; // pin on driver
  int Step; // pin on driver
  int Sleep; // pin on driver, active HIGH
  int Place; // current place of stepper, 0 will be left or down most position of lead screw
  int Max; // max is the number of steps on each lead screw, used for fault code inside toggleStep
};

Steppers Stepper[5] = { // sets pins for Dir, Step, and Sleep pins and intializes place and max
  {23, 25, 27, 0, 0}, // LIL_X, 5 on PCB
  {29, 31, 33, 0, 0}, // BIG_X,, 4 on PCB
  {35, 37, 39, 0, 0},  // LIL_Y, 3 on PCB
  {41, 43, 45, 0, 0},  // BIG_Y, 2 on PCB
  {47, 49, A15, 0, 0}  //Z, 1 on PCB
};

// assign values to names of steppers to be used to eaisly call stepper from struct
#define LIL_X 0
#define BIG_X 1
#define LIL_Y 2
#define BIG_Y 3
#define Z 4

// directions of steppers
#define UP HIGH // for LIL_Y and BIG_Y
#define DOWN LOW // for LIL_Y and BIG_Y
#define RIGHT HIGH //  for LIL_X and BIG_X                // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!
#define LEFT LOW // for LIL_X and BIG_X                   // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!
#define OUT HIGH // for Z                                 // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!
#define IN LOW  // for Z                                  // CHECK IF HIGH AND LOW ARE ASSIGNED CORRECTLY!!!

// limit switches        /// GET THE CORRECT PINS!! FOR THE LIMIT SWITCHES ////////            
#define LIL_X_LEFT   0
#define LIL_X_RIGHT  1
#define LIL_X_HOME   2

#define BIG_X_LEFT   3
#define BIG_X_RIGHT  4
#define BIG_X_HOME   5

#define LIL_Y_DOWN   6
#define LIL_Y_UP     7
#define LIL_Y_TRAIN  8

#define BIG_Y_DOWN    9
#define BIG_Y_UP    10
#define BIG_Y_HOME   11

#define Z_IN         12
#define Z_OUT        13

int Limits[14]; // array for limit switches that hold 0 if limit switch is not pressed and 1 if it is

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

////////////////////////// ARM MOVEMENT FUNCTIONS TO BE CALLED BY MASTER ///////////////////////////////////////
void Arm_Start_Finish_Pos(){ // how arm will be set up for the start
  limitStep(Z, Z_IN, IN, 1);
  limitStep(BIG_Y, BIG_Y_DOWN, DOWN, 1); // OR SHOULD BIG_Y_DOWN BE HOME?? WHERE IS HOME FOR BIG_Y AND LIL_Y?
  limitStep(LIL_Y, LIL_Y_DOWN, DOWN, 1);
  limitStep(BIG_X, BIG_X_HOME, LEFT, 1); 
  limitStep(LIL_X, LIL_X_HOME, LEFT, 1);
}

void Arm_Approach_Barge(bool Mirror){ // arm position for approaching the barges
  limitStep(BIG_Y, BIG_Y_UP, UP, 1);
  limitStep(LIL_Y, LIL_Y_UP, UP, 1);
  limitStep(Z, Z_OUT, OUT, 1);
    
  if(Mirror) { // side 1/A
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
    limitStep(BIG_X, BIG_X_HOME, RIGHT, 1); 
  }
  else { // side 2/B
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
    limitStep(BIG_X, BIG_X_HOME, LEFT, 1);
  }
}

void Arm_Find_Blocks(){ // move the arm down until the IRs see blokcs
  //drop BIG_Y until limit or IR, if needed drop LIL_Y until limit or IR

//  senseIRs();  // gets packages from slave                                               ADD THIS WHEN THIS CODE GOES INTO MASTER CODE!!!!!!
  if(Limits[BIG_Y_DOWN] == 0 && IR_package1 == 11111111 && IR_package2 == 11111111 ){   // if IRs on the gripper and frame don't see anything
    toggleStep(BIG_Y, DOWN);    // move BIG_Y down
  } 
  
 else if(Limits[LIL_Y_DOWN] == 0 && IR_package1 == 11111111 && IR_package2 == 11111111 ){ // after moving BIG_Y if IRs still don't see anything move LIL_Y
      toggleStep(LIL_Y, DOWN); // move LIL_Y down
    }
}

void IR_Hunt(bool Mirror){ //  moves LIL_X until the IRs on the frame and grippers see the correct IR pattern
//  senseIRs();  // gets packages from slave                                               ADD THIS WHEN THIS CODE GOES INTO MASTER CODE!!!!!!
  if(Mirror) { // side 1/A
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

void Arm_Leave_Barge(){ // arm position when leaving all barges but second trip to barge A
  // retract arm a bit so it does not hit things when moving to next location
  Step(BIG_Y, BIG_Y_UP, UP, 1 , 800); // BIG_Y up a bit             //SEE HOW MANY STEPS ARE BEST!!!!
  limitStep(Z, Z_IN, IN, 1);
}  

void Arm_Boat_Pos(){ // arm position for dropping blocks in boat
  limitStep(Z, Z_OUT, OUT, 1); // move Z to put arm over boat as far as possible
  limitStep(BIG_Y, BIG_Y_DOWN, DOWN, 1); // move BIG_Y down to be close to boat
}  

void Arm_Leave_Boat(){ // arm position for arm before leaving boat
  limitStep(Z, Z_IN, IN, 1); // move Z in
  limitStep(BIG_Y, BIG_Y_UP, UP, 1); // move BIG_Y UP
}

void Arm_Truck_Pos(){ // position arm for going into truck
 limitStep(BIG_Y, BIG_Y_DOWN, DOWN, 1); // BIG_Y down
 limitStep(LIL_Y, LIL_Y_DOWN, DOWN, 1); // LIL_Y down

 if(MIRROR){ // side is 1/A
    limitStep(BIG_X, BIG_X_LEFT, LEFT, 1);
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
 }
 else{ // side 2/B
    limitStep(BIG_X, BIG_X_RIGHT, RIGHT, 1);
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
 }
}  

void Arm_First_Train(){ // arm position for dropping blocks in first train
 limitStep(LIL_Y, LIL_Y_UP, UP, 1);      /// FIX THIS IT DOESN'T MAKE SENSE
 limitStep(BIG_Y, BIG_Y_HOME, DOWN, 1);
 limitStep(LIL_Y, LIL_Y_TRAIN, DOWN, 1);
 limitStep(Z, Z_OUT, OUT, 1);      
  }

void Arm_Train_Down(bool Mirror){ // arm position for dropping blocks in trains moving toward boat 
  if(Mirror){ // side is 1/A
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
  }
  else{ // side 2/B
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
  }  
}

void Arm_Train_Back(bool Mirror){ // arm position for dropping blocks in trains moving away from boat 
  if(Mirror){ // side is 1/A
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
  }
  else{ // side 2/B
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
  }  
}

void Arm_Leave_Train(bool Mirror){ // arm position to set before leaving trains
  limitStep(Z, Z_IN, IN, 1);
  if(Mirror){ // side 1/A
    limitStep(LIL_X, LIL_X_HOME, LEFT, 1);
  }
  else{ // side 2/B
    limitStep(LIL_X, LIL_X_HOME, RIGHT, 1);
  }
}

void Arm_Leave_BargeA(bool Mirror){ // arm position to set before leaving Barge A
  Step(BIG_Y, BIG_Y_UP, UP, 1 , 300); // BIG_Y up a bit             //SEE WHAT STEP SIZE IS BEST

  if(MIRROR){ // side 1/A
    // move x to right
    limitStep(BIG_X, BIG_X_RIGHT, RIGHT, 1);
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
  }

  else{ // side 2/B
    // move x to left
    limitStep(BIG_X, BIG_X_LEFT, LEFT, 1);
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
  }
  limitStep(Z, Z_IN, IN, 1);
  
}

void Arm_BargeA_Reach_Spot(bool Mirror){ // arm position to set for reaching blocks on barge A behind rail cars
  if(MIRROR){ // side 1/A
    limitStep(BIG_X, BIG_X_LEFT, LEFT, 1);
    limitStep(LIL_X, LIL_X_LEFT, LEFT, 1);
  }
  else{ // side 2/B
    limitStep(BIG_X, BIG_X_RIGHT, RIGHT, 1);
    limitStep(LIL_X, LIL_X_RIGHT, RIGHT, 1);
 
  }
}

////////////////// FUNCTIONS NESTED IN ARM MOVEMENT FUCNTIONS ///////////////////////////////////////////////

void buttonStep(){ // movement of arm while master asks girpper if the buttons have been pressed
  // drop BIG_Y until limit, if needed drop LIL_Y until limit
  if(Limits[BIG_Y_DOWN] == 0 ){
    toggleStep(BIG_Y, DOWN);
  } 
 else if(Limits[LIL_Y_DOWN] == 0 ){
      toggleStep(LIL_Y, DOWN);
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
  int countSteps = 0; // counts number of steps travelled by stepper
  setModes(stepSize);
  digitalWrite(Stepper[Stepper_SL].Dir, spin); // sets direction of stepper

  while( Limits[Limit_SL] == 0){    // while limit switch is not pressed
    toggleStep(Stepper_SL, spin);
    countSteps++;
    if (countSteps > stepNum){ // if it has travelled the number of steps it was told to
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
  // assign mode pins HIGH or LOW
  digitalWrite(MODE_0, mode[0]);
  digitalWrite(MODE_1, mode[1]);
  digitalWrite(MODE_2, mode[2]);
}

void toggleStep(int Stepper_SL, bool spin){ // toggles the Step pin on the driver, moves stepper one step
  // toggles on the rising edge
  digitalWrite(Stepper[Stepper_SL].Step, LOW);
  delayMicroseconds(500); // don't change this delay!!
  digitalWrite(Stepper[Stepper_SL].Step, HIGH);

  delay(1);  // CHANGE LATER! we want this to be the smallest number possible that allows the motors to move
}

