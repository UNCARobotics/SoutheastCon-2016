
/* Corey Pullium && Fiona Popp / March 2016 / IEEE SoutheastCon

 */
  //Arduino's Libraries 
  #include <LiquidCrystal.h> 
  #include <NewPing.h>
  #include <SPI.h>  
  #include <Pixy.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_NeoMatrix.h>
  #include <Adafruit_NeoPixel.h>
  #include <SoftwareSerial.h>

  //Ripley's Libraries
  #include "Macros.h"
  #include "Gripperset.h"
  #include "Sides.h"

  const int TxPin = A0; //Pin for LCD screen
  SoftwareSerial mySerial = SoftwareSerial(255, TxPin); //Create the LCD screen
  //PIXY
  Pixy pixy; // Create the Pixy
  struct trainCar{  //Place to store what the train car data from Pixy
  byte color; 
  int xpos; 
  int width;
  };

  //CHASSIS MOTORS
  struct Motors {   //Place to store commands for the Motors  
    int Inv_Backward;  //high = backwards
    int Inv_Forward;      //High = forwards
    int Speed;
    
    int Inv_Backward_Pin;  
    int Inv_Forward_Pin; 
    int SpeedPin;
  };
  
  
  //Motor numbers(Picture Below shows motor numbers);
  //        F
  //    ||-2 1-|| 
  // A  ||-3 0-||   L
  //        B

  
 // ARM
  struct Steppers { //Place to store commands for the Arm Motors
      int Dir; // pin on driver
      int Step; // pin on driver
      int Sleep; // pin on driver, active HIGH
      int Place; // current place of stepper, 0 will be left or down most position of lead screw
      int Max; // max is the number of steps on each lead screw, used for fault code inside toggleStep
   };
   bool stepMode[3]; // used to set the Arm step Modes
   int place = 0; // for counting steps
   int Ramp_Delay = 0; // for ramping up
   
  //Packages being recieved from IR slave, IR_package1 is the gripper's IRs, IR_package2 is the frame's IRs
   byte IR_package1 = 0; byte IR_package2 = 0;

  
  //Misc Global Variables
  int c = 0; //counter for led panel 
  bool board = 0; //Which playing board we are on
  
  //checking millis for loop time correction
  long timer = 0;
  long ct = 1;
  #define MAXIMUM 15
  bool flagger;

  
// End of Declarations//////////////////////////////////////////////////////////////////////////////////

// INITIALIZATIONS////////////////////////////////////////////////////////////////////////////////////
  
//Pixy blocks and globals   
struct trainCar box[2];
  float Pixy_Error = 0;
  int PixyFlag = 0;
  int gapCount = 0;
  
// Motors
    Motors Motor[4] ={
    { LOW, HIGH, LOW, M0_IN1, M0_IN2, M0_D2},
    { LOW, HIGH, LOW, M1_IN1, M1_IN2, M1_D2},
    { LOW, HIGH, LOW, M2_IN1, M2_IN2, M2_D2},
    { LOW, HIGH, LOW, M3_IN1, M3_IN2, M3_D2}
  }; 
    
  //Sides
               // PT,   DT,   PR,  DR
  Side Front     {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P2};
  Side Back      {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P3};
  Side Arm       {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P4};
  Side Leg       {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P5};
  // small P gains below. Small moves Ellie
  Side Truck_R   {0,   0,   0,     0,  0, 0, 0, 0, {},{},0,0, SS_T};
  Side Truck_L   {0,   0,   0,     0,  0, 0, 0, 0, {},{},0,0, SS_T};
  Side Truck_Arm {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P4};
  Side Truck_Leg {1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_P5};

  //Arm Motors
  Steppers ArmMotor[5] = { // sets pins for Dir, Step, and Sleep pins and intializes place and max
      {23, 25, 27, 0, 0}, // LIL_X, 5 on PCB
      {29, 31, 33, 0, 0}, // BIG_X,, 4 on PCB
      {35, 37, 39, 0, 0},  // LIL_Y, 3 on PCB
      {41, 43, 45, 0, 0},  // BIG_Y, 2 on PCB
      {47, 49, A15, 0, 0}  //Z, 1 on PCB
   };

   //Grippers
   Gripperset Grippers[2] = {
    {SS_AG1, SS_BG1, {0}, {0}, 0, 0},
    {SS_AG2, SS_BG2, {0}, {0}, 0, 0}
   }; 

  //LED Panel
  Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, LED_PIN,
  NEO_MATRIX_LEFT     + NEO_MATRIX_TOP +
  NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);
  
    
// SETUP///////////////////////////////////////////////////////////////////////////////////////////
  
  void setup() {
    Serial.begin(115200);
    
    //LCD Begin (Still untested)....
    pinMode(TxPin, OUTPUT);
    digitalWrite(TxPin, HIGH);
    mySerial.begin(9600);
    mySerial.write(12);                 // Clear             
    mySerial.write(17);                 // Turn backlight on
    delay(5); 
    mySerial.print("Lt. Ripley");  // First line
    mySerial.write(13);                 // Form feed
    mySerial.print("Testing");   // Second line
    mySerial.write(212);                // Quarter note
    mySerial.write(220);                // A tone
    delay(3000);                        // Wait 3 seconds// Required delay

    //LED Panel setup
    matrix.begin();
    matrix.fillScreen(0);
    matrix.show();
    matrix.setBrightness(20);
    
    //Pixy
    pixy.init(); //Initiate the PIXY

    //Motor Driver setup
    pinMode (EN_M0_M1, OUTPUT);
    pinMode (EN_M2_M3, OUTPUT);
    digitalWrite(EN_M0_M1, HIGH);
    digitalWrite(EN_M2_M3, HIGH);
    
    for(int i=0;i<4;i++){  //configure pin modes
      pinMode (Motor[i].Inv_Backward_Pin, OUTPUT);
      pinMode (Motor[i].Inv_Forward_Pin, OUTPUT); 
      pinMode (Motor[i].SpeedPin, OUTPUT); 
    }
    
      // Arm setup
    pinMode(MODE_0,OUTPUT);
    pinMode(MODE_1,OUTPUT);
    pinMode(MODE_2,OUTPUT);
  
    for (int i = 0; i < 1; i++) {
      pinMode(ArmMotor[i].Dir, OUTPUT);
      pinMode(ArmMotor[i].Step, OUTPUT);
      pinMode(ArmMotor[i].Sleep, OUTPUT);
    }
  
    // initialize setpsize
    digitalWrite(MODE_0, LOW);
    digitalWrite(MODE_1, LOW);
    digitalWrite(MODE_2, LOW);
  
   for (int i = 0; i < 1; i++) {
      digitalWrite(ArmMotor[i].Dir, HIGH);
      pinMode(ArmMotor[i].Step, LOW);
      pinMode(ArmMotor[i].Sleep, HIGH);
    }
    
    //All Ping SPI slave select pins
    pinMode(SS_P2, OUTPUT); 
    digitalWrite(SS_P2, HIGH);  
    pinMode(SS_P3, OUTPUT); 
    digitalWrite(SS_P3, HIGH);
    pinMode(SS_P4, OUTPUT); 
    digitalWrite(SS_P4, HIGH);
    pinMode(SS_P5, OUTPUT); 
    digitalWrite(SS_P5, HIGH);
  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  }
  
  
  
// LOOP/////////////////////////////////////////////////////////////////////////////////////////// 
  void loop(){ 
    // variables to make calling Rotation function easier to read
    int CCW = 0, CW = 1;
    int F = 0, A = 1, B = 2, L = 3;

    Front.sensePings();
    Back.sensePings();
    Arm.sensePings();
    Leg.sensePings();
    printReadings(0,0,0,0);
    
//  printCountdown();
//  
//  
//  Serial.println("Searching");
//  gripperCommand('s');
//  Serial.println("Touchdown");
//  gripperCommand('c');
//  printColors_Grippers(0);
//  gripperCommand('h');
//  Serial.print("H");
//  delay(8000);
//  gripperCommand('y');
//  delay(2000);
//  gripperCommand('g');
//  delay(2000);
//  gripperCommand('b');
//  delay(2000);
//  gripperCommand('r');
//  delay(2000);
//  gripperCommand('y');
//  delay(2000);
//  gripperCommand('a');

  }
  
  

  // NAVIGATION/////////////////////////////////////////////////////////////////////////////////////////////
  
  void Nav(float F, float B, float A, float L){
    float numParameters = 0;                      //number of non-Zero parameters 
    float Ns_Tfb = 0; float Avg_ErT_fb = 100;    //new speed; Avg Error; for front/back translation
    float Ns_Tal = 0; float Avg_ErT_al = 100;    //new speed; Avg Error; for arm/leg translation
    float Ns_R = 0;   float Avg_ErR = 100;       //new speed; Avg Error; for all translation
    
    
    fill_error_arrays(); //reset Error arrays
    
    while (1){ 
        //  timer = millis();
      
      led_Nav(1); //turn on LED PANEL
      
      Ns_R = 0;
      Avg_ErR = 0;
      numParameters = 0;    // reset each time you take a measurment 
      if(F){               // if passed a setpoint for Frontside
        numParameters++;
        Front.sensePings();
        Ns_Tfb = Front.PD_T(F);
        Ns_R = Front.PD_R();  
        Avg_ErT_fb = Front.getAvg_ErT();
        Avg_ErR = Front.getAvg_ErR();
      }
          
      else if(B != 0){        // if passed a setpoint for Backside
        numParameters++;
        Back.sensePings();
        Ns_Tfb = -Back.PD_T(B);
        Ns_R = -Back.PD_R();
        Avg_ErT_fb = Back.getAvg_ErT();
        Avg_ErR = Back.getAvg_ErR();
      }
      
      if(A != 0){             // if passed a setpoint for Armside
        numParameters++;
        Arm.sensePings();
        Ns_Tal = Arm.PD_T(A);
        Ns_R += Arm.PD_R();
        Avg_ErT_al = Arm.getAvg_ErT();
        Avg_ErR += Arm.getAvg_ErR();
      }
      
      else if(L != 0){        // if passed a setpoint for Legside
        numParameters++;
        Leg.sensePings();
        Ns_Tal = -Leg.PD_T(L);
        Ns_R += -Leg.PD_R();
        Avg_ErT_al = Leg.getAvg_ErT();
        Avg_ErR += Leg.getAvg_ErR();
      }

      if(Avg_ErT_fb < 2 && Avg_ErT_al < 2 && Avg_ErR/numParameters < 2){
        printReadings(F,B,A,L);
        stopDrive();
        led_Nav(0);
        return;
      }
    // Sum all speed caculations in proper orintation for mecanum drive
      Motor[0].Speed = Ns_Tfb + Ns_Tal - (Ns_R/numParameters);
      Motor[1].Speed = Ns_Tfb - Ns_Tal - (Ns_R/numParameters);
      Motor[2].Speed = Ns_Tfb - Ns_Tal + (Ns_R/numParameters);
      Motor[3].Speed = Ns_Tfb + Ns_Tal + (Ns_R/numParameters);
    
      flipMotors();
      printReadings(F,B,A,L); //print the Sonar readings to the LCD screen
      setDrive(); //everything so far stored motor change data, now tell the motors to use that data
//
//      if(!(ct++%1000)){
//      lcd.clear();
//      lcd.print(millis());
//      }

      
//      flagger = LOW;
//      while((millis()-timer)<(MAXIMUM)){ flagger = HIGH;}
//      if(flagger==LOW){
//        lcd.clear();
//        lcd.print("Exceeded MAX");
//      }

    }
  }
  

    
// ROTATION//////////////////////////////////////////////////////////////////////////////////////////

  void Rotate(int side, int Spin, int roTime){
    int RoSpeed = 0;
    float Avg_ErR;
    
    c = 0; //counter for led panel  
    uint32_t lastlight = 0; //tracking time for led panel
    Serial.println("rotate");

    Bump(Spin, roTime); // give the robot a kick in the right direction

    fill_error_arrays(); // resets error arrays
    
    while(1){
    lastlight = led_Rotate(Spin, lastlight); //LED PANEL  
    
      //cases for each side
      if (side == 0){       
        Front.sensePings();
        RoSpeed = Front.PD_R();
        Avg_ErR = Front.getAvg_ErR();
      }
      else if (side == 1){
        Arm.sensePings();
        RoSpeed = Arm.PD_R();
        Avg_ErR = Arm.getAvg_ErR();
      }
      else if (side == 2){
        Back.sensePings();
        RoSpeed = Back.PD_R();
        Avg_ErR = Back.getAvg_ErR();
      }
      else if (side == 3){
        Leg.sensePings();
        RoSpeed = Leg.PD_R();
        Avg_ErR = Leg.getAvg_ErR();
      }
      if (Avg_ErR < 2){
        stopDrive();
        led_Nav(0);
        return;
      }
      //store motor speeds according to the functions above
      Motor[0].Speed = RoSpeed;
      Motor[1].Speed = RoSpeed;
      Motor[2].Speed = -RoSpeed;
      Motor[3].Speed = -RoSpeed;
     
      flipMotors();
      setDrive(); //everything so far stored motor change data, now tell the motors to use that data
    }
  }
  
  
  
// SET;STOP;BUMP;FLIP////////////////////////////////////////////////////////////////////////////////////
// Set Drive
  void setDrive(){
    digitalWrite(EN_M0_M1, HIGH);
    digitalWrite(EN_M2_M3, HIGH);
    
    for(int i=0;i<4;i++){
      analogWrite(Motor[i].SpeedPin, Motor[i].Speed); 
    }
  }
// Stop Drive  
  void stopDrive(){
    digitalWrite(EN_M0_M1, LOW);
    digitalWrite(EN_M2_M3, LOW);
    Front.stopPings();
    Back.stopPings();
    Arm.stopPings();
    Leg.stopPings();
    
      
  }
 // Bump 
  void Bump(int spin, int RoTime){

    int x;
    int y;
    Serial.println("bump");
    if (spin == 0){                         // if CCW
     x = LOW;  y = HIGH;
    }
    else if (spin == 1){                    // if CW
     x = HIGH; y = LOW;
    }
      
    for(int i=0;i<2;i++){         // Motors 0-1 spin one direction
        digitalWrite(Motor[i].Inv_Backward_Pin, x);
        digitalWrite(Motor[i].Inv_Forward_Pin, y); 
        
    }
    for(int i=2;i<4;i++){       // Motors 2-3 spin the opposite direction
        digitalWrite(Motor[i].Inv_Backward_Pin, y);
        digitalWrite(Motor[i].Inv_Forward_Pin, x); 
       
    }
    for(int i=0;i<4;i++){       // Set Bump Speed
      Motor[i].Speed = 100; 
    }
    setDrive();
    delay(RoTime);  // rotate for a certian amount of time before stoping 
    stopDrive();
  }
 // Flip Motors 
  void flipMotors(){ 
    // if given a final negative speed, make the final speed positive and reverse the wheel direction for each motor
    for(int i=0;i<4;i++){
        if (Motor[i].Speed < 0) {
            Motor[i].Speed = -Motor[i].Speed;
            digitalWrite(Motor[i].Inv_Backward_Pin, HIGH);
            digitalWrite(Motor[i].Inv_Forward_Pin, LOW); 
        }
        else {
        digitalWrite(Motor[i].Inv_Backward_Pin, LOW);
        digitalWrite(Motor[i].Inv_Forward_Pin, HIGH); 
        }
      }
    
    // constrain the speeds of the motors to a value in between 0 and 255, the faster the motor can go is 255
    // if the speed is < 0 it gets set as 0, if the speed is > 255 it gets set as 255
    for(int i=0;i<4;i++){
      Motor[i].Speed = constrain(Motor[i].Speed,0,255);
      //condition for motors not handling torque
      Motor[i].Speed = (Motor[i].Speed <= 40) ? 0 : Motor[i].Speed;
    }
    
  }

  void fill_error_arrays(){
    Front.fill();
    Back.fill();
    Arm.fill();
    Leg.fill();
  }

  
// PIXY NAVIGATION////////////////////////////////////////////////////////////////////////////////////////////

  void PIXY_Nav(float d){        // d == -1 for the first pass (toward the boat), d == 1 for coming back
    int blockNum = 0;
    float Ns_Tfb = 0; float Avg_ErT_fb = 100;    //new speed; Avg Error; for front/back translation
    float Ns_Tal = 0; float Avg_ErT_al = 100;    //new speed; Avg Error; for arm/leg translation
    float Ns_R = 0;   float Avg_ErR = 100;       //new speed; Avg Error; for all translation

    fill_error_arrays(); // resets the error arrays
    
    while(1){
    blockNum = (int)pixySense(d);    // store info about boxcars in struct and return #boxes seen
    led_Pixy(2,d);
    Arm.sensePings();         // run normal Ping>motor PID for arm side of bot 
    Ns_Tal = Arm.PD_T(8);
    Ns_R = Arm.PD_R();
    Avg_ErT_al = Arm.getAvg_ErT();
    Avg_ErR = Arm.getAvg_ErR();
  
    Ns_Tfb = 100 * d;      // speed is constant until the end of the trains. var d give it direction
    
    if (blockNum == 1) PixyFlag = 0;
    
    else if(blockNum == 2 && PixyFlag == 0){   //if there's two blocks we're looking for the gap
            if(gapCount < 2){   // if we're not at the last gap (So we don't need to stop)
              checkGapDrop();
            }
    }
    
    else if(gapCount == 2 && PixyFlag == 0){ //trying to stop on last gap
      Ns_Tfb = Pixy_PD() * d;  //use PD to slow backward motion until stopped
      Avg_ErT_fb = Front.getAvg_ErT();
        if(Avg_ErT_fb < 2 && Avg_ErT_al < 2 && Avg_ErR < 2){ //if there
        gapCount = 0; 
        led_Pixy(1, d); 
        stopDrive();
        return;
        }       
    }
 
  
      // Sum all speed caculations in proper orintation for mecanum drive
        Motor[0].Speed = Ns_Tfb + Ns_Tal - (Ns_R);
        Motor[1].Speed = Ns_Tfb - Ns_Tal - (Ns_R);
        Motor[2].Speed = Ns_Tfb - Ns_Tal + (Ns_R);
        Motor[3].Speed = Ns_Tfb + Ns_Tal + (Ns_R);
      
        flipMotors();
        setDrive(); //everything so far stored motor change data, now tell the motors to use that data
    }
  }
    
// PIXY FUNCTIONS////////////////////////////////////////////////////////////////////////////////////////
  int pixySense(int d){
    
    uint16_t blocks = 0;
    int j, x;
    
    blocks = pixy.getBlocks();
    
    // If detecting trainCars, store info into struct 
    if (blocks){
        for (j=0; j<blocks; j++)
        {
          //detect color 
          x = pixy.blocks[j].signature;
          
          //place color string in struct
               if (x==1) box[j].color = 'r';
          else if (x==2) box[j].color = 'y';
          else if (x==3) box[j].color = 'b';
          else if (x==4) box[j].color = 'g';
          
          // get x position and width of block
          box[j].xpos = pixy.blocks[j].x;  
          box[j].width = pixy.blocks[j].width;       
        }
        if (blocks==2) sortTrains(d);
    }
    return blocks;
  }
  
  void sortTrains(int d){
    if(d==-1){
      if (box[0].xpos > box[1].xpos) swap(); //Right box is [1], biggest on approach
    }
    else if(d==1){
      if (box[1].xpos > box[0].xpos) swap(); //Left box is [1], biggest on approach
    }
  }
  
  void swap(){
      
    struct trainCar temp; /*create temp storage slot*/
    /*swap elements in struct array*/
    temp = box[0];
    box[0] = box[1];
    box[1] = temp;
  }
  
  void checkGapDrop(){
    int approach = box[1].width-box[0].width;
    if(approach <= 0){
     // dropColor(box[0].color);  //change when gripper class is
      gapCount++;
      PixyFlag = 1;
    }
  }
  float Pixy_PD(){
      float Prev_Error, Correction, NewSpeed;
      float P_T = 10; float D_T = 0.1;
      int T = 1; // condition for averaging translation in Avg_error function
      
      Prev_Error = Pixy_Error;
      Pixy_Error = (float)(box[1].width-box[0].width);
      Front.Avg_Error(Pixy_Error, T);
      Correction = P_T*(Pixy_Error) + D_T*(Pixy_Error - Prev_Error);
      NewSpeed = BaseSpeed + Correction;
      return NewSpeed;
  }
  
  void Pixy_ReadAndDrop(){
    pixySense(1);
    gripperCommand(box[0].color);
  }
  
 // TRUCK ////////////////////////////////////////////////////////////////////
void  Truck_Nav_LineUp(bool mirror, float L){
  bool TooFar_flag = 0;
    float Ns_Tal = 0; float Avg_ErT_al = 100;    //new speed; Avg Error; for front/back translation
    float Ns_Trk = 0; float Avg_ErTrk = 100;    //new speed; Avg Error; for arm/leg translation
    float Ns_R = 0;   float Avg_ErR = 100;       //new speed; Avg Error; for all translation

  fill_error_arrays();

  while (1){
    Ns_R = 0; 
    Avg_ErR = 0;
    led_Truckline(1);
  
   if(mirror){ 
    Back.sensePings();
    Ns_R = -Back.PD_R();
    Avg_ErR = Back.getAvg_ErR();
    
    Truck_R.senseTruckPings('r');
    Ns_Trk = Truck_R.Truck_PD(305);
    Avg_ErTrk = Truck_R.getAvg_ErT();

    if (TooFar_flag == 0){
        if (Back.Ping1 < 230){Ns_Trk *= -1; TooFar_flag = 1;}
      }
      if (TooFar_flag == 1){
        if (Back.Ping1 > 525){Ns_Trk *= -1; TooFar_flag = 0;}
      }
   }
   
   else{
    Front.sensePings();
    Ns_R = Front.PD_R();
    Avg_ErR = Front.getAvg_ErR();
    
    Truck_L.senseTruckPings('l');
    Ns_Trk = Truck_L.Truck_PD(305);
    Avg_ErTrk = Truck_L.getAvg_ErT();

      if (TooFar_flag == 0){
        if (Front.Ping1 < 230){Ns_Trk *= -1; TooFar_flag = 1;}
      }
      if (TooFar_flag == 1){
        if (Front.Ping1 > 525){Ns_Trk *= -1; TooFar_flag = 0;}
      }
   }
   
  Leg.sensePings();
  Ns_Tal = -Leg.PD_T(L);
  Ns_R += -Leg.PD_R();
  Avg_ErT_al = Leg.getAvg_ErT();
  Avg_ErR += Leg.getAvg_ErR();

  if(Avg_ErTrk < 2 && Avg_ErT_al < 2 && Avg_ErR/2 < 2){
      printReadings(0,1,0,1);
      stopDrive();
      Truck_L.stopPings();
      Truck_R.stopPings();
      led_Truckline(0);
      return;
    }
  // Sum all speed caculations in proper orintation for mecanum drive
    Motor[0].Speed = Ns_Trk + Ns_Tal - (Ns_R/2);
    Motor[1].Speed = Ns_Trk - Ns_Tal - (Ns_R/2);
    Motor[2].Speed = Ns_Trk - Ns_Tal + (Ns_R/2);
    Motor[3].Speed = Ns_Trk + Ns_Tal + (Ns_R/2);
  
    flipMotors();
    printReadings(0,1,0,1); //print the Sonar readings to the LCD screen
    setDrive(); //everything so far stored motor change data, now tell the motors to use that data
  }  
}

void Truck_Nav_dock(bool mirror){
  float Ns_Tal = 0; float Avg_ErT_al = 100;    //new speed; Avg Error; for arm translation
    float Ns_Trk = 0; float Avg_ErTrk1 = 100; float Avg_ErTrk2 = 100;   //new speed; Avg Error;for Truck Sideways translation
    float Ns_R = 0;   float Avg_ErR = 100;       //new speed; Avg Error; for all Rotation
    
  fill_error_arrays();
  while(1){
    led_Truck(1); 
    
    Leg.sensePings();
    Ns_R = -Leg.PD_R();
    Avg_ErR = Leg.getAvg_ErR();
  
    Truck_Arm.sensePings();
    Truck_Arm.Truck_Arm_PD(mirror);
    Avg_ErT_al = Truck_Arm.getAvg_ErT();

    Truck_L.senseTruckPings('l');
    Truck_R.senseTruckPings('r');
    Ns_Trk = Truck_L.Truck_PD(305);
    Ns_Trk -= Truck_R.Truck_PD(305);
    Avg_ErTrk1 = Truck_R.getAvg_ErT();
    Avg_ErTrk2 = Truck_L.getAvg_ErT();
    
    if((Avg_ErTrk1 > 100) || (Avg_ErTrk2 > 100)){
      Ns_Tal = 0; //if not lined up don't move forward
      led_Truck(0); 
    }
    
    if(Avg_ErT_al < 2){
        printReadings(0,1,0,1);
        stopDrive();
        Truck_L.stopPings();
        Truck_R.stopPings();
        led_Truck(2);
        return;
    }
    // Sum all speed caculations in proper orintation for mecanum drive
      Motor[0].Speed = Ns_Tal - Ns_R - Ns_Trk;
      Motor[1].Speed = -Ns_Tal - Ns_R + Ns_Trk;
      Motor[2].Speed = -Ns_Tal + Ns_R - Ns_Trk;
      Motor[3].Speed = Ns_Tal + Ns_R + Ns_Trk;
    
      flipMotors();
      printReadings(0,1,0,1); //print the Sonar readings to the LCD screen
      setDrive(); //everything so far stored motor change data, now tell the motors to use that data
  }
}


  // Gripper//////////////////////////////////////////////////////////////////////////////////////
  void gripperCommand(byte x){
    
    if (x=='s'){ //lower arm while checking for 2 alpha buttons
      int pressed = 0;
      while(pressed != 1){
        pressed = 0;
        
       // buttonStep();
        pressed += Grippers[0].buttonCheck();
       // pressed += Grippers[1].buttonCheck();
        delayMicroseconds(50);
      }
    }
    if (x=='c'){ // Alpha Senses and sends colors to beta, both manage their colors /////
      Grippers[0].senseColors();
      //Grippers[1].senseColors();
      delay(3000); // It will take a while to read all 6 colors on each Alpha
      Grippers[0].manageColors();
      //Grippers[1].manageColors();
      led_BlockColors();
    }
    if (x=='h'){ // make sure the blocks are firmly gripped before moving away
      int holding = 0;
      while(holding != 1){
        holding = 0;
        holding += Grippers[0].holdCheck(); //add 1 if 2 blocks ready
        //holding += Grippers[1].holdCheck(); //add another 1 if other 2 blocks ready
        delayMicroseconds(1000); //give the gripper a little time to work
      }
    }
    
    if ((x=='r') | (x=='y') | (x=='b') | (x=='g')){ //Send drop color command/////
      Grippers[0].dropColor(x);
      //Grippers[1].dropColor(x);
    }
    if (x=='a'){ // send drop all command  (must do this before picking up more blocks
      Grippers[0].dropAll();
      //Grippers[1].dropAll();
    }
  }
  
  // Truck and IR Slave //////////////////////////////////////////////////////////////////////////////////////////////////////
        void senseIRs() {       
         digitalWrite(SS_IR, LOW);  // open communication (direct slave into interupt routine)
           transferAndWait ('i');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (2);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           IR_package1 = transferAndWait(3); 
           IR_package2 = transferAndWait(4); 
           
         digitalWrite(SS_IR, HIGH); // close communication, but pings will continue to read
        }
       
// Transfer and Wait /////////////////////////////////////////////////////////////////////////////////////////////        
      byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
      } 
      
   void Mirror_Check() { // tells the robot which version of the course it is on
      Arm.sensePings();
      Leg.sensePings();
      if( Arm.Ping1 > Leg.Ping1) { board = 1;} // bot is on ver 1/A
      else board = 0;  // bot is on ver 2/B
    }   


   // ARM MOVEMENT FUNCTIONS TO BE CALLED BY MASTER //////////////////////////////////////////////////////////////////////
void Arm_Start_Pos(){ // how arm will be set up for the start
  limitStep(Z, 'Z','I', IN, 1, Z_DELAY);
  limitStep(BIG_Y, 'Y', 'D', DOWN, 1, BIG_Y_DELAY);
  // LIL_Y will be set by hand
  limitStep(BIG_X, 'X', 'H', LEFT_BIG_X, 1, BIG_X_DELAY); 
  limitStep(LIL_X, 'x', 'H', LEFT_LIL_X, 1, LIL_X_DELAY);
}

void Arm_Finish_Pos(){
  limitStep(Z, 'Z','I', IN, 1, Z_DELAY);
  limitStep(BIG_Y, 'Y', 'D', DOWN, 1, BIG_Y_DELAY);
  limitStep(LIL_Y, 'Y', 'U', UP, 1, LIL_Y_DELAY);
  limitStep(BIG_X, 'X', 'H', LEFT_BIG_X, 1, BIG_X_DELAY); 
  limitStep(LIL_X, 'x', 'H', LEFT_LIL_X, 1, LIL_X_DELAY);
}

void Arm_Z_Out(){
  limitStep(Z, 'Z', 'O', OUT, 1, Z_DELAY);
}

void Arm_Z_In(){
  limitStep(Z, 'Z', 'I', IN, 1, Z_DELAY);
}

void Arm_Approach_Barge_C(bool Mirror){ // arm position for approaching barge C
  limitStep(BIG_Y, 'Y', 'U', UP, 1, BIG_Y_DELAY);
    
  if(Mirror) { // side 1/A
    limitStep(LIL_X, 'x', 'R', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }
  else { // side 2/B
    limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY);
  }
}

void Arm_Approach_Barge_B(bool Mirror){ // arm position for approaching barge B
   limitStep(BIG_Y, 'Y', 'U', UP, 1, BIG_Y_DELAY);
   limitStep(LIL_Y, 'y', 'U', UP, 1, LIL_Y_DELAY);
    
  if(Mirror) { // side 1/A
    limitStep(LIL_X, 'x', 'R', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }
  else { // side 2/B
    limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY);
  }
}

void Arm_Approach_Barge_A(bool Mirror){ // arm position for approaching barge A
   limitStep(BIG_Y, 'Y', 'U', UP, 1, BIG_Y_DELAY);
   limitStep(LIL_Y, 'y', 'U', UP, 1, LIL_Y_DELAY);
}

void Arm_Leave_Barge(){ // arm position when leaving all barges but second trip to barge A
  // retract arm a bit so it does not hit things when moving to next location
            
  if(getLimits('Y', 'U') == HIGH){ // if limit switch is not hit
    Step(BIG_Y, 'Y', 'U', UP, 1, 800, BIG_Y_DELAY); // move BIG_Y up     //SEE HOW MANY STEPS ARE BEST!!!!
  }
  else {
    Step(LIL_Y, 'y', 'U', UP, 1, 800, LIL_Y_DELAY); // move LIL_Y up      //SEE HOW MANY STEPS ARE BEST!!!!
   }

}  

void Arm_Leave_BargeA(bool Mirror){ // arm position to set before leaving Barge A

  if(Mirror){ // side 1/A
    // move x to right
    limitStep(BIG_X, 'X', 'H', RIGHT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'H', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }

  else{ // side 2/B
    // move x to left
    limitStep(BIG_X, 'X', 'H', LEFT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'H', LEFT_LIL_X, 1, LIL_X_DELAY);
  }

  Arm_Leave_Barge(); // move BIG_Y up a bit
  
}

void Arm_Find_Blocks(byte axis, byte Switch){ // move the arm down until the IRs see blocks
  //drop BIG_Y until limit or IR, if needed drop LIL_Y until limit or IR
   senseIRs();  // gets packages from slave 
                                                 
  if(getLimits(axis, Switch) == 0 && IR_package1 == 11111111 && IR_package2 == 11111111 ){   // if IRs on the gripper and frame don't see anything
    limitStep(BIG_Y, 'Y', 'D', DOWN, 1, BIG_Y_DELAY); // move BIG_Y down
  } 
  
  else if(getLimits(axis, Switch) == 0 && IR_package1 == 11111111 && IR_package2 == 11111111 ){ // after moving BIG_Y if IRs still don't see anything move LIL_Y
    limitStep(LIL_Y, 'y', 'D', DOWN, 1, LIL_Y_DELAY); // move LIL_Y down
   }
}

void IR_Hunt(bool Mirror){ //  moves LIL_X until the IRs on the frame and grippers see the correct IR pattern
  senseIRs();  // gets packages from slave   
                                             
  if(Mirror) { // side 1/A
    while(IR_package1 != 192 && IR_package2 != 255){ // IRs don't see correct pattern

      if(getLimits('x', 'L') == HIGH){
         limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY); // move LIL_X left
      }
      else {
        limitStep(BIG_X, 'X', 'L', LEFT_BIG_X, 1, BIG_X_DELAY); // move BIG_X left
      }
    }
  }
  
  else { // side 2/B
    while(IR_package1 != 192 && IR_package2 != 255){ // IRs don't see correct pattern
   
      if(getLimits('x', 'R') == HIGH){
          limitStep(LIL_X, 'x', 'L', RIGHT_LIL_X, 1, LIL_X_DELAY); // move LIL_X right
      }
      else {
        limitStep(BIG_X, 'X', 'L', RIGHT_BIG_X, 1, BIG_X_DELAY); // move BIG_X right
      }
    }
  }
}  

void Arm_Boat_Pos(){ // arm position for dropping blocks in boat, DO NOT USE UNLESS NECESSARY
  limitStep(BIG_Y, 'Y',  'D', DOWN, 1, BIG_Y_DELAY); // move BIG_Y down to be close to boat
}  

void Arm_Leave_Boat(){ // arm position for arm before leaving boat, DO NOT USE UNLESS NECESSARY
  limitStep(BIG_Y, 'Y', 'U', UP, 1, BIG_Y_DELAY); // move BIG_Y UP
}

void Arm_Truck_Enter_Pos(bool Mirror){ // position arm for going into truck
 limitStep(BIG_Y, 'Y', 'D', DOWN, 1, BIG_Y_DELAY); // BIG_Y down
 limitStep(LIL_Y, 'y', 'D', DOWN, 1, LIL_Y_DELAY); // LIL_Y down

 if(Mirror){ // side is 1/A
    limitStep(BIG_X, 'X', 'L', LEFT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY);
 }
 else{ // side 2/B
    limitStep(BIG_X, 'X', 'R', RIGHT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'R', RIGHT_LIL_X, 1, LIL_X_DELAY);
 }
}  

void Arm_Truck_Leave_Pos(bool Mirror){ // position arm for leaving the truck
 limitStep(BIG_Y, 'Y', 'U', UP, 1, BIG_Y_DELAY); // BIG_Y up
 limitStep(LIL_Y, 'y', 'U', UP, 1, LIL_Y_DELAY); // LIL_Y up

 if(Mirror){ // side is 1/A
    limitStep(BIG_X, 'X', 'H', RIGHT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'H', RIGHT_LIL_X, 1, LIL_X_DELAY);
 }
 else{ // side 2/B
    limitStep(BIG_X, 'X', 'H', LEFT_BIG_X, 1, BIG_X_DELAY);
    limitStep(LIL_X, 'x', 'H', LEFT_LIL_X, 1, LIL_X_DELAY);
 }
}

void Arm_First_Train(){ // arm position for dropping blocks in first train // MIGHT NEED TO CHANGE TO LIL_Y
 limitStep(BIG_Y, 'Y', 'H', DOWN, 1, BIG_Y_DELAY);  
  }

void Arm_Train_Down(bool Mirror){ // arm position for dropping blocks in trains moving toward boat 
  if(Mirror){ // side is 1/A
    limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY);
  }
  else{ // side 2/B
    limitStep(LIL_X, 'x', 'R', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }  
}

void Arm_Train_Back(bool Mirror){ // arm position for dropping blocks in trains moving away from boat 
  if(Mirror){ // side is 1/A
    limitStep(LIL_X, 'x', 'R', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }
  else{ // side 2/B
    limitStep(LIL_X, 'x', 'L', LEFT_LIL_X, 1, LIL_X_DELAY);
  }  
}

void Arm_Leave_Train(bool Mirror){ // arm position to set before leaving trains
  if(Mirror){ // side 1/A
    limitStep(LIL_X, 'x', 'H', LEFT_LIL_X, 1, LIL_X_DELAY);
  }
  else{ // side 2/B
    limitStep(LIL_X, 'x', 'H', RIGHT_LIL_X, 1, LIL_X_DELAY);
  }
}


// FUNCTIONS NESTED IN ARM MOVEMENT FUCNTIONS ///////////////////////////////////////////////

void buttonStep(byte axis, byte Switch){ // movement of arm while master asks girpper if the buttons have been pressed
  // drop BIG_Y until limit, if needed drop LIL_Y until limit
  if(getLimits(axis, Switch) == 0 ){
    toggleStep(BIG_Y, DOWN, BIG_Y_DELAY);
  } 
 else if(getLimits(axis, Switch) == 0 ){
      toggleStep(LIL_Y, DOWN, LIL_Y_DELAY);
    }
}

void limitStep(int Stepper_SL, byte axis, byte Switch, bool spin, int stepSize, int step_delay){   // moves the stepper until the limit switch is tripped
  setModes(stepSize);
  digitalWrite(ArmMotor[Stepper_SL].Dir, spin); // sets direction of stepper
  digitalWrite(ArmMotor[Stepper_SL].Sleep, HIGH);  // enable stepper

  // ramp up
  while(  (place >= 0) && (place < (DELAYED_STEPS)) ){
    Ramp_Delay -= ( (BOTTOM_SPEED - TOP_SPEED)/ DELAYED_STEPS);
    toggleStep(Stepper_SL, spin, Ramp_Delay);
    place++;
  }
  
  while(getLimits(axis, Switch) == HIGH){ // while limit switch is not pressed
    toggleStep(Stepper_SL, spin, step_delay);
  }
  digitalWrite(ArmMotor[Stepper_SL].Sleep,LOW);
}

void Step(int Stepper_SL, byte axis, byte Switch, bool spin, int stepSize, int stepNum, int step_delay){  // moves the stepper for certain amounts of steps unless limit switch is tripped
  int countSteps = 0; // counts number of steps travelled by stepper
  setModes(stepSize);
  digitalWrite(ArmMotor[Stepper_SL].Dir, spin); // sets direction of stepper
  digitalWrite(ArmMotor[Stepper_SL].Sleep, HIGH);  // enable stepper

   // ramp up
  while(  (place >= 0) && (place < (DELAYED_STEPS)) ){
    Ramp_Delay -= ( (BOTTOM_SPEED - TOP_SPEED)/ DELAYED_STEPS);
    toggleStep(Stepper_SL, spin, Ramp_Delay);
    place++;
  }

  while(getLimits(axis, Switch) == HIGH){    // while limit switch is not pressed
    toggleStep(Stepper_SL, spin, step_delay);
    countSteps++;
    if ((countSteps-100) > stepNum) break;// if it has travelled the number of steps it was told to
  }   
  digitalWrite(ArmMotor[Stepper_SL].Sleep, LOW);  // disable stepper
}

bool getLimits(byte axis, byte Switch){ // asks LimitSwitch_Slave for limit switch data
  bool Limit = 0;
  byte Tripped = 0;
    digitalWrite(SS_LS, LOW);   
        transferAndWait (axis);  //request button condition
        transferAndWait(0);
        Tripped = transferAndWait (0);  //recieve button condition 
    digitalWrite(SS_LS, HIGH);
         Limit = (Tripped == Switch) ? 1 : 0;
          return Limit;
}

void setModes(int stepSize){ // sets the mode pins on the driver given the stepSize that is wanted
  if (stepSize == 1)   { stepMode[2] = LOW; stepMode[1] = LOW; stepMode[2] = LOW;}; // full step
  if (stepSize == 2)   { stepMode[2] = LOW; stepMode[1] = LOW; stepMode[0] = HIGH;}; // 1/2 step
  if (stepSize == 4)   { stepMode[2] = LOW; stepMode[1] = HIGH; stepMode[0] = LOW;}; // 1/4 step
  if (stepSize == 8)   { stepMode[2] = LOW; stepMode[1] = HIGH; stepMode[0] = HIGH;}; // 8 microsteps per step
  if (stepSize == 16)  { stepMode[2] = HIGH; stepMode[1] = LOW; stepMode[0] = LOW;}; // 16 microsteps per step
  if (stepSize == 32)  { stepMode[2] = HIGH; stepMode[1] = LOW; stepMode[0] = HIGH;}; // 32 microsteps per step
  // assign mode pins HIGH or LOW
  digitalWrite(MODE_0, stepMode[0]);
  digitalWrite(MODE_1, stepMode[1]);
  digitalWrite(MODE_2, stepMode[2]);
}

void toggleStep(int Stepper_SL, bool spin, int step_delay){ // toggles the Step pin on the driver, moves stepper one step
  // toggles on the rising edge
  digitalWrite(ArmMotor[Stepper_SL].Step, LOW);
  delayMicroseconds(500); // don't change this delay!!
  digitalWrite(ArmMotor[Stepper_SL].Step, HIGH);

  delayMicroseconds(step_delay);  // CHANGE LATER! we want this to be the smallest number possible that allows the motors to move
}     

      
  // PRINTS ///////////////////////////////////////////////////////////////
 void printCountdown(){
//    lcd.clear();
//    lcd.print("Ready...3");
//    delay(1000);
//    lcd.clear();
//    lcd.print("Ready...2");
//    delay(1000);
//    lcd.clear();
//    lcd.print("Ready...1");
//    delay(1000);
 }
  
  void printReadings(float F, float B, float A, float L) {
    //Function for debugging. It prints Front and Arm sonar reading to the LCD. 
    Serial.print("F  ");
    Serial.println(Front.Ping1);
    Serial.println(Front.Ping2);
    Serial.print("B  ");
    Serial.println(Back.Ping1);
    Serial.println(Back.Ping2);
    Serial.print("A  ");
    Serial.println(Arm.Ping1);
    Serial.println(Arm.Ping2);
    Serial.print("L  ");
    Serial.println(Leg.Ping1);
    Serial.println(Leg.Ping2);
    Serial.println();
    Serial.println();

//    if (F){
//      lcd.setCursor(0, 0);
//      lcd.print("F  ");   
//      lcd.print(Front.Ping1);
//      
//      lcd.setCursor(7, 0);
//      lcd.print(" ");
//      lcd.print(Front.Ping2);
//    }
//    
//    if (B){
//      lcd.setCursor(0, 0);
//      lcd.print("B  ");   
//      lcd.print(Back.Ping1);
//      
//      lcd.setCursor(7, 0);
//      lcd.print(" ");
//      lcd.print(Back.Ping2);      
//    }
//    
//    if (A){
//      lcd.setCursor(0, 1);
//      lcd.print("A  ");   
//      lcd.print(Arm.Ping1);
//      
//      lcd.setCursor(7, 1);
//      lcd.print(" "); 
//      lcd.print(Arm.Ping2);
//    }
//    
//    if (L){
//      lcd.setCursor(0, 1);
//      lcd.print("L  ");   
//      lcd.print(Leg.Ping1);
//      
//      lcd.setCursor(7, 1);
//      lcd.print(" "); 
//      lcd.print(Leg.Ping2);
//    }
  }

  void printColors_Grippers(int set){
    
    Serial.print("A0: ");   
    Serial.println(Grippers[set].AlphaColors[0]);
        
  
    Serial.print("1: ");   
    Serial.println(Grippers[set].AlphaColors[1]); 
  
   
    Serial.print("2: ");   
    Serial.println(Grippers[set].AlphaColors[2]);
  

    Serial.print("A0: ");   
    Serial.println(Grippers[set].BetaColors[0]);
        
  
    Serial.print("1: ");   
    Serial.println(Grippers[set].BetaColors[1]); 
  
   
    Serial.print("2: ");   
    Serial.println(Grippers[set].BetaColors[2]);
}

// lED PANEL ////////////////////////////////////////////////////////////////////////////
void led_Nav(bool n){
  int x[12] = {0,1,0,7,6,7,0,1,0,7,6,7};
  int y[12] = {0,0,1,0,0,1,7,7,6,7,7,6};
  for(int i = 0; i<12; i++){
    if (n == 1) matrix.drawPixel(x[i], y[i], matrix.Color(0, 255 , 0));
    else matrix.drawPixel(x[i], y[i], matrix.Color(255, 0 , 0));
  }
  matrix.show();
}

uint32_t led_Rotate(bool dir, uint32_t lastlight){
  int x[12] = {0,1,0,7,6,7,0,1,0,7,6,7}; int y[12] = {0,0,1,0,0,1,7,7,6,7,7,6};
  int cwX[4] = {2,7,5,0}; int cwY[4] = {0,2,7,5};
  int ccwX[4] = {5,7,2,0}; int ccwY[4] = {0,5,7,2}; 
  
  for(int i = 0; i<12; i++){
    matrix.drawPixel(x[i], y[i], matrix.Color(255, 150 , 0));
  }
  if (dir == 1){ //CW
    if((millis() - lastlight) > 200){
      for(int k=0;k<=c;k++){
        for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(cwX[i]+k, cwY[i], matrix.Color(0, 200 , 150));
          if(i == 1) matrix.drawPixel(cwX[i], cwY[i]+k, matrix.Color(0, 200 , 150));
          if(i == 2) matrix.drawPixel(cwX[i]-k, cwY[i], matrix.Color(0, 200 , 150));
          if(i == 3) matrix.drawPixel(cwX[i], cwY[i]-k, matrix.Color(0, 200 , 150));       
        }
      }
      c++;
     if(c == 5){
        for(int k=0;k<(c-1);k++){
          for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(cwX[i]+k, cwY[i], matrix.Color(0, 0 , 0));
          if(i == 1) matrix.drawPixel(cwX[i], cwY[i]+k, matrix.Color(0, 0 , 0));
          if(i == 2) matrix.drawPixel(cwX[i]-k, cwY[i], matrix.Color(0, 0 , 0));
          if(i == 3) matrix.drawPixel(cwX[i], cwY[i]-k, matrix.Color(0, 0 , 0));       
        }
      }
      c = 0;
     }
      lastlight = millis();
    }
    
  }
  else { //CCW
        if((millis() - lastlight) > 200){
      for(int k=0;k<=c;k++){
        for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(ccwX[i]-k, ccwY[i], matrix.Color(0, 200 , 150));
          if(i == 1) matrix.drawPixel(ccwX[i], ccwY[i]-k, matrix.Color(0, 200 , 150));
          if(i == 2) matrix.drawPixel(ccwX[i]+k, ccwY[i], matrix.Color(0, 200 , 150));
          if(i == 3) matrix.drawPixel(ccwX[i], ccwY[i]+k, matrix.Color(0, 200 , 150));       
        }
      }
      c++;
     if(c == 5){
        for(int k=0;k<(c-1);k++){
          for(int i=0; i<4; i++){
          if(i == 0) matrix.drawPixel(ccwX[i]-k, ccwY[i], matrix.Color(0, 0 , 0));
          if(i == 1) matrix.drawPixel(ccwX[i], ccwY[i]-k, matrix.Color(0, 0 , 0));
          if(i == 2) matrix.drawPixel(ccwX[i]+k, ccwY[i], matrix.Color(0, 0 , 0));
          if(i == 3) matrix.drawPixel(ccwX[i], ccwY[i]+k, matrix.Color(0, 0 , 0));       
        }
      }
      c = 0;
     }
      lastlight = millis();
    }

  }
  matrix.show();
  return lastlight;
}
void led_Truckline(bool n){
  int x[12] = {0,1,0,7,6,7,0,1,0,7,6,7};
  int y[12] = {0,0,1,0,0,1,7,7,6,7,7,6};
  for(int i = 0; i<12; i++){
    if (n == 1) { 
    matrix.drawPixel(x[i], y[i], matrix.Color(0, 0 , 255));
    matrix.show();
    }
    else{ 
      for(int i=0; i<4; i++){
        for(int j=0; j<12; j++){
            matrix.drawPixel(x[j], y[j], matrix.Color(0, 0 , 255));
            matrix.show();
        }
        delay(150);
        for(int j=0; j<12; j++){
            matrix.drawPixel(x[j], y[j], matrix.Color(0, 0 , 0));
            matrix.show();
        }
        delay(150);
        }
    }
  }
  
}

void led_Truck(int n){
    int x[4] = {0,1,6,7};
  
  if (n==0){
    for(int i=0;i<4;i++){
      matrix.drawFastVLine(x[i],0,8,matrix.Color(255, 0 , 0));
    }
    matrix.show(); 
  }
  if (n==1){
    for(int i=0;i<4;i++){
      matrix.drawFastVLine(x[i],0,8,matrix.Color(0, 0 , 255));
    }
    matrix.show(); 
  }
  if (n==2){
    matrix.fillScreen(0);
    matrix.show();
    delay(500);
    for(int k=0; k<3;k++){
      int j = 8;
      for(int i=0; i<4; i++){
        matrix.drawRect(i,i,j,j,matrix.Color(0, 0 , 255));
        j -= 2;
        matrix.show(); 
        delay(100);
      }
      matrix.fillScreen(0);
      matrix.show();
      delay(250);
    }
    led_Truckline(1);
    led_BlockColors();
    
  }
}
void led_BlockColors(){
 int y=6;
 int x=2;
 for(int k=0;k<2;k++){
   for(int i=0; i<3; i++){ 
     matrix.drawPixel(x, y, Al_Color(i,k));
     matrix.show();
     y--;
   }
   x++;
   y=6;
   for(int i=0; i<3; i++){ 
     matrix.drawPixel(x, y, Be_Color(i,k));
     matrix.show();
     y--;
   }
   x++;
   y=6;
 }
}


uint16_t Al_Color(int i, int set){
  if(Grippers[set].AlphaColors[i]==0) return matrix.Color(255, 0 , 0);
  if(Grippers[set].AlphaColors[i]==1) return matrix.Color(255, 255 , 0);
  if(Grippers[set].AlphaColors[i]==2) return matrix.Color(0, 0 , 255);
  if(Grippers[set].AlphaColors[i]==3) return matrix.Color(0, 255 , 0);
}
uint16_t Be_Color(int i, int set){
  if(Grippers[set].BetaColors[i]==0) return matrix.Color(255, 0 , 0);
  if(Grippers[set].BetaColors[i]==1) return matrix.Color(255, 255 , 0);
  if(Grippers[set].BetaColors[i]==2) return matrix.Color(0, 0 , 255);
  if(Grippers[set].BetaColors[i]==3) return matrix.Color(0, 255 , 0);
}

void led_Pixy(int n, int d){
  matrix.drawRect(0,0,2,7,matrix.Color(200, 200 , 200));
  matrix.drawRect(6,0,2,7,matrix.Color(200, 200 , 200));
  matrix.drawFastHLine(0,7,8,matrix.Color(200, 200 , 200));
  matrix.drawFastHLine(2,3,4,matrix.Color(200, 200 , 200));
  
  if(n==1){
    matrix.drawRect(2,0,2,3,matrix.Color(255, 0 , 0));
    matrix.drawRect(4,0,2,3,matrix.Color(255, 0 , 0));
    matrix.show();  
  }
  if(n==2){
    if(d==1){
      matrix.drawRect(2,0,2,3,led_pixyColor(0));
      matrix.drawRect(4,0,2,3,led_pixyColor(1));
      matrix.show();  
    }
    if(d==-1){
      matrix.drawRect(2,0,2,3,led_pixyColor(1));
      matrix.drawRect(4,0,2,3,led_pixyColor(0));
      matrix.show();  
    }
  }
}

uint16_t led_pixyColor(int i){ 
  if(box[i].color=='r') return matrix.Color(255, 0 , 0);
  if(box[i].color=='y') return matrix.Color(255, 255 , 0);
  if(box[i].color=='b') return matrix.Color(0, 0 , 255);
  if(box[i].color=='g') return matrix.Color(0, 255 , 0);
}  
