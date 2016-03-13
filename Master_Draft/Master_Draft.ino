
/* Corey Pullium/ March 2016 / IEEE SoutheastCon

 */
  // include the library code:
  #include <LiquidCrystal.h> 
  #include <NewPing.h>
  #include <SPI.h>  
  #include <Pixy.h>
 
  #define ER_ARRAY_SIZE 30
  
  //Side Pings
  #define SS_2 38 
  #define SS_3 40
  #define SS_4 42
  #define SS_5 44
  #define SS_T 20

  #define SS_BG1 15
  #define SS_BG2 16 
  #define SS_AG1 17 
  #define SS_AG2 18 
    
  #define M0_IN1 48
  #define M0_IN2 49
  #define M0_D2 4

  #define M1_IN1 34
  #define M1_IN2 35
  #define M1_D2 5 
  
  #define M2_IN1 30
  #define M2_IN2 31
  #define M2_D2 3
  
  #define M3_IN1 33
  #define M3_IN2 32
  #define M3_D2 2
  
  #define EN_M0_M1 46
  #define EN_M2_M3 22

  //checking millis for loop time correction
  long timer = 0;
  long ct = 1;
  #define MAXIMUM 15
  bool flagger;

  
  // initialize the library with the numbers of the interface pins
  LiquidCrystal lcd(39, 41, 43, 45, 47, 37);

  Pixy pixy;

  struct trainCar{
  byte color; 
  int xpos; 
  int width;
  };
  
  float BaseSpeed = 0;
  
  struct Motors {   //Motor Values   
    int Inv_Backward;  //high = backwards
    int Inv_Forward;      //High = forwards
    int Speed;
    
    int Inv_Backward_Pin;  //high = backwards
    int Inv_Forward_Pin; 
    int SpeedPin;
  };
  
  //Motor numbers(Picture Below shows motor numbers);
  //        F
  //    ||-2 1-|| 
  // A  ||-3 0-||   L
  //        B
// End of Declarations//////////////////////////////////////////////////////////////////////////////////

// GRIPPER CLASS DEFINITION //////////////////////////////////////////////////////////////////////////////////////
  class Gripperset {
   public: 
    int SSL; //Slave select for the Alpha
    int SS_B; //Slave select for it's Beta
    int AlphaColors[3];  //Colors sent from Alpha to display
    int BetaColors[3]; //Beta Colors sent from Alpha. These will be displayed and sent to beta
    byte Alpha; //byte holding encoded colors for Alpha
    byte Beta; //byte holding encoded colors for Beta
    
    byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
    } 
      
    int buttonCheck(){ // Looking for Button on Alpha, telling Beta///////////////////
      byte checkA = 0; //message recieved from Alpha 
      int touchdown = 0;  
        digitalWrite(SSL, LOW);   
          checkA =transferAndWait ('s');  //request button condition
          checkA =transferAndWait ('s');  //recieve button condition 
          //after frist transfer it could recieve on either transfer
        digitalWrite(SSL, HIGH);
        // Master will recieve B11110000, if button is pressed
        //Next we let the Beta know if the button was pressed
        
        digitalWrite(SS_B, LOW);   
          transferAndWait ('s'); //ready Beta for transfer
          transferAndWait (checkA);  //send it Alpha button condition
        digitalWrite(SS_B, HIGH);
        
        //check if the message meets condition 
        touchdown = (checkA == B11110000) ? 1 : 0;
        return touchdown; 
     }
    
    void senseColors(){ // Tell alpha to fire color sensors /////////////
        digitalWrite(SSL, LOW);   
          transferAndWait ('c');   //prep Alpha to take color readings
          transferAndWait (0);    //take color readings 
        digitalWrite(SSL, HIGH);
      
    }
    
    void manageColors(){ // Request color readings from Alpha ///////////
       byte decoder = B00110000;
       
        digitalWrite(SSL, LOW);   
          transferAndWait (1);  // request Alpha send packages
          transferAndWait (2);  //Alpha preps package[0]
          Alpha = transferAndWait (0); //recieve alpha colors, prep package[1]
          Beta = transferAndWait (0); // recieve beta colors 
        digitalWrite(SSL, HIGH); 

        
        for (int i=0;i<3;i++){ //unpack 3 alpha colors
          for(int j=4;i<3;j-2){ //change bit shifting by 2 each time
            //fill color array by viewing byte Alpha with the decoder mask
           
              AlphaColors[i] = (int)(Alpha & decoder); 
              AlphaColors[i]>>=j; //shift decoded number until its on 2 bits
              decoder >>= 2;  //shift the mask down to the next 2 bits
              
            }
        }
        decoder = B00110000; // reset decoder
        
        for (int i=0;i<3;i++){ // unpack beta colors as above
          for(int j=4;i<3;j-2){
            BetaColors[i] = Beta & decoder;
            BetaColors[i]>>=j;
            decoder >>= 2;
            
          }
        }
        //send beta colors to the beta slave
        digitalWrite(SS_B, LOW);   
          transferAndWait ('c'); //request to send beta its colors
          transferAndWait (Beta); //beta gets colors 
          transferAndWait (0); //tell beta to move on to managing colors and gripping
        digitalWrite(SS_B, HIGH);  
        
    }
    
    int holdCheck(){
      byte checkA = 0; byte checkB = 0;
      int holding = 0;  
        digitalWrite(SSL, LOW);   
          checkA =transferAndWait ('h');  //request gripping condition from Alpha
        digitalWrite(SSL, HIGH);
        
        digitalWrite(SS_B, LOW);   
          checkB =transferAndWait ('h');  //request gripping condition from Beta 
        digitalWrite(SS_B, HIGH);
        
        //then see if both Alpha and Beta send the "we have them" message
        holding = ((checkA == B00001111) & (checkB == B00001111)) ? 1 : 0;
        return holding;
    }
    
    void dropColor(byte x){ // Transfer a color to the Beta and Alpha //////////////
        digitalWrite(SS_B, LOW);   
          transferAndWait (x);  //send color and prep beta
          transferAndWait (0);  // beta executes drop
        digitalWrite(SS_B, HIGH);
        
        digitalWrite(SSL, LOW);   
          transferAndWait (x); //send color and prep alpha  
          transferAndWait (0);  // alpha executes drop
        digitalWrite(SSL, HIGH);
        
    }
    
    void dropAll(){ // Drops all Blocks (should be done before picking up new blocks)
      digitalWrite(SS_B, LOW);   
        transferAndWait ('a');  //prep beta
        transferAndWait (0);  //execute
      digitalWrite(SS_B, HIGH);
      
      digitalWrite(SSL, LOW);   
        transferAndWait ('a');   //prep alpha
        transferAndWait (0);    //execute
      digitalWrite(SSL, HIGH);
    }
    
  };
  
  
  // SIDE CLASS DEFINITION///////////////////////////////////////////////////////////////////////////
  class Side { 
    public:
        //initialized values
        int PingPin1;   //sensor number
        int PingPin2;
        
        float P_T;      // PD constants
        float D_T;
        float P_R;
        float D_R;
        
        // found values
        float Ping1;  //for storing distance
        float Ping2;
        
        float Error_T;
        float Error_R;
        
        float Error_T_History[ER_ARRAY_SIZE];
        float Error_R_History[ER_ARRAY_SIZE];
        float Avg_ErT;
        float Avg_ErR;

        int SSL;
        //////////////////////////////////////////////////
                            //sense distance for each side
        void sensePings() {                
         digitalWrite(SSL, LOW);  // open communication (direct slave into interupt routine)
           transferAndWait ('f');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (2);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           Ping1 = ((int)transferAndWait(3) << 8) | (int)transferAndWait(4); 
           Ping2 = ((int)transferAndWait(0) << 8) | (int)transferAndWait(0); 
         digitalWrite(SSL, HIGH); // close communication, but pings will continue to read
        }
        
        void stopPings(){
          digitalWrite(SSL, LOW);   
          transferAndWait ('q');  // add command 
          transferAndWait (2);  // add command
          digitalWrite(SSL, HIGH);
          
        }
       byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
      } 
        
        float PD_T(float setpoint){ // PD caculation for translation
          float Prev_Error, Correction, NewSpeed;
          int T = 1; // condition for averaging translation in Avg_error function
          
          Prev_Error = Error_T;
          Error_T = (Ping1 + Ping2)/2 - setpoint; 
          Avg_Error(Error_T, T);
          Correction = P_T*(Error_T) + D_T*(Error_T - Prev_Error);
          NewSpeed = BaseSpeed + Correction;
         
          return NewSpeed;
        }
    
        
        float PD_R(){ // PD caculation for rotation
          float Prev_Error, Correction, NewSpeed;
          int R = 0;  // condition for averaging Rotation in Avg_error function
    
          Prev_Error = Error_R; 
          Error_R = -(Ping1 - Ping2);
          Avg_Error(Error_R, R);
          Correction = P_R*(Error_R) + D_R*(Error_R - Prev_Error);
          NewSpeed = BaseSpeed + Correction; 
          return NewSpeed; //Positive is ClockWise
        }


        //This is for the exit conditions.
        void Avg_Error(float Er, int type){    //Shifts the array of errors then adds the most recent to the 0 spot
          if (type == 1){                      //Translation                                        
            for(int i=ER_ARRAY_SIZE-1; i>0; i--){   //Shifts        
              Error_T_History[i] = Error_T_History[i-1];
            }
            Error_T_History[0] = Er;
                                               //Sums/w abs() and finds Average
            for(int k=0;k<ER_ARRAY_SIZE;k++){
              Avg_ErT += abs(Error_T_History[k]);
            }
            Avg_ErT = Avg_ErT/ER_ARRAY_SIZE;
          }
          else if(type==0){                   //Rotation
            for(int i=ER_ARRAY_SIZE-1; i>0; i--){  //Shifts
              Error_R_History[i] = Error_R_History[i-1];
            }
            Error_R_History[0] = Er;
            for(int k=0;k<ER_ARRAY_SIZE;k++){       //Sums/w abs() and finds Average
              Avg_ErR += abs(Error_T_History[k]);
            }
            Avg_ErR = Avg_ErR/ER_ARRAY_SIZE;
          }
        }

        float getAvg_ErT(){     //needed to send Error Avg because PD function can't return 2 things
          return Avg_ErT;
        }
        float getAvg_ErR(){     //needed to send Error Avg because PD function can't return 2 things
          return Avg_ErR;
        }
        
        void fill(){                //sets the error array very high
          for(int i=0;i<ER_ARRAY_SIZE;i++){
             Error_T_History[i]=100;
             Error_R_History[i]=100;
          }
        }

  };  

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

   //Grippers
   Gripperset Grippers[2] = {
    {SS_AG1, SS_BG1, {}, {}, 0, 0},
    {SS_AG2, SS_BG2, {}, {}, 0, 0}
   }; 
   
  //Sides
         //Pin1,  Pin2,  PT,   DT,   PR,     DR
  Side Front {0,   1,    1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_2};
  Side Back  {2,   3,    1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_3};
  Side Arm   {4,   5,    1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_4};
  Side Leg   {6,   7,    1,   4,   0.01,  0,  0, 0, 0, 0, {},{},0,0, SS_5};
    
// SETUP///////////////////////////////////////////////////////////////////////////////////////////
  
  void setup() {
    // for communicating with Processing
    Serial.begin(115200);
    
    lcd.begin(16, 2); // set up the LCD's number of columns and rows
    lcd.print("IEEE TESTING");   // Print a message to the LCD.
    delay(2000);
    lcd.clear();

    pixy.init(); //Initiate the PIXY
    
    pinMode (EN_M0_M1, OUTPUT);
    pinMode (EN_M2_M3, OUTPUT);
    digitalWrite(EN_M0_M1, HIGH);
    digitalWrite(EN_M2_M3, HIGH);
    for(int i=0;i<4;i++){  //configure pin modes
      
      pinMode (Motor[i].Inv_Backward_Pin, OUTPUT);
      pinMode (Motor[i].Inv_Forward_Pin, OUTPUT); 
      pinMode (Motor[i].SpeedPin, OUTPUT); 
    }
    
    //All Ping SPI slave select pins
    pinMode(SS_2, OUTPUT); 
    digitalWrite(SS_2, HIGH);  
    pinMode(SS_3, OUTPUT); 
    digitalWrite(SS_3, HIGH);
    pinMode(SS_4, OUTPUT); 
    digitalWrite(SS_4, HIGH);
    pinMode(SS_5, OUTPUT); 
    digitalWrite(SS_5, HIGH);
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
    
  lcd.clear();
  lcd.print("Ready...3");
  delay(1000);
  lcd.clear();
  lcd.print("Ready...2");
  delay(1000);
  lcd.clear();
  lcd.print("Ready...1");
  delay(1000);


  Nav(1760, 0, 50, 0);
  lcd.clear();
  lcd.print("here");
  delay(5000);
 
//  Nav(150,0,0, 150); //Navigate out of tunnel using Front and Leg
//  lcd.clear();
//  lcd.print("here");
//  delay(5000);
//
//  Rotate(A, CW,1000); //Rotate Arm to Bardge C
//  lcd.clear();
//  lcd.print("Rotated");
//  delay(5000);
//  
//  Nav(300,150,0,0); //Center with Front and Arm
//  lcd.clear();
//  lcd.print("Im Here");
//  delay(5000);
//  
//  lcd.clear();
//  lcd.print("Turning off");
//  delay(1500);
  
 
//  
//  Rotate(L, CCW, 500); //Rotate to boat unload orientation 
//  Nav(30,0,0,30);
//  delay(5000);
//  
//  Nav(30,0,0,100); //Navigate to Middle B using Front and Leg
//  lcd.clear();
//  lcd.print("Done");
//  delay(10000);
  
  }
  
  

  // NAVIGATION/////////////////////////////////////////////////////////////////////////////////////////////
  
  void Nav(float F, float B, float A, float L){
    float numParameters = 0;                      //number of non-Zero parameters 
    float Ns_Tfb = 0; float Avg_ErT_fb = 100;    //new speed; Avg Error; for front/back translation
    float Ns_Tal = 0; float Avg_ErT_al = 100;    //new speed; Avg Error; for arm/leg translation
    float Ns_R = 0;   float Avg_ErR = 100;       //new speed; Avg Error; for all translation
    
    
    fill_error_arrays();
    
    while (1){ 
        //  timer = millis();
      //Process_PD();
      
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
        Avg_ErT_al = Back.getAvg_ErT();
        Avg_ErR += Back.getAvg_ErR();
      }

      if(Avg_ErT_fb < 2 && Avg_ErT_al < 2 && Avg_ErR/numParameters < 2){
        printReadings(F,B,A,L);
        stopDrive();
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
    Serial.println("rotate");
    Bump(Spin, roTime); // give the robot a kick in the right direction

    fill_error_arrays(); // resets error arrays
    
    while(1){
      
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
  void setDrive(){
    digitalWrite(EN_M0_M1, HIGH);
    digitalWrite(EN_M2_M3, HIGH);
    
    for(int i=0;i<4;i++){
      analogWrite(Motor[i].SpeedPin, Motor[i].Speed); 
    }
  }
  
  void stopDrive(){
    digitalWrite(EN_M0_M1, LOW);
    digitalWrite(EN_M2_M3, LOW);
      
  }
  
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

  // Gripper//////////////////////////////////////////////////////////////////////////////////////
  void gripperCommand(byte x){
    
    if (x=='s'){ //lower arm while checking for 2 alpha buttons
      int pressed = 0;
      while(pressed != 2){
        pressed = 0;
        //armMovesDown();
        pressed += Grippers[0].buttonCheck();
        pressed += Grippers[1].buttonCheck();
        delayMicroseconds(50);
      }
    }
    if (x=='c'){ // Alpha Senses and sends colors to beta, both manage their colors /////
      Grippers[0].senseColors();
      Grippers[1].senseColors();
      delay(3000); // It will take a while to read all 6 colors on each Alpha
      Grippers[0].manageColors();
      Grippers[1].manageColors();
    }
    if (x=='h'){ // make sure the blocks are firmly gripped before moving away
      int holding = 0;
      while(holding != 2){
        holding = 0;
        holding += Grippers[0].holdCheck(); //add 1 if 2 blocks ready
        holding += Grippers[1].holdCheck(); //add another 1 if other 2 blocks ready
        delayMicroseconds(1000); //give the gripper a little time to work
      }
    }
    
    if ((x=='r') | (x=='y') | (x=='b') | (x=='g')){ //Send drop color command/////
      Grippers[0].dropColor(x);
      Grippers[1].dropColor(x);
    }
    if (x=='a'){ // send drop all command
      Grippers[0].dropAll();
      Grippers[1].dropAll();
    }
  }
  
  // Truck //////////////////////////////////////////////////////////////////////////////////////////////////////
        void truckSensePings() {  
         float Ping_T1 = 0; float Ping_T2 = 0; float Ping_T3 = 0; float Ping_T4 = 0;
                        
         digitalWrite(SS_T, LOW);  // open communication (direct slave into interupt routine)
           transferAndWait ('f');  // asks to turn pings on, and sets up the first byte transfer
           transferAndWait (2);   //pings are on and first request is recieved  
           //get leading bits, shift them left, get trailing bits, splice them together
           Ping_T1 = ((int)transferAndWait(3) << 8) | (int)transferAndWait(4); 
           Ping_T2 = ((int)transferAndWait(5) << 8) | (int)transferAndWait(6); 
           Ping_T3 = ((int)transferAndWait(7) << 8) | (int)transferAndWait(8); 
           Ping_T4 = ((int)transferAndWait(0) << 8) | (int)transferAndWait(0); 
         digitalWrite(SS_T, HIGH); // close communication, but pings will continue to read
        }
        
        void TruckstopPings(){
          digitalWrite(SS_T, LOW);   
          transferAndWait ('q');  // add command 
          transferAndWait (2);  // add command
          digitalWrite(SS_T, HIGH);
          
        }
// Transfer and Wait /////////////////////////////////////////////////////////////////////////////////////////////        
      byte transferAndWait (const byte what){  // function does SPI transfer and delays enough to complete
         byte a = SPI.transfer (what);
         delayMicroseconds (20);
         return a;
      } 
  ///////////////////////////////////////////////Print Sonar Readings////////////////////////////////////////////////
  void printReadings(float F, float B, float A, float L) {
    //Function for debugging. It prints Front and Arm sonar reading to the LCD. 
    if (F){
      lcd.setCursor(0, 0);
      lcd.print("F  ");   
      lcd.print(Front.Ping1);
      
      lcd.setCursor(7, 0);
      lcd.print(" ");
      lcd.print(Front.Ping2);
    }
    
    if (B){
      lcd.setCursor(0, 0);
      lcd.print("B  ");   
      lcd.print(Back.Ping1);
      
      lcd.setCursor(7, 0);
      lcd.print(" ");
      lcd.print(Back.Ping2);      
    }
    
    if (A){
      lcd.setCursor(0, 1);
      lcd.print("A  ");   
      lcd.print(Arm.Ping1);
      
      lcd.setCursor(7, 1);
      lcd.print(" "); 
      lcd.print(Arm.Ping2);
    }
    
    if (L){
      lcd.setCursor(0, 1);
      lcd.print("L  ");   
      lcd.print(Leg.Ping1);
      
      lcd.setCursor(7, 1);
      lcd.print(" "); 
      lcd.print(Leg.Ping2);
    }
  }


  
 void Process_PD(){
    // for temporarily holding values of constants that have been recieved from processing
  byte pt; byte dt; byte pr; byte dr;
  int PT_total = 0; int DT_total = 0; int PR_total = 0; int DR_total = 0;
  int side;
  bool dataReady = LOW;
  int index = 0;
  int bufferArray[10];
  float Mag;
  float scale = 0;
  
  if(Serial.available()){  // if there is serial data to read...
         bufferArray[index++] = Serial.read(); // load the current byte into the current array index location
         if(index == 10){
          index=0;
          dataReady = HIGH;
         }

   
         lcd.print("v");
     }

    if(dataReady){

      lcd.clear();
      lcd.print("im here");
      // setting value of magnitude
      scale = (float)bufferArray[9];

      if(scale == 1) Mag = 100;

      if(scale == 2) Mag = 10;

      if(scale == 3) Mag = 1;

      if(scale == 4) Mag = 0.1;

      if(scale == 5) Mag = 0.01;
      
    // if side is front
      if(bufferArray[0]==0){
        PT_total = bufferArray[1];
        PT_total <<= 8;
        PT_total = PT_total | bufferArray[2];
        Front.P_T = (float)PT_total/Mag;
        Back.P_T = (float)PT_total/Mag;
      
        DT_total = bufferArray[3];
        DT_total <<= 8;
        DT_total = DT_total | bufferArray[4];
        Front.D_T = (float)DT_total/Mag;
        Back.D_T = (float)DT_total/Mag;
      
        PR_total = bufferArray[5];
        PR_total <<= 8;
        PR_total = PR_total | bufferArray[6];
        Front.P_R = (float)PR_total/Mag;
        Back.P_R = (float)PR_total/Mag;
      
        DR_total = bufferArray[7];
        DR_total <<= 8;
        DR_total = DR_total | bufferArray[8];
        Front.D_R = (float)DR_total/Mag;
        Back.D_R = (float)DR_total/Mag;

         lcd.print("yoyo");

      }   
  
  // if side is Arm
      if(bufferArray[0]==1){
        PT_total = bufferArray[1];
        PT_total <<= 8;
        PT_total = PT_total | bufferArray[2];
        Arm.P_T = (float)PT_total/Mag;
        Leg.P_T = (float)PT_total/Mag;
      
        DT_total = bufferArray[3];
        DT_total <<= 8;
        DT_total = DT_total | bufferArray[4];
        Arm.D_T = (float)DT_total/Mag;
        Leg.D_T = (float)DT_total/Mag;
      
        PR_total = bufferArray[5];
        PR_total <<= 8;
        PR_total = PR_total | bufferArray[6];
        Arm.P_R = (float)PR_total/Mag;
        Leg.P_R = (float)PR_total/Mag;
      
        DR_total = bufferArray[7];
        DR_total <<= 8;
        DR_total = DR_total | bufferArray[8];
        Arm.D_R = (float)DR_total/Mag;
        Leg.D_R = (float)DR_total/Mag;
 
      }   
         dataReady = LOW;
    }
 }


