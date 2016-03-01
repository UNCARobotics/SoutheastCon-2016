/* Corey Pullium/ Feb 2016 / IEEE SoutheastCon
 *  
 * This code is the coneptualization of a navigation function that provides setpoints to a robot
 * and a rotation function that sets up a 90 turn. 
 * The robot uses these setpoints in conjuction with sonar and a PD controller 
 * to navigate directly to the given (X,Z) coordinate.
 * 
 * The 3 noteable sections: 
 * 
 * Class "side" allows for the creation of objects representing each side of the robot, containing it's sensor 
 * retreval method and the PD controllor methods. (this accounts for translation, and correctional rotation)
 * 
 * The Navigation function takes a setpoint for each side of the robot, then asks each side object to 
 * sense and caculate it's own needed adjustments. It then sums adjustments needed from each side and 
 * directs the motors accordingly. 
 * 
 * The Rotate function is past a side to take sonar readings, a direction of rotation and a time. 
 * The time and direction are simple to push the robot in the right direction. Then the function points 
 * the given side to the corner. After that, calling Nav(), again finishes the turn with it's own rotational correction.
 */
  // include the library code:
  #include <LiquidCrystal.h> 
  #define ER_ARRAY_SIZE 7
  // initialize the library with the numbers of the interface pins
  LiquidCrystal lcd(41, 43, 45, 47, 49, 39);
 
  // for temporarily holding values of constants that have been recieved from processing
  byte pt;
  byte dt;
  byte pr;
  byte dr;
  int side;
  bool dataReady = LOW;
  int index = 0;
  int bufferArray[10];
  int PT_total = 0;
  int DT_total = 0;
  int PR_total = 0;
  int DR_total = 0;
  float Mag;
  float scale = 0;
   
  ////////////////////////////////////////////END OF INITIALIZATION///////////////////////////////////////////////////
 
  class Side { /////////////////////////////SIDE CLASS DEFINITION/////////////////////////////////////////////////////
    public:
        //initialized values
       
        // PD constants
        float P_T;   
        float D_T;
        float P_R;
        float D_R;  
  };     
   
  //intitiate sides:////////////////////////////////////////////////////
             //  P_T,   D_T,   P_R,   D_R
  Side Front {0,   0,   0,   0};
  Side Back  {0,   0,   0,   0};
  Side Arm   {0,   0,   0,   0};
  Side Leg   {0,   0,   0,   0};

  ////////////////////////////////////////////////SETUP///////////////////////////////////////////////////////////////
  
  void setup() {
    lcd.begin(16, 2); // set up the LCD's number of columns and rows
    lcd.print("IEEE TESTING");   // Print a message to the LCD.
    delay(2000);
    lcd.clear();
    
     // for communicating with Processing
     Serial.begin(115200);
  }
  
   ////////////////////////////////////////////////////LOOP/////////////////////////////////////////////////////// 
  void loop(){
   
  Nav(20,0,0,20); //Navigate out of tunnel using Front and Leg
  }
 
  ///////////////////////////////////////////////////NAVIGATION/////////////////////////////////////////////////////////
  
  void Nav(float F, float B, float A, float L){

  // reading data from serial monitor from processing
    if(Serial.available()){  // if there is serial data to read...
         bufferArray[index++] = Serial.read(); // load the current byte into the current array index location
         if(index == 10){
          index=0;
          dataReady = HIGH;
         }
     }

    if(dataReady){
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
      }   
  
  // if side is front
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
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("PR ");   
    lcd.print(Front.P_R);
    
    lcd.setCursor(8, 0);
    lcd.print("DR ");
    lcd.print(Front.D_R);

    lcd.setCursor(0, 1);
    lcd.print("PT ");
    lcd.print(Front.P_T);
    
    lcd.setCursor(8, 1);
    lcd.print("DT ");
    lcd.print(Front.D_T);
    
    delay(100);
    
  }
  


  

