// hunt for blocks
struct IR{
  uint8_t pin;
  uint32_t lastDebounce;
  uint8_t lastRead = 0;
  uint8_t state = 0;
};

int IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8, IR_9, IR_10, IR_11, IR_12;
int count =0;
struct IR grippers[6], frame[6];
  
// IRs 1, 4, 5, 8, 9, 12 are in the grippers and have 10cm and less range
// IRs 2, 3, 6, 7, 10, 11 are on the frame and have 15cm and less range
// 1 means it doesn't see anything, 0 means it does

void setup() {
  Serial.begin(115200);
  Serial.println("IEEE TESTING");
  Serial.println("////////////////////////////////////////////////");
    Serial.println("////////////////////////////////////////////////");
  delay(1000);
  

  grippers[0].pin = 4;
  grippers[1].pin = 9;
  grippers[2].pin = 7;
  grippers[3].pin = 15;
  grippers[4].pin = 16;
  grippers[5].pin = 19;


  for(int i = 0; i < 6; i++)
  {
    pinMode(grippers[i].pin, INPUT);
  }

}

void loop()
{
  bool centered = true;

  uint8_t reading;
  for(int i = 0; i < 6; i++)
  {
    reading = digitalRead(grippers[i].pin);
    
    if(grippers[i].lastRead != reading)
      grippers[i].lastDebounce = millis();
      
    if((millis() - grippers[i].lastDebounce) > 25)
    {
      grippers[i].state = reading;
    }
    
    if(grippers[i].state)
    {
//      Serial.print(grippers[i].pin);
//      Serial.print(" ");
      centered = false;
    }
    grippers[i].lastRead = reading;
  }
  
  if(centered){
  Serial.print("Centered!!");
  Serial.println(count++);
  }
  
  
//  // Grippers
//  IR_1 = digitalRead(4);
//  IR_4 = digitalRead(9);
//  IR_5 = digitalRead(7);
//  IR_8 = digitalRead(15);
//  IR_9 = digitalRead(16);
//  IR_12 = digitalRead(19);
//  
  // Frame
  IR_2 = digitalRead(3);
  IR_3 = digitalRead(2);
  IR_6 = digitalRead(6);
  IR_7 = digitalRead(5);
  IR_10 = digitalRead(17);
  IR_11 = digitalRead(18);



//  Serial.print("Gripper: ");
//
//  Serial.print(IR_1);
//    Serial.print(" ");
//       Serial.print(IR_4);
//   Serial.print(" ");
//      Serial.print(IR_5);
//     Serial.print(" ");
//      Serial.print(IR_8);
//    Serial.print(" ");
//  Serial.print(IR_9);
//     Serial.print(" ");
//     Serial.print(IR_12);
//  Serial.print("\t");
//
//Serial.print("Frame: ");
// Serial.print(IR_2);
//   Serial.print(" ");
//  Serial.print(IR_3);
//     Serial.print(" ");


//  Serial.print(IR_6);
//     Serial.print(" ");
//
//  Serial.print(IR_7);
//     Serial.print(" ");
//
//  Serial.print(IR_10);
//     Serial.print(" ");
//  Serial.print(IR_11);
//     Serial.println();
// 


























//  IR_1 = analogRead(4);
//  IR_4 = analogRead(9);
//  IR_5 = analogRead(7);
//  IR_8 = analogRead(15);
//  IR_9 = analogRead(16);
//  IR_12 = analogRead(19);
//
//
// Serial.print(IR_1);
//   Serial.print(" ");
//  Serial.print(IR_4);
//     Serial.print(" ");
//
//  Serial.print(IR_5);
//     Serial.print(" ");
// Serial.print(IR_8);
//     Serial.print(" ");
//
// Serial.print(IR_9);
//     Serial.print(" ");
//  Serial.print(IR_12);
//     Serial.print(" ");
//Serial.println("");
//
//
//












  //========================================
 
//  IR_2 = analogRead(3);
//  IR_3 = analogRead(2);
//  
//  IR_6 = analogRead(6);
//  IR_7 = analogRead(5);
//  
//  IR_10 = analogRead(17);
//  IR_11 = analogRead(18);
//
//
//  
// Serial.print(IR_2);
//   Serial.print(" ");
//  Serial.print(IR_3);
//     Serial.print(" ");
//
//  Serial.print(IR_6);
//     Serial.print(" ");
// // Serial.print(IR_5);
//  Serial.print(IR_7);
//     Serial.print(" ");
// // Serial.print(IR_8);
// // Serial.print(IR_9);
//  Serial.print(IR_10);
//     Serial.print(" ");
//  Serial.print(IR_11);
//     Serial.print(" ");
// // Serial.print(IR_12);
//  Serial.println();




}




//
//// IR_1 = digitalRead(4);
//  IR_2 = analogRead(3);
//  IR_3 = analogRead(2);
//  IR_4 = digitalRead(9);
//  IR_5 = digitalRead(7);
//  IR_6 = digitalRead(6);
//  IR_7 = analogRead(5);
//  IR_8 = digitalRead(15);
//  IR_9 = digitalRead(16);
//  IR_10 = analogRead(17);
//  IR_11 = analogRead(18);
//  IR_12 = digitalRead(19);
//
//  
//  Serial.print(IR_1);
//  //Serial.print(" ");
//
////IR_1 = digitalRead(4);
////  Serial.print(IR_1);
////  Serial.println(" ");
//  
// Serial.print(IR_2);
//   Serial.print(" ");
//  Serial.print(IR_3);
//     Serial.print(" ");
// // Serial.print(IR_4);
//  Serial.print(IR_6);
//     Serial.print(" ");
// // Serial.print(IR_5);
//  Serial.print(IR_7);
//     Serial.print(" ");
// // Serial.print(IR_8);
// // Serial.print(IR_9);
//  Serial.print(IR_10);
//     Serial.print(" ");
//  Serial.print(IR_11);
//     Serial.print(" ");
// // Serial.print(IR_12);
//  Serial.println();
//
//
//
//
//
//
//
//
//
//

