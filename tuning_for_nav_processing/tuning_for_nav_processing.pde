import controlP5.*;
import processing.serial.*;

Serial myPort; 
String pt = "pt=0\n";
String dt = "dt=0\n";
String pr = "pr=0\n";
String dr = "dr=0\n";
String side = "side=0\n";

ControlP5 cp5;
int myColor = color(0,0,0);
int PT_slider = 0;
int DT_slider = 0;
int PR_slider = 0;
int DR_slider = 0;
int Choose_Side = 0;
int Choose_Magnitude = 0;

Slider abc;

byte[] PT_package = new byte[2];
byte[] DT_package = new byte[2];
byte[] PR_package = new byte[2];
byte[] DR_package = new byte[2];
byte Mag_package = 0;
byte Side_package = 0;

// variables for saving values in
int pt_1 = 0;
int dt_1 = 0;
int pr_1 = 0;
int dr_1 = 0;
int side_1;
int mag_1;

int pt_2 = 0;
int dt_2 = 0;
int pr_2 = 0;
int dr_2 = 0;
int side_2 = 0;
int mag_2 = 0;

byte front = 0;
byte arm = 1;

void setup() {
  size(800,400);
  noStroke();
  cp5 = new ControlP5(this);

 myPort = new Serial(this, "/dev/cu.usbmodem1421", 115200);
     
    // add a vertical slider
    cp5.addSlider("PT_slider")
       .setPosition(100,50)
       .setSize(200,20)
       .setRange(0,100)
       .setValue(0)
       .setDecimalPrecision(2)
       ;
       
    cp5.addSlider("DT_slider")
       .setPosition(100,100)
       .setSize(200,20)
       .setRange(0,100)
       .setValue(0)
       .setDecimalPrecision(1)
       ;
  
    cp5.addSlider("PR_slider")
       .setPosition(100,150)
       .setSize(200,20)
       .setRange(0,100)
       .setValue(0)
       .setDecimalPrecision(1)
       ;
       
   cp5.addSlider("DR_slider")
       .setPosition(100,200)
       .setSize(200,20)
       .setRange(0,100)
       .setValue(0)
       .setDecimalPrecision(1)
       ;
  
   cp5.addButton("Clear_All")
       .setValue(0)
       .setPosition(350,125)
       .setSize(45,20)
       ;
  
   cp5.addButton("Save_1")
       .setValue(0)
       .setPosition(100,300)
       .setSize(35,20)
       ;
  
   cp5.addButton("Preset_1")
       .setValue(0)
       .setPosition(150,300)
       .setSize(40,20)
       ;
  
   cp5.addButton("Save_2")
       .setValue(0)
       .setPosition(100,350)
       .setSize(35,20)
       ;
  
   cp5.addButton("Preset_2")
       .setValue(0)
       .setPosition(150,350)
       .setSize(40,20)
       ;
  
   cp5.addSlider("Choose_Side")
       .setPosition(100,15)
       .setSize(200,20)
       .setRange(0,1)
       .setValue(0)
       .setDecimalPrecision(0)
       ;

    cp5.addButton("Front")
       .setValue(0)
       .setPosition(10,15)
       .setSize(30,20)
       ;
       
    cp5.addButton("Arm")
       .setValue(0)
       .setPosition(50,15)
       .setSize(30,20)
       ;    
       
   cp5.addButton("Send")
       .setValue(0)
       .setPosition(350,150)
       .setSize(30,20)
       ;
       
   cp5.addButton("0.01")
       .setValue(0)
       .setPosition(100,250)
       .setSize(30,20)
       ;  
       
   cp5.addButton("0.1")
       .setValue(0)
       .setPosition(150,250)
       .setSize(30,20)
       ;  
  
  cp5.addButton("1")
       .setValue(0)
       .setPosition(200,250)
       .setSize(30,20)
       ;  
  
  cp5.addButton("10")
       .setValue(0)
       .setPosition(250,250)
       .setSize(30,20)
       ; 
  
  cp5.addButton("100")
       .setValue(0)
       .setPosition(300,250)
       .setSize(30,20)
       ; 
  
 cp5.addSlider("Choose_Magnitude")
       .setPosition(350,250)
       .setSize(200,20)
       .setRange(0,5)
       .setValue(0)
       .setDecimalPrecision(1)
       ; 
  
    // reposition the Label for controller 'slider'
    cp5.getController("PT_slider").getValueLabel().align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("PT_slider").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("DT_slider").getValueLabel().align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("DT_slider").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("PR_slider").getValueLabel().align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("PR_slider").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("DR_slider").getValueLabel().align(ControlP5.LEFT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    cp5.getController("DR_slider").getCaptionLabel().align(ControlP5.RIGHT, ControlP5.BOTTOM_OUTSIDE).setPaddingX(0);
    
}

  long timer_update = 0;
  
void draw() {
  
  
  fill(150);
  rect(0,0,width,height);
  
  //cp5.getController("PT_slider").setValue(0);
  
  //add Clear_All button, Save buttons, Preset Buttons, and side buttons
  
  //String pt = "pt=" + PT_slider + "\n";
  //String dt = "dt=" + DT_slider + "\n";
  //String pr = "pr=" + PR_slider + "\n";
  //String dr = "dr=" + DR_slider + "\n";
  //String side = "side=" + Choose_Side + "\n";
  
  if(cp5.getController("Clear_All").isMousePressed() == true ){
    PT_slider = 0;
    DT_slider = 0;
    PR_slider = 0;
    DR_slider = 0;
    cp5.getController("PT_slider").setValue(0);
    cp5.getController("DT_slider").setValue(0);
    cp5.getController("PR_slider").setValue(0);
    cp5.getController("DR_slider").setValue(0);
  }
  
  if(cp5.getController("Save_1").isMousePressed() == true ){
    pt_1 = PT_slider;
    dt_1 = DT_slider;
    pr_1 = PR_slider;
    dr_1 = DR_slider;
    side_1 = Choose_Side;
    mag_1 = Choose_Magnitude;
  }
  
  if(cp5.getController("Preset_1").isMousePressed() == true ){
    PT_slider = pt_1;
    DT_slider = dt_1;
    PR_slider = pr_1;
    DR_slider = dr_1;
    Choose_Side = side_1;
    Choose_Side = mag_1;
    cp5.getController("PT_slider").setValue(pt_1);
    cp5.getController("DT_slider").setValue(dt_1);
    cp5.getController("PR_slider").setValue(pr_1);
    cp5.getController("DR_slider").setValue(dr_1);
    cp5.getController("Choose_Side").setValue(side_1);
    cp5.getController("Choose_Magnitude").setValue(mag_1);
  }
  
  if(cp5.getController("Save_2").isMousePressed() == true ){
    pt_2 = PT_slider;
    dt_2 = DT_slider;
    pr_2 = PR_slider;
    dr_2 = DR_slider;
    side_2 = Choose_Side;
    mag_2 = Choose_Magnitude;
  }
  
  if(cp5.getController("Preset_2").isMousePressed() == true ){
    PT_slider = pt_2;
    DT_slider = dt_2;
    PR_slider = pr_2;
    DR_slider = dr_2;
    Choose_Side = side_2;
    Choose_Side = mag_2;
    cp5.getController("PT_slider").setValue(pt_2);
    cp5.getController("DT_slider").setValue(dt_2);
    cp5.getController("PR_slider").setValue(pr_2);
    cp5.getController("DR_slider").setValue(dr_2);
    cp5.getController("Choose_Side").setValue(side_2);
    cp5.getController("Choose_Magnitude").setValue(mag_2);
  }
  
  // When Choose_Side is 0 it is front, when 1 it is arm
  if(cp5.getController("Front").isMousePressed() == true ){
    Choose_Side = front;
    cp5.getController("Choose_Side").setValue(front);
  }
  
  if(cp5.getController("Arm").isMousePressed() == true ){
    Choose_Side = arm;
    cp5.getController("Choose_Side").setValue(arm);
  }
  
  if(cp5.getController("0.01").isMousePressed() == true ){
   Choose_Magnitude = 1;
   cp5.getController("Choose_Magnitude").setValue(1);
  }
  
  if(cp5.getController("0.1").isMousePressed() == true){
    Choose_Magnitude = 2;
    cp5.getController("Choose_Magnitude").setValue(2);
  }
  
  if(cp5.getController("1").isMousePressed() == true){
    Choose_Magnitude = 3;
    cp5.getController("Choose_Magnitude").setValue(3);
  }
  
  if(cp5.getController("10").isMousePressed() == true){
    Choose_Magnitude = 4;
    cp5.getController("Choose_Magnitude").setValue(4);
  }
  
  if(cp5.getController("100").isMousePressed() == true){
    Choose_Magnitude = 5;
    cp5.getController("Choose_Magnitude").setValue(5);
  }
  
 if(millis() < 5000) return; // wait 5 sec after reset before allowing an update
 if((cp5.getController("Send").isMousePressed() == true) && ((millis()-timer_update) > 2000)){
   // make sure at least 2 seconds have passed before allowing another data transfer
   timer_update = millis();
   
 Side_package = (byte)(Choose_Side); 
   
  PT_package[0] = (byte)(PT_slider>>8);
  PT_package[1] = (byte)(PT_slider & 0xFF);
  
  DT_package[0] = (byte)(DT_slider>>8);
  DT_package[1] = (byte)(DT_slider & 0xFF);
  
  PR_package[0] = (byte)(PR_slider>>8);
  PR_package[1] = (byte)(PR_slider & 0xFF);
  
  DR_package[0] = (byte)(DR_slider>>8);
  DR_package[1] = (byte)(DR_slider & 0xFF);
  
  Mag_package = (byte)(Choose_Magnitude);

  // send to arduino
  byte[] arrayOne = {Side_package, PT_package[0], PT_package[1], DT_package[0], DT_package[1], PR_package[0], PR_package[1], DR_package[0], DR_package[1], Mag_package};
  for(byte x=0; x<10; x++){
    myPort.write(arrayOne[x]);
  }
 }
   
  
}