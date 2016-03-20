void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void Mirror_Check() { // tells the robot which version of the course it is on
  Arm.sensePings();
  Leg.sensePings();
  if( Arm.Ping1 > Leg.Ping1) { MIRROR = 1;} // bot is on ver 1/A
  else MIRROR = 0;  // bot is on ver 2/B
}
