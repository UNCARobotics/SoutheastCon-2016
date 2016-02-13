
#include <LiquidCrystal.h>

LiquidCrystal lcd(12,11,5,4,3,2);

#define Xdelay 1900

String a;
String b;
String c;
String d;

void setup() {
    lcd.begin(16,2);
    lcd.setCursor(0,0);

    clearLCD();
    backlightOn();

    lcd.print("HELLO, WORLD!");
    delay(Xdelay);
}

void loop()
{
  char databuff[16];
  char dispbuff[16];

  for(int x=5; x>0; x--)
    {
      delay(1000);
      displayOff();
      delay(1000);
      displayOn();
    }

  clearLCD();
  backlightOn();
  lcd.print("SLOW FADE    ");
  fadeOut(100);
  fadeIn(10);

  lcd.print("0123456789abcdef");
  delay(Xdelay);
  lcd.print("ghijklmnopqrstuv");
  delay(Xdelay);
  lcd.print("wxyz +?*&%$#()!=");
  delay(Xdelay);
  lcd.print("                ");
  delay(Xdelay);
  lcd.print("                ");
  delay(Xdelay);  

  a = "0123456789abcdef";
  b = "ghijklmnopqrstuv";
  c = "wxyz +?*&%$#()!=";
  d = "                ";

  selectLineTwo();
  lcd.print(a);
  delay(Xdelay);

  selectLineOne();
  lcd.print(a);
  selectLineTwo();
  lcd.print(b);
  delay(Xdelay);

 
  selectLineOne();
  lcd.print(b);
  selectLineTwo();
  lcd.print(c);
  delay(Xdelay); 

  
  selectLineOne();
  lcd.print(c);
  selectLineTwo();
  lcd.print(d);
  delay(Xdelay); 

  
  selectLineOne();
  lcd.print(d);
  selectLineTwo();
  lcd.print(d);
  delay(Xdelay); 

for (int x=0; x<=5; x++)
  {
   for(int i = 15; i>=0; i--)
    {
      goTo(i);
      if (i%4 == 1)
        lcd.print("- ");
       if (i%4 == 2)
        lcd.print("I ");
       if (i%4 == 1)
        lcd.print("- ");
       if (i%4 == 1)
        lcd.print("I ");
      delay(100);
    }
   for (int i=0; i<=14; i++)
    {
      goTo(i);
      lcd.print(" @");
      delay(100);
    }
  }
 clearLCD();
}

void selectLineOne()
{
   lcd.write(0xFE);
   lcd.write(128);
   delay(10);
}
void selectLineTwo()
{
  lcd.write(0xFE);
  lcd.write(192);
  delay(10);
 }
 void goTo(int position)
 {
  if (position<16)
  {
  lcd.write(0xFE);
  lcd.write((position+128));
  } else if (position<32)
  {
    lcd.write(0xFE);
    lcd.write((position+48+128));
  } else {goTo(0);}
  delay(10);
 }

  void clearLCD()
  {
    lcd.write(0xFE);
  lcd.write(0x01);
  delay(10);
  }
  void backlightOn()
  {
    lcd.write(0x7C);
    lcd.write(157);
    delay(10);
  }
   void backlightOff()
  {
    lcd.write(0x7C);
    lcd.write(128);
    delay(10);
  }
  void backlightValue(int bv)
  {
    int val = bv;
    if (bv < 128) val = map(bv, 0 , 1023, 128, 157);
    if (bv > 157) val = map(bv, 0 , 1023, 128, 157);

    lcd.write(0x7C);
    lcd.write(val);
    delay(10);
  }
   void displayOn()
   {
    lcd.write(0xFE);
    lcd.write(0x0C);
    delay(10);
   }
    void displayOff()
   {
    lcd.write(0xFE);
    lcd.write(0x08);
    delay(10);
   }
   void fadeOut(int fade)
   {for (int x = 157; x>128; x--)
   {
    backlightValue(x);
    delay(fade);
   }
  }
 void fadeIn(int fade)
   {for (int x = 128; x>=157; x++)
   {
    backlightValue(x);
    delay(fade);
   }
  }
    
  

