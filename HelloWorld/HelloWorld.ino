/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

*/

// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int Button = 13;
int flag = 0;
void setup() {
  // set up the LCD's number of columns and rows:
  //Serial.begin(9600);
  lcd.begin(16, 2);
  // Print a message to the LCD.
  // lcd.print("hello, world!");
  pinMode(Button, INPUT);


  
  lcd.home();
  lcd.print("We begin!");
  //delay(1000);
  lcd.setCursor(2, 1);
  lcd.print("Press Button");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  // lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  // lcd.print(millis() / 1000);
  start_();
}

void start_(){
//  lcd.home();
//  lcd.print("We begin!");
//  //delay(1000);
//  lcd.setCursor(2, 1);
//  lcd.print("Press Button");
  //Serial.print(digitalRead(Button));

  if(digitalRead(Button)==1){
    lcd.clear();
    lcd.home();
    lcd.print("Aniket Singh");
    lcd.setCursor(0,1);
    lcd.print("8929163145");
    delay(100);
  }
  
  if(digitalRead(Button)==0){
    lcd.clear();
    lcd.home();
    lcd.print("Aniket Singh");
    for(int i=0;i<16;i++){
      lcd.clear();
      lcd.home();
      lcd.print("Aniket Singh");
      lcd.setCursor(i,1);
      lcd.print("8929163145");
      delay(500);
      if(digitalRead(Button)==1)
        break; 
    }
  }
//  if(digitalRead(Button)==1 and flag == 1){
//    flag = 0;
//  }

//  if(flag == 1){
//    lcd.clear();
//    lcd.home();
//    lcd.print("Aniket Singh");
//  }if(flag == 0){
//    lcd.clear();
//    lcd.home();
//    lcd.print("Info is hidden");
//    lcd.setCursor(2, 1);
//    lcd.print("Press, Button");
//  }
}
