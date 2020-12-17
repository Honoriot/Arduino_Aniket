
// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// make custom character
byte arrow[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
};

byte tree[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b01110,
  0b11111,
  0b01110,
  0b11111,
  0b00100,
};

byte arrow_head[8] = {
  0b00000,
  0b10000,
  0b11100,
  0b11111,
  0b11111,
  0b11100,
  0b10000,
  0b00000
};

void setup() {
  lcd.begin(16, 2);
  lcd.createChar(0, arrow);
  lcd.createChar(1, arrow_head);
  lcd.createChar(2, tree);
  
  lcd.setCursor(0, 0);
  lcd.write(byte(0));
  lcd.write(1);
  lcd.write(2);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for(int x=0;x<2;x++){
//    for(int y=0;y<16;y++){
//      lcd.clear();
//      lcd.setCursor(y, x);
//      lcd.write(byte(0));
//      lcd.write(1);
//
//      lcd.setCursor(y+2, x);
//      lcd.write(byte(0));
//      lcd.write(1);
//      delay(500);
//    }
//  }
}
