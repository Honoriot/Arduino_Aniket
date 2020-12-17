#include <EEPROM.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define Flag_data_add 0
#define Time_data_add 1

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 16, 2);

int led_1 = 12;
int button_1 = 11;
int switch_1 = 5;


int led_2 = 10;
int button_2 = 9;
int switch_2 = 6;

int flag;
int Time;

void setup()
{
  Serial.begin(9600);
  // initialize the LCD
  lcd.begin();
  
  pinMode(led_1, OUTPUT);
  pinMode(button_1, INPUT);
  pinMode(switch_1, OUTPUT);
  
  pinMode(led_2, OUTPUT);
  pinMode(button_2, INPUT);
  pinMode(switch_2, OUTPUT);
  
  // Turn on the blacklight and print a message.

  digitalWrite(switch_1, HIGH);
  digitalWrite(switch_2, HIGH);

  if(EEPROM.read(Flag_data_add)==255){
    EEPROM.update(Flag_data_add, 0);
  }

  if(EEPROM.read(Time_data_add)==255){
    EEPROM.update(Time_data_add,5);
  }
  flag = EEPROM.read(Flag_data_add);
  Time = EEPROM.read(Time_data_add);
  Serial.print("Flag: " );Serial.println(flag);
  lcd.backlight();
  lcd.print("Hello, world!");
  delay(500);
}

void loop()
{
  if(digitalRead(button_2)==1){
    digitalWrite(led_2, HIGH);
    setting_mode();
  }
  else{
    digitalWrite(led_2, LOW);
    play_mode(Time);
  }
}

void play_mode(int Time_to_Play){
    if(digitalRead(button_1)==1 and flag==0){
    digitalWrite(led_1, HIGH);
    digitalWrite(switch_1, LOW);
    lcd.clear();
    lcd.print("Motor On: ");
    lcd.print(Time_to_Play);
    flag = 1;
    EEPROM.update(Flag_data_add, flag);
    delay(500);
    for(int y=0;y<Time_to_Play;y=y+1){
      for(int i=0;i<=60;i=i+1){
        lcd.setCursor(0,1);
        lcd.print(y);
        lcd.print(":");
        lcd.print(i);
        delay(1000);
      if(digitalRead(button_1)==0){
            digitalWrite(led_1, LOW);
            digitalWrite(switch_1, HIGH);
            flag = 0;
            break;
            }        
      }
      if(digitalRead(button_1)==0){
            digitalWrite(led_1, LOW);
            digitalWrite(switch_1, HIGH);
            flag = 0;
            EEPROM.update(Flag_data_add, flag);
            delay(500);
            break;
            }
    }
    
    lcd.clear();
  digitalWrite(switch_1, HIGH);     
  }

  if(digitalRead(button_1)==1 and flag==1){
    lcd.setCursor(0,0);
    lcd.print("Plase Off");
    lcd.setCursor(0,1);
    lcd.print(flag);
    if(digitalRead(button_1)==0){
      flag = 0;
      digitalWrite(led_1, LOW);
      digitalWrite(switch_1, HIGH);
      lcd.setCursor(0,0);
      lcd.print("Thankyou!!");
      lcd.setCursor(0,1);
      lcd.print(flag);
    }
  }

  if(digitalRead(button_1)==0 and flag==0){
    digitalWrite(led_1, LOW);
    digitalWrite(switch_1, HIGH);
    flag = 0;
    lcd.setCursor(0,0);
    lcd.print("Motor is OFF");
    lcd.setCursor(0,1);
    lcd.print(flag);
  }

  
  if(digitalRead(button_1)==0 and flag==1){
    flag = 0;
  }  
 
}

void setting_mode(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setting Mode");
  delay(500);
  lcd.setCursor(0,0);
  lcd.print("Present time: ");
  lcd.print(Time);
  delay(500);
  lcd.setCursor(0,0);
  lcd.print("Set Time: Press");
  
  while(1){
    lcd.setCursor(0,1);
    lcd.print(Time);
    delay(500);
    if(digitalRead(button_1)==1 and Time<60){
      Time = Time + 5;
    }
    lcd.setCursor(0,1);
    lcd.print(Time);
    delay(1000);
    if(digitalRead(button_1)==0){
      break;
    }
    if(Time == 60 or Time > 60){
      Time = 0;
    }
  }
  EEPROM.update(Time_data_add,Time);
}
