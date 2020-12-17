
int led_1 = 12;
int led_2 = 10;

int button_1 = 11;
int button_2 = 9;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(button_1, INPUT);
  pinMode(button_2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led_1, HIGH);
  digitalWrite(led_2, HIGH);
  Serial.print("Button 1: "); Serial.println(digitalRead(button_1));
  Serial.print("Button 2: "); Serial.println(digitalRead(button_2));
  delay(1000);
}
