
int Laser_beam = 13; 
int motor_L = 23;
int motor_R = 22;


void setup() {
  // put your setup code here, to run once:
  pinMode(Laser_beam, OUTPUT);
  pinMode(motor_L, OUTPUT);
  pinMode(motor_R, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(Laser_beam, HIGH);
//  delay(1000);
//  digitalWrite(Laser_beam, LOW);
//  delay(1000);
   motorL();
   motorR();
}


void motorL(){
  digitalWrite(motor_L, HIGH);
  digitalWrite(motor_R, LOW);
  delay(5000);
  }

void motorR(){
  digitalWrite(motor_R, HIGH);
  digitalWrite(motor_L, LOW);
  delay(5000);
  }  
