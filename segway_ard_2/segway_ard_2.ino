

#include "AH_MCP41xxx.h"      เรียกใช้ Library  ของชิบ DAC
#include <SPI.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
MPU6050 read_sensor;

#define DATAOUT  51   //uno MOSI , IC SI
#define SPICLOCK 52   //uno SCK  , IC SCK
#define CS   53   //chipselect pin
#define SHDN 9   //shutdown pin
#define RS   8   //reset pin

#define POTIOMETER_0 0
#define POTIOMETER_1 1

AH_MCP41xxx mcp42010;


#define Ts 0.01;

/*
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 */

/*
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 
 
 */



void setup() {


  pinMode(8,OUTPUT); //reset pin
  pinMode(9,OUTPUT); //shutdown pin
  pinMode(10,INPUT);

  Wire.begin();
  Serial.begin(9600);
 

  read_sensor.initialize();
  read_sensor.setFullScaleGyroRange(0); 
  read_sensor.setDLPFMode(5);
  // read_sensor.setMasterClockSpeed(9); //I2C Master Clock Speed 500kHz


mcp42010.init_MCP42xxx (CS, SHDN, RS);


  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65536-16000000/256*Ts;            // preload timer 65536-16MHz/(256/2Hz)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts


  
}









float x[2][1]={
  {
    0  }
  ,{
    0.0  }
};
float P[2][2]={
  {
    0  }
  ,{
    0  }
};
float K[2][1];

float ax,az;
float gx;
float theta;
float zk;
float uk;
float gyro;
float dt=Ts;
float Q_angle=0.001;
float Q_bias=0.003; 
float R_measure=0.03;
float theta_d,gyro_offset=-0.02;


void KalmanSensorFusion(){


  gyro=(float)gx*PI/(131*180);

  /* Predict Project the state ahead */

  uk=gyro;
  x[0][0]=x[0][0]+dt*uk-dt*x[1][0];
  x[1][0]=x[1][0];

  /* P=A*P*P'+Q */


  P[0][0]=P[0][0]+dt*(dt*P[1][1]-P[0][1]-P[1][0]+Q_angle);
  P[0][1]=P[0][1]-dt*P[1][1];
  P[1][0]=P[1][0]-dt*P[1][1];
  P[1][1]=P[1][1]+Q_bias*dt;

  /* measure*/

  theta=(atan2(ax,az)); /* Acclerometer*/
  zk=theta;

  K[0][0]=P[0][0]/(P[0][0]+R_measure);
  K[1][0]=P[1][0]/(P[0][0]+R_measure);

  x[0][0]+=K[0][0]*(zk-x[0][0]);
  x[1][0]+=K[1][0]*(zk-x[0][0]);

  P[0][0]=(1-K[0][0])*P[0][0];
  P[0][1]=(1-K[0][0])*P[0][1];
  P[1][0]=-K[1][0]*P[0][0]+P[1][0];
  P[1][1]=-K[1][0]*P[0][1]+P[1][1];


}





float uf1,uf2;
long in1,in2;
float k1,k2;
float err,err_dot,offset=0.0; 
float FR,LR;

void PID()
{

  err=x[0][0]-theta_d-offset-0.05;
  err_dot=gyro-gyro_offset;

  if(digitalRead(10)==0 and abs(err<0.16)){
    
    uf1=deadzone(255*(k1*err+k2*err_dot)+FR,-2,2);
    uf2=deadzone(255*(k1*err+k2*err_dot)-FR,-2,2);
    

    
  }
  else
  {
    uf1=0;
    uf2=0;
    

  }



} 

void SentPID2Drive()
{

  in1=(signed int)uf1;
  mcp42010.setValue(map(in1,-255,255,0,256), POTIOMETER_0);  
  
  //////////////////////////////////////////////// 
  in2=(signed int)uf2;
  mcp42010.setValue(map(in2,-255,255,0,256), POTIOMETER_1);

  



}





ISR(TIMER1_OVF_vect) 
{  
  TCNT1 = 65536-16000000/256*Ts;

  KalmanSensorFusion();
  PID();
  SentPID2Drive();

}

void loop() {

  ax=(float)read_sensor.getAccelerationX()/8192;
  az=(float)read_sensor.getAccelerationZ()/8192;
  gx=(float)read_sensor.getRotationX();


  

    k1=(float)analogRead(0)*5/1024;
    k2=(float)analogRead(1)*.1/1024;
    
    FR=(float)(analogRead(2)-512)*20/1024;
    LR=(float)(analogRead(3)-512)*20/1024;



  Serial.print(theta);
  Serial.print("\t");
  Serial.print(LR);
  Serial.print("\t");
  Serial.print(gyro);
  Serial.print("\t");
  Serial.print(uf1);
  Serial.print("\t");
  Serial.print(uf2);
  
  Serial.println();

}


float deadzone(float x,float a,float b){
  float y;
  
    if(a<=x<b){
       y=0;
    }
    if(x<a){
      y=x-a;
    }
    if(x>=b)
    {
      y=x-b;
    }
    return y;
}
