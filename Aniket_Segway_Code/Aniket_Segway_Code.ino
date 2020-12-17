/*
 Nano with MPU6050
 SCL = A5
 SDA = A4
 PWM_PINS = 3, 5, 6, 9, 11
*/

#include "I2Cdev.h"
#include "Wire.h"
#include <MPU6050.h>
#include <math.h>
#include <SoftwareSerial.h>

MPU6050 accelgyro;

int16_t ax, ay, az;   
int16_t gx, gy, gz;

#define Pin_Steering_Command A0;
// #define Pin_steering_right  ;    // Pin connection for the steering command right
// #define Pin_steering_left   ;    // Pin connection for the left steering command

// #define Pin_P ;
// #define Pin_I ;
// #define Pin_D ;

int LoopTime_Should = 9;                       // Desired loop duration in ms to get to 100 Hz                 
int LoopTime_Customized = LoopTime_Should;     // last loop time with forced pause        
int LoopTime_So_Far = LoopTime_Should;         // last loop time without forced pause   
unsigned long LoopTime_Start = 0;              // Start time of the loop


float Angle;                    // current angle of inclination                                         
float Angle_Should;             // Target value of the angle of inclination, i.e. 0 °
float Angle_Border;             // Maximum permissible angle of inclination over which the Segway is switched off

float ACC_angle;         // Angle from the accelerometer
float GYRO_rate;         // Angular speed from the gyro sensor


float Kp,Ki,Kd,K;                                //  Differentteil, Integralteil, Differentialteil, K : total share
int Motor;                                       //  value received from the PID control for motor control
int Motor_Right, Motor_Left;                     // Values ​​for the two engines
float K_Motor_Left, K_Motor_Right;               // Correction factors for synchronous running of the two motors


int Steering_Entrance_Right = 0;                  // Variable for detecting a steering command to the right
int Steering_Entrance_Left = 0;                   // Variable for detecting a steering command to the left
float Steering_Max;                               // Value by which the motor control should change at a maximum when a steering command is given
float Steering_Right, Steering_Left;              // Current and gradually increased control value when steering to the right or left


// Motor driver pin connection
int Motor_Left_P = 4;
int Motor_Left_N = 5;
int Motor_Left_PWM = 3;

int Motor_Right_P = 6;
int Motor_Right_N = 7;
int Motor_Right_PWM = 9;

// ****************************************************************************
// ****************************** SETUP ***************************************
// ****************************************************************************  

void setup() {
  Wire.begin(); 

  Serial.begin(9600); 

  // initialize device
  accelgyro.initialize();

  calibrateSensors();    // Subprogram for one-time calibration of the sensors
}

// ***********************************************************************************
// ****************************** Calibration ***************************************
// ***********************************************************************************

void calibrateSensors()
{

  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);


  Angle_Should = 0.0;                 // Setpoint for the angle of inclination
  Angle_Border = 30.0;                // maximum permitted angle of inclination


  Kp = 10 // analogRead(Pin_PID_Regelung_P) * 25.0 / 1023.0;               0 - 25
  Ki = 0.1;                                                      
  Kd = 40 // analogRead(Pin_PID_Regelung_D) * 100.0 / 1023.0;       // Differentialanteil mit Poti festgelegt      0 - 100
  K = 1.0;                                                    // Total share


   K_Motor_Right = 1.0;                  // Correction factor for the right motor
   K_Motor_Left = 0.8;                   // Correction factor for the left motor

  
    
    Steering_Max = 25.0;                      // Value by which the motor control should change MAXIMUM when a steering command is given
    Steering_Right = 0.0;                    // current additional value when steering to the right
    Steering_Left = 0.0;                     // current additional value when steering to the left

    pinMode(Pin_Steering_Command, INPUT);
    // pinMode(Pin_Steering_right, INPUT);      // The pin for steering to the right is declared as an input
    // pinMode(Pin_Steering_left, INPUT);       // The pin for steering to the left is declared as an input
}


// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************
// ********************************************************************** MAIN LOOP **************************************************************
// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************


void loop() {

   // *******************************************************
   // ********************* Sensor Query *******************
   // *******************************************************

   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
    ACC_angle = atan(ay * 1.0 / az * 1.0) * 180.0 / 3.141592654;       // Resolution 2g: 16384 / g
   
    //ACC_angle = atan((ay/16384.0) / (az/16384.0)) * 180.0 / 3.141592654;       // Resolution 2g: 16384 / g
    
    GYRO_rate = gx/65.5;                                 // Resolution 500 ° / s: 65.5 / ° / s


   // *******************************************************
   // ********** K - values ​​for PID control *************
   // *******************************************************


   // ********************************************************************************
   // ****************** Kalman filter, PWM calculation and motor values *****************
   // ********************************************************************************
     
     
   Angle = kalmanCalculate(ACC_angle, GYRO_rate, LoopTime_customized); // Angle calculated with Kalman filter

  if (Angle > Angle_Border || Angle < (Angle_Border * (-1)))       
        {
         // ===============================================
         // Cancellation due to the too large angle of inclination!
         // ===============================================
         
          //         ST.motor(1, 0);
          //         ST.motor(2, 0);

          analogWrite(Motor_Left_PWM, 0);
          analogWrite(Motor_Right_PWM, 0);
        }
    else
        {
         // =========================
         // Tilt angle ok
         // =========================
      
 
         Motor = pid(Angle, Angle_Should, GYRO_rate);      // Calculation of the PWM value for controlling the motors   soll = should
     
         Motor_Right = K_Motor_Right * Motor;            // Calculation of the motor speed synchronized by means of the K factor for the right motor

         Motor_Left = K_Motor_Left * Motor;              // Calculation of the motor speed for the left motor synchronized using the K factor
     
         
          
         // **************************************************************************************
         // ***** Query whether the steering has been operated and change in the motor control *****
         // **************************************************************************************
     
     
         Steering_Entrance_Right = digitalRead(Pin_Lenkung_rechts);   // Query the pin for steering to the right

         if (Steering_Entrance_Right == HIGH)
            {     
              // ******************************************
              // *** Right steering was pressed ***
              // ******************************************
          
              if (Motor_Right >= 0)    // segway is driving straight forward or is stationary. It doesn't matter which engine is queried.
                 {
                  Motor_Right = Motor_Right - (int)Steering_Right;   // Maybe try multiplying a factor (e.g. * (1 - 0.1)
                  Motor_Left = Motor_Left + (int)Steering_Right;     // Maybe try multiplying a factor (e.g. * (1 + 0.1))
                 }
              else                      // segway is currently reversing
                 {
                  Motor_Right = Motor_Right + (int)Steering_Right;   // Maybe try multiplying a factor (e.g. * (1 + 0.1))
                  Motor_Left = Motor_Left - (int)Steering_Right;     // Maybe try multiplying a factor (e.g. * (1 - 0.1)
                 }
                 
             Steering_Right = Steering_Right + 0.05;                                 // Better just to e.g. Increase 0.1 per query so that the steering is not too abrupt!
             
             if (Steering_Right > Steering_Max) Steering_Right = Steering_Max;        // The maximum steering value must not be exceeded!
             
             //Lenkung_rechts = constrain(Lenkung_rechts, 0, Lenkung_max);          // Right steering value brought into the interval [0, Steering_max]
            } 
         else
            {
             Steering_Right = 0.0;
            }
    
    
         Steering_Entrance_Left = digitalRead(Pin_Steering_Left);    // Query the pin for steering to the left

         if (Steering_Entrance_Left == HIGH)
            {     
              // *****************************************
              // *** Left steering was pressed ***
              // *****************************************
          
              if (Motor_Left >= 0)    // segway is driving straight forward or is stationary. Which engine is queried does not matter.
                 {
                  Motor_Right = Motor_Right + (int)Steering_Left;   // Maybe try multiplying a factor (e.g. * (1 + 0.1))
                  Motor_Left = Motor_Left - (int)Steering_Left;     //Maybe try multiplying a factor (e.g. * (1 - 0.1)
                 }
              else                      // segway is currently reversing
                 {
                  Motor_Right = Motor_Right - (int)Steering_Left;   // Maybe try multiplying a factor (e.g. * (1 - 0.1)
                  Motor_Left = Motor_Left + (int)Steering_Left;     // Maybe try multiplying a factor (e.g. * (1 + 0.1))
                 }
                 
             Steering_Left = Steering_Left + 0.05;                                 // Better just to e.g. Increase 0.1 per query so that the steering is not too abrupt!
             
             if (Steering_Left > Steering_Max) Steering_Left = Steering_Max;        // The maximum steering value must not be exceeded!
             
             //Lenkung_links = constrain(Lenkung_links, 0, Lenkung_max);          // linker Lenkungswert ins Intervall [0,Lenkung_max] gebracht
            } 
         else
            {
             Steering_Left = 0.0;
            }
       
        
        
     
         // *******************************************************************************************
         // ******************************** Controlling the motors  ***********************************
         // *******************************************************************************************
        
         
         Motor_Right = constrain(Motor_Right, -255, 255);          // right motor value brought into the interval [-127,127]
         Motor_Left = constrain(Motor_Left, -255, 255);            // left motor value brought into the interval [-127.127]
         
                      
     /*
         // Use of a square root function instead of the linear control function to improve the response behavior at low motor values
         // ======================================================================================================================================
         
         if (Motor_rechts >= 0)     // right motor turns forward
            { 
             Motor_rechts = sqrt(127 * Motor_rechts);             // to improve the response behavior at low engine values
              
             ST.motor(2, Motor_rechts);      
            }
         else                       // right motor turns backwards
            {
             Motor_rechts = -sqrt(127 * -Motor_rechts);           // to improve the response behavior at low engine values
             
             ST.motor(2, Motor_rechts);               
            }


         if (Motor_links >= 0)      // left motor turns forward
            {
             Motor_links = sqrt(127 * Motor_links);               // to improve the response behavior at low engine values
             
             ST.motor(1, Motor_links);               
            }
         else                       // left motor rotates backwards
            {
             Motor_links = -sqrt(127 * -Motor_links);             // to improve the response behavior at low engine values
             
             ST.motor(1, Motor_links);  
            }
         */

         if(Motor_Right < 0 || Motor_Left < 0){
            digitalWrite(Motor_Left_P, LOW);
            digitalWrite(Motor_Left_N, HIGH);
            analogWrite(Motor_Left_PWM, -1*Motor_Left);
            
            digitalWrite(Motor_Left_P, LOW);
            digitalWrite(Motor_Left_N, HIGH);
            analogWrite(Motor_Right_PWM, -1*Motor_Right);
          }else{
              digitalWrite(Motor_Left_P, HIGH);
            digitalWrite(Motor_Left_N, LOW);
            analogWrite(Motor_Left_PWM, Motor_Left);
            
            digitalWrite(Motor_Left_P, HIGH);
            digitalWrite(Motor_Left_N, LOW);
            analogWrite(Motor_Right_PWM, Motor_Right);
            }
         
         // ST.motor(1, Motor_Left);
         // ST.motor(2, Motor_Right);
         
        }
        

    // ************************************************************************ 
    // *********************** Output of the measured values **************************
    // ************************************************************************

    Value_Output();       // Value output


    // ******************************************************************
    // *********************** Keyboard query **************************
    // ******************************************************************

    // Keyboard_input();


   // **********************************************************************
   // *********************** loop timing control **************************
   // **********************************************************************


    LoopTime_So_Far = millis() - LoopTime_Start;        // Time since the last loop
     
     if(LoopTime_So_Far < LoopTime_Soll)
        {
         delay(LoopTime_Should - LoopTime_So_Far);         // Delay to get the same loop time
        }
     
     LoopTime_Customized = millis() - LoopTime_Start;     // updated duration of the last loop, should be equal to LoopTime_Soll = e.g. Be 10 msec!
     LoopTime_Start = millis();                          // new start time of the loop

}




// ********************************************************************************************
// ****************** Value output to the serial interface ******************************
// ********************************************************************************************

void Value_Output()
 {
    /*
    Serial.print(Winkel);
    Serial.print("     ");
    Serial.println(Motor);
    Serial.print("     ");
    */
    
    Serial.print("a_y = ");
    Serial.print(ay/16384.0);
    Serial.print("    a_z = ");
    Serial.print(az/16384.0);
    Serial.print("    ACC_angle = ");
    Serial.print(ACC_angle,0);
    Serial.print("    GYRO_rate = ");
    Serial.print(GYRO_rate,0);
    Serial.print("    Angle: ");
    Serial.println(Angle,0);
    
    
    Serial.print("   Motor: ");
    Serial.print(Motor);
    Serial.print("    Motor_Right: ");
    Serial.print(Motor_Right);
    Serial.print("    Motor_Left: ");
    Serial.println(Motor_Left);
}




// ******************************************************************************************************
// ***************************************** PID control **********************************************
// ******************************************************************************************************

float error;
float last_error = 0;
float pTerm;
float iTerm;
float dTerm;
float integrated_error = 0;
int GUARD_GAIN = 40;           // maximum integrated angle error

int pid(float Angle_Current, float Angle_Specification, float Angular_velocity)
{
   error = Angle_Specification - Angle_Current;
   
   pTerm = Kp * error;                                                         // Differenzenanteil
   
   
   integrated_error = integrated_error + error;
  
   iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);          // Integralanteil
  
   
   dTerm = Kd * Angular_velocity / 100.0;                                 // Differential component; : 100 to get usable values!
   
   /*
   Serial.print("    K_p: ");
   Serial.print(pTerm);
   Serial.print("    K_d: ");
   Serial.println(dTerm);
   */
  
   last_error = error;
  
   // Serial.println(K*(pTerm + iTerm + dTerm));
   
  
   return constrain(K*(pTerm + iTerm + dTerm), -127, 127);                     // Output of the motor value for the two motors within the limits [-127,127]
} 

// ******************************************************************************************************
// ************************************** Kalman filter module ******************************************
// ******************************************************************************************************


float Q_angle  =  0.001;    // E(alpha2) = 0.001
float Q_gyro   =  0.003;      // E(bias2) = 0.003
float R_angle  =  0.001;      // Sz = 0.03 !!! the larger the number, the less sensitive the angle reacts to changes !!!
float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float dt, y, S;
float K_0, K_1;

float kalmanCalculate(float newAngle, float newRate, int looptime)
{
  dt = float(looptime)/1000;    // dt in Sekunden
  x_angle = x_angle + dt * (newRate - x_bias);
  P_00 = P_00 - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 = P_01 - dt * P_11;
  P_10 = P_10 - dt * P_11;
  P_11 = P_11 + Q_gyro * dt;
  
  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;
  
  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return x_angle;
}      
