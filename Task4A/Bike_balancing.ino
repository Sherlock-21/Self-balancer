#include <PID_v1_bc.h>      //initializing libraries
#include "Wire.h"
#include <MPU6050_light.h>
#include <Encoder.h>

#define enA 6  
#define in1 A2
#define in2 A3                //initializing variables
MPU6050 mpu(Wire);

#define enA 6  
#define in1 A2
#define in2 A3

unsigned long timer = 0;
double y,z,tilt,yaw;
double setpoint_tilt = 0.0;
double setpoint_yaw = 0.0;
int Omniwheel;
int wheel1a;
int wheel2;
int wheel2a;

double Pk1 = 0.5; 
double Ik1 = 0;
double Dk1 = 0;
//Initializing PID
double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position

double Pk2 = 500;    
double Ik2 = 250;   
double Dk2 = 200;   

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT); // PID Balancing

Encoder DC_Encoder(2, 3);

long old_ticks = -999;

int encoder(){                        //Function to read encoder data
  long ticks = DC_Encoder.read();
  if (ticks != old_ticks) {
    old_ticks = ticks;
  }
  Serial.println(ticks);
  return ticks;
}

void dc_motor_init(){               
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
}

void motor_control(int pwm) {       //Function to control omniwheel motor
  if (pwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwm = -pwm;
  } 
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}



void setup() {
  Serial.begin(9600);
  Wire.begin();
  dc_motor_init();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));      //calibirating the imu
  delay(1000);
 
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  PID1.SetMode(AUTOMATIC);   //PID settings            
  PID1.SetOutputLimits(-200, 200);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-1000,1000);
  PID2.SetSampleTime(10);
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){
  y=mpu.getAngleY();     //Getting Y axis rotation (Tilt_angle in our case)
 
  timer = millis();  
  }
  z=encoder();           //Getting motor position through encoder data
  
  Setpoint2 = 0;
  Input2 = -y;
  PID2.Compute();   // Calculating Output for balancing

  Setpoint1= 0;         
  Setpoint1 = Setpoint1 + Output2;     //output of  balancing pid is added as setpoint for position pid, to control the position while maintaing balance over time
  Input1 = -z;
  PID1.Compute();           //Calculating Output for Yaw and tilt control.

  Omniwheel = Output1;     //Saving output as omniwheel
  

  float speed_f = Omniwheel ;      //speed for the omniwheel

  int speed = mapToRange(speed_f, -255, 255);        //Mapping the output to motor speed range
  motor_control(speed);
  Serial.print("\n ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" ");
  Serial.print(Output1);
  Serial.print(" ");
  Serial.print(Output2);                         //printing for degbugging
  Serial.print(" ");
 
}


