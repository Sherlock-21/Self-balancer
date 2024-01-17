#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define enA 6  
#define in1 A2
#define in2 A3

unsigned long timer = 0;
double y,z,tilt,yaw,Tilt_error,Yaw_error;
double speedt,speedy;
double error_integralt = 0.0 ;
double previous_errort = 0.0;
double error_derivativet = 0.0;
double error_integraly = 0.0 ;
double previous_errory = 0.0;
double error_derivativey = 0.0;

double kpt = 100.0;
double kit = 0;
double kdt = 0;
double setpoint_tilt = 0.0;

double kpy = 0;
double kiy = 0.0;
double kdy = 0;
double setpoint_yaw = 0.0;

void dc_motor_init(){
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
}

void motor_control(int pwm) {
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


int mapToRange(int value, int minValue, int maxValue) {
  return constrain(value, minValue, maxValue);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  dc_motor_init();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
 
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){
  y=mpu.getAngleY();
  z=mpu.getAngleZ();
  timer = millis();  
  }
  //error calculation
  Tilt_error = setpoint_tilt - y;
  Yaw_error = setpoint_yaw - z;

  //pid
  error_integralt += Tilt_error;
  error_derivativet = Tilt_error - previous_errort;
  speedt = - kpt * Tilt_error - kit * error_integralt - kdt * error_derivativet ; 
  if (speedt > 255) {
    error_integralt = 255 / kit;
  } else if (speedt < -255) {
    error_integralt = -255 / kit;
  }

  error_integraly += Yaw_error;
  error_derivativey = Yaw_error - previous_errory;
  speedy = - kpy * Yaw_error - kiy * error_integraly - kdy * error_derivativey ; 
  //if (speedy > 255) {
    //error_integraly = 255 / kiy;
  //} else if (speedy < -255) {
    //error_integraly = -255 / kiy;
  //}

  float speed_f = speedt - speedy;

  int speed = mapToRange(speed_f, -255, 255);
  motor_control(speed);
  Serial.print("\n ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" ");
  Serial.print(speedy);
  Serial.print(" ");
  Serial.print(speedt);
  previous_errort = Tilt_error;
  previous_errory = Yaw_error;
}


