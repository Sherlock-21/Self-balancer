#include <PID_v1_bc.h>      //initializing libraries
#include "Wire.h"
#include <MPU6050_light.h>
#include <Encoder.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define enA 6  
#define in1 A2
#define in2 A3                //initializing pins
MPU6050 mpu(Wire);

#define enA 6  
#define in1 A2
#define in2 A3
#define enB 5
#define in3 9
#define in4 4

unsigned long timer = 0;
double y,z,tilt,yaw;                 //initializing variables
double setpoint_tilt = 0.0;
double setpoint_yaw = 0.0;
int Omniwheel;
int wheel1a;
int wheel2;
int wheel2a;
int speedb=0;
int speedbo=0;
double direct=0;
double Pk1 = 0.9;                   //PID constants
double Ik1 = 50;
double Dk1 = 0.5;
//Initializing PID
double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position

double Pk2 = 9;    
double Ik2 = 180;
double Dk2 = 2.4;   

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT); // PID Balancing

RF24 radio(10, 8); // CE, CSN
const byte address[6] = "00001";
char receivedData[32] = "";
int  xAxis, yAxis;

Encoder DC_Encoder(2, 3);
long old_ticks = -999;
const int ticks_per_revolution = 700;  // ticks per revolution of the encoder

float encoder() {
  long ticks = DC_Encoder.read();

  if (ticks != old_ticks) {
    old_ticks = ticks;
    // Calculate angle in degrees based on ticks and ticks per revolution
    float angle = (static_cast<float>(ticks) / ticks_per_revolution) * 360.0;
    Serial.println(angle);
    return angle;
  }

  return 0.0;  // No change in ticks, return 0 as the default value
}


void dc_motor_init(){               
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);                     //initializing motor driver
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
}
void bo_motor_init(){
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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

void bo_motor(int speedb) {       //Function to control bo motor
  if (speedb < 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    speedb = -speedb;
  } 
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB, speedb);
}

int mapToRange(int value, int minValue, int maxValue) {                 // Mapping the pid output
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
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));      //calibirating the imu
  delay(700);
 
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  PID1.SetMode(AUTOMATIC);   //PID settings            
  PID1.SetOutputLimits(-200, 200);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-20000,20000);
  PID2.SetSampleTime(10);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {

  if (radio.available()) {   // If the NRF240L01 module received data
    radio.read(&receivedData, sizeof(receivedData)); // Read the data and put it into character array
    xAxis = atoi(&receivedData[0]); // Convert the data from the character array (received X value) into integer
    if (xAxis > 950) {
      direct+=0.1;

    }
    else if(xAxis<50) {
      direct-=0.1;
    }
    radio.read(&receivedData, sizeof(receivedData));
    yAxis = atoi(&receivedData[0]);
    speedbo=yAxis-510;
    speedb=mapToRange(speedbo,-255,255);
    bo_motor(speedb);
  }

  mpu.update();
  
  if((millis()-timer)>10){
  y=mpu.getAngleY();     //Getting Y axis rotation (Tilt_angle in our case)
  z=mpu.getAngleZ();
  timer = millis();  
  }
  //z=encoder();           //Getting motor position through encoder data
  
  Setpoint2 = 0;
  Input2 = -y;
  PID2.Compute();   // Calculating Output for balancing
  //Cascade PID
  Setpoint1= direct;         
  Setpoint1 = Setpoint1+Output2 ;     //output of  balancing pid is added as setpoint for position pid, to control the position while maintaing balance over time
  Input1 = -z;
  PID1.Compute();           //Calculating Output for Yaw and tilt control.

  Omniwheel = +Output1;     //Saving output as omniwheel
  

  float speed_f = Omniwheel ;      //speed for the omniwheel

  int speed = mapToRange(speed_f, -255, 255);        //Mapping the output to motor speed range
  motor_control(speed);

  
 
  Serial.print("\n ");
  Serial.print(speedb);
  Serial.print(" ");
  Serial.print(-z);
  Serial.print(" ");
  //Serial.print(Output1);
  Serial.print(" ");
 // Serial.print(Output2);                         //printing for degbugging
  Serial.print(" ");
 
}


