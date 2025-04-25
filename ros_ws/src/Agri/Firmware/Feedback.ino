#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#define encoder_1_A 22
#define encoder_1_B 23
#define motorIn_1_1  25
#define motorIn_1_2  26

#define encoder_2_A 16
#define encoder_2_B 17
#define motorIn_2_1  18
#define motorIn_2_2  19

#define encoder_3_A 32
#define encoder_3_B 33
#define motorIn_3_1  34
#define motorIn_3_2  35

#define encoder_4_A 12
#define encoder_4_B 13
#define motorIn_4_1  14
#define motorIn_4_2  15

Adafruit_MPU6050 mpu;

int lastAState1 = 0;
volatile long motorPosition1=0;

int lastAState2 = 0;
volatile long motorPosition2=0;

int lastAState3 = 0;
volatile long motorPosition3=0;

int lastAState4 = 0;
volatile long motorPosition4=0;

void updateMotorPosition1(){

  int currentAState1 = digitalRead(encoder_1_A);

  if (currentAState1 != lastAState1) {

  if (digitalRead(encoder_1_B) != digitalRead(encoder_1_A)) {
    motorPosition1++;
    }
  else  {
    motorPosition1--;
      }
      lastAState1 = currentAState1;
    }
  }

void updateMotorPosition2(){

  int currentAState2 = digitalRead(encoder_2_A);

  if (currentAState2 != lastAState2) {

  if (digitalRead(encoder_2_B) != digitalRead(encoder_2_A)) {
    motorPosition2++;
    }
  else {
    motorPosition2--;
    }
    lastAState2 = currentAState2;
  }
}

void updateMotorPosition3(){

  int currentAState3 = digitalRead(encoder_3_A);

  if (currentAState3 != lastAState3) {

  if (digitalRead(encoder_3_B) != digitalRead(encoder_3_A)) {
    motorPosition3++;
  }
  else {
    motorPosition3--;
      }
    lastAState3 = currentAState3;
  }
}
void updateMotorPosition4(){

  int currentAState4 = digitalRead(encoder_4_A);

  if (currentAState4 != lastAState4) {

  if (digitalRead(encoder_4_B) != digitalRead(encoder_4_A)) {
    motorPosition4++;
    }
  else  {
      motorPosition4--;
    }
    lastAState4 = currentAState4;
  }
}



void setup() {

  Serial.begin(9600);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 ready!");
  delay(2000);

  // Encoder 1
  pinMode(encoder_1_A, INPUT);
  pinMode(encoder_1_B, INPUT);
  lastAState1 = digitalRead(encoder_1_A);
  attachInterrupt(digitalPinToInterrupt(encoder_1_A),updateMotorPosition1,CHANGE);
  pinMode(motorIn_1_1, OUTPUT);
  pinMode(motorIn_1_2, OUTPUT);
  digitalWrite(motorIn_1_1, HIGH);
  digitalWrite(motorIn_1_2, LOW);

  // Encoder 2
  pinMode(encoder_2_A, INPUT);
  pinMode(encoder_2_B, INPUT);
  lastAState2 = digitalRead(encoder_2_A);
  attachInterrupt(digitalPinToInterrupt(encoder_2_A),updateMotorPosition2,CHANGE);
  pinMode(motorIn_2_1, OUTPUT);
  pinMode(motorIn_2_2, OUTPUT);
  digitalWrite(motorIn_2_1, HIGH);
  digitalWrite(motorIn_2_2, LOW);

  // Encoder 3
  pinMode(encoder_3_A, INPUT);
  pinMode(encoder_3_B, INPUT);
  lastAState2 = digitalRead(encoder_3_A);
  attachInterrupt(digitalPinToInterrupt(encoder_3_A),updateMotorPosition3,CHANGE);
  pinMode(motorIn_3_1, OUTPUT);
  pinMode(motorIn_3_2, OUTPUT);
  digitalWrite(motorIn_3_1, HIGH);
  digitalWrite(motorIn_3_2, LOW);

  // Encoder 4
  pinMode(encoder_4_A, INPUT);
  pinMode(encoder_4_B, INPUT);
  lastAState2 = digitalRead(encoder_4_A);
  attachInterrupt(digitalPinToInterrupt(encoder_4_A),updateMotorPosition4,CHANGE);
  pinMode(motorIn_4_1, OUTPUT);
  pinMode(motorIn_4_2, OUTPUT);
  digitalWrite(motorIn_4_1, HIGH);
  digitalWrite(motorIn_4_2, LOW);
  
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.x);

  Serial.print(",");

  Serial.print(a.acceleration.y);

  Serial.print(",");

  Serial.print(a.acceleration.z);

  Serial.print(",");

  Serial.print(g.gyro.x);

  Serial.print(",");

  Serial.print(g.gyro.y);

  Serial.print(",");

  Serial.print(g.gyro.z);

  Serial.print(",");
  
  Serial.println(String(motorPosition1)+","+String(motorPosition2)+","+String(motorPosition3)+","+String(motorPosition4));

  
}
