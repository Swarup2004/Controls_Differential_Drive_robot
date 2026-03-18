#include <math.h>

// ---------------- PIN MAP ----------------
#define R_EN 6
#define R_IN1 7
#define R_IN2 8

#define L_EN 11
#define L_IN1 12
#define L_IN2 13

#define L_ENC_A 14
#define L_ENC_B 15
#define R_ENC_A 16
#define R_ENC_B 17

// ---------------- ROBOT PARAMETERS ----------------
const float WHEEL_DIAMETER = 0.069;
const float CPR = 208.0;

const float DT = 0.02;

const float TARGET_DISTANCE = 2.0;   // meters
const float TARGET_RPM = 140.0;      // adjust for speed

// ---------------- CONTROLLERS ----------------
float Kp_speed = 2.0;
float Ki_speed = 5.0;

float K_straight = 0.02;  // straight correction gain

float intL = 0;
float intR = 0;

// ---------------- ENCODERS ----------------
volatile long leftCount  = 0;
volatile long rightCount = 0;

void leftISR(){
  if(digitalRead(L_ENC_B)) leftCount++;
  else leftCount--;
}

void rightISR(){
  if(digitalRead(R_ENC_B)) rightCount++;
  else rightCount--;
}

void setMotorPWM(int pwmL, int pwmR){
  pwmL = constrain(pwmL,0,255);
  pwmR = constrain(pwmR,0,255);

  analogWrite(L_EN,pwmL);
  analogWrite(R_EN,pwmR);
}

void setup(){

  Serial.begin(115200);

  pinMode(R_EN,OUTPUT);
  pinMode(R_IN1,OUTPUT);
  pinMode(R_IN2,OUTPUT);

  pinMode(L_EN,OUTPUT);
  pinMode(L_IN1,OUTPUT);
  pinMode(L_IN2,OUTPUT);

  pinMode(L_ENC_A,INPUT_PULLUP);
  pinMode(L_ENC_B,INPUT_PULLUP);
  pinMode(R_ENC_A,INPUT_PULLUP);
  pinMode(R_ENC_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

  digitalWrite(R_IN1,HIGH);
  digitalWrite(R_IN2,LOW);

  digitalWrite(L_IN1,HIGH);
  digitalWrite(L_IN2,LOW);

  setMotorPWM(0,0);
}

void loop(){

  static unsigned long last=0;
  if(millis()-last < DT*1000) return;
  last=millis();

  // ---------- encoder delta ----------
  static long prevL=0, prevR=0;

  long curL = leftCount;
  long curR = rightCount;

  long dL = curL - prevL;
  long dR = curR - prevR;

  prevL = curL;
  prevR = curR;

  // ---------- RPM calculation ----------
  float rpmL = (dL * 60.0) / (CPR * DT);
  float rpmR = (dR * 60.0) / (CPR * DT);

  // ---------- distance ----------
  float distL = (curL / CPR) * (PI * WHEEL_DIAMETER);
  float distR = (curR / CPR) * (PI * WHEEL_DIAMETER);

  float distance = (distL + distR) * 0.5;

  if(distance >= TARGET_DISTANCE){

    setMotorPWM(0,0);
    Serial.println("DONE");
    return;
  }

  // ---------- SPEED PI ----------
  float errR = TARGET_RPM - rpmR;
  float errL = TARGET_RPM - rpmL;

  intR += errR * DT;
  intL += errL * DT;

  intR = constrain(intR,-30,30);
  intL = constrain(intL,-30,30);

  int pwmR = 70 + (int)(Kp_speed*errR + Ki_speed*intR);
  int pwmL = 70 + (int)(Kp_speed*errL + Ki_speed*intL);

  // ---------- STRAIGHT CORRECTION ----------
  long straightError = curL - curR;

  pwmL -= K_straight * straightError;
  pwmR += K_straight * straightError;

  pwmL = constrain(pwmL,0,255);
  pwmR = constrain(pwmR,0,255);

  setMotorPWM(pwmL,pwmR);

  Serial.print("dist=");
  Serial.print(distance);
  Serial.print(" rpmL=");
  Serial.print(rpmL);
  Serial.print(" rpmR=");
  Serial.print(rpmR);
  Serial.print(" pwmL=");
  Serial.print(pwmL);
  Serial.print(" pwmR=");
  Serial.println(pwmR);
}