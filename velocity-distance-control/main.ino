#include <math.h>

// ================= PIN CONFIGURATION =================
#define RIGHT_PWM   6
#define RIGHT_IN1   7
#define RIGHT_IN2   8

#define LEFT_PWM    11
#define LEFT_IN1    12
#define LEFT_IN2    13

#define LEFT_ENC_A  14
#define LEFT_ENC_B  15
#define RIGHT_ENC_A 16
#define RIGHT_ENC_B 17

// ================= ROBOT PARAMETERS =================
const float WHEEL_DIAMETER = 0.069f;
const float CPR = 208.0f;

const float DT = 0.02f;  // control loop period (20 ms)

// ================= TASK PARAMETERS =================
const float TARGET_DISTANCE = 2.0f;   // meters to travel
const float TARGET_RPM      = 140.0f; // desired wheel speed

// ================= CONTROLLER GAINS =================
float Kp_speed = 2.0f;
float Ki_speed = 5.0f;

float K_straight = 0.02f;  // correction gain for straight motion

// ================= CONTROLLER STATE =================
float integralLeftRPM  = 0.0f;
float integralRightRPM = 0.0f;

// ================= ENCODER STATE =================
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

// ================= ENCODER INTERRUPTS =================
void leftEncoderISR(){
  if(digitalRead(LEFT_ENC_B)) encoderLeftCount++;
  else encoderLeftCount--;
}

void rightEncoderISR(){
  if(digitalRead(RIGHT_ENC_B)) encoderRightCount++;
  else encoderRightCount--;
}

// ================= MOTOR CONTROL =================
void setMotorPWM(int pwmLeft, int pwmRight){
  pwmLeft  = constrain(pwmLeft,  0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  analogWrite(LEFT_PWM,  pwmLeft);
  analogWrite(RIGHT_PWM, pwmRight);
}

// ================= SETUP =================
void setup(){

  Serial.begin(115200);

  // Motor pins
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach interrupts for encoder A channels
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Set forward direction
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  setMotorPWM(0, 0);
}

// ================= MAIN LOOP =================
void loop(){

  static unsigned long lastTime = 0;
  if(millis() - lastTime < DT * 1000) return;
  lastTime = millis();

  // ===== ENCODER DELTA =====
  static long prevLeftCount = 0, prevRightCount = 0;

  long currLeftCount  = encoderLeftCount;
  long currRightCount = encoderRightCount;

  long deltaLeft  = currLeftCount  - prevLeftCount;
  long deltaRight = currRightCount - prevRightCount;

  prevLeftCount  = currLeftCount;
  prevRightCount = currRightCount;

  // ===== RPM ESTIMATION =====
  float rpmLeft  = (deltaLeft  * 60.0f) / (CPR * DT);
  float rpmRight = (deltaRight * 60.0f) / (CPR * DT);

  // ===== DISTANCE ESTIMATION =====
  float distLeft  = (currLeftCount  / CPR) * (PI * WHEEL_DIAMETER);
  float distRight = (currRightCount / CPR) * (PI * WHEEL_DIAMETER);

  float traveledDistance = (distLeft + distRight) * 0.5f;

  // Stop condition
  if(traveledDistance >= TARGET_DISTANCE){
    setMotorPWM(0, 0);
    Serial.println("DONE");
    return;
  }

  // ===== SPEED PI CONTROL =====
  float errorRightRPM = TARGET_RPM - rpmRight;
  float errorLeftRPM  = TARGET_RPM - rpmLeft;

  integralRightRPM += errorRightRPM * DT;
  integralLeftRPM  += errorLeftRPM  * DT;

  integralRightRPM = constrain(integralRightRPM, -30.0f, 30.0f);
  integralLeftRPM  = constrain(integralLeftRPM,  -30.0f, 30.0f);

  int pwmRight = 70 + (int)(Kp_speed * errorRightRPM + Ki_speed * integralRightRPM);
  int pwmLeft  = 70 + (int)(Kp_speed * errorLeftRPM  + Ki_speed * integralLeftRPM);

  // ===== STRAIGHT LINE CORRECTION =====
  // Use encoder difference to correct drift
  long encoderError = currLeftCount - currRightCount;

  pwmLeft  -= K_straight * encoderError;
  pwmRight += K_straight * encoderError;

  pwmLeft  = constrain(pwmLeft,  0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  setMotorPWM(pwmLeft, pwmRight);

  // ===== DEBUG OUTPUT =====
  Serial.print("dist=");
  Serial.print(traveledDistance);
  Serial.print(" rpmL=");
  Serial.print(rpmLeft);
  Serial.print(" rpmR=");
  Serial.print(rpmRight);
  Serial.print(" pwmL=");
  Serial.print(pwmLeft);
  Serial.print(" pwmR=");
  Serial.println(pwmRight);
}
