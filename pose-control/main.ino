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
const float WHEEL_RADIUS   = WHEEL_DIAMETER / 2.0f;
const float WHEEL_BASE     = 0.202f;

const float CPR = 208.0f;

// Control loop timing
const float DT = 0.02f; // 20 ms

// RPM smoothing window
const float RPM_WINDOW = 0.10f;
const int   RPM_STEPS  = (int)(RPM_WINDOW / DT + 0.5f);

// ================= GOAL =================
float goalX = 1.0f;
float goalY = 0.22f;
float goalTheta = 0.0f; //Theta = 0 means, The robot is heading towards its initial direction after achieving the GOAL X,Y.

// ================= NAVIGATION GAINS =================
float Kp_rho = 0.7f;

// ================= HEADING PID =================
float Kp_heading = 1.3f;
float Ki_heading = 0.0f;
float Kd_heading = 0.15f;

float headingIntegral = 0.0f;
float prevHeadingError = 0.0f;

// ================= SPEED PI =================
float Kp_speed = 2.0f;
float Ki_speed = 5.0f;

float integralLeftRPM  = 0.0f;
float integralRightRPM = 0.0f;

// ================= STATE =================
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

float posX = 0.0f;
float posY = 0.0f;
float theta = 0.0f;

// ================= MODE =================
enum Mode { MOVE_TO_POINT, ALIGN_HEADING, COMPLETE };
Mode currentMode = MOVE_TO_POINT;

// ================= ENCODER INTERRUPTS =================
void leftEncoderISR(){
  if(digitalRead(LEFT_ENC_B)) encoderLeftCount++;
  else encoderLeftCount--;
}

void rightEncoderISR(){
  if(digitalRead(RIGHT_ENC_B)) encoderRightCount++;
  else encoderRightCount--;
}

// ================= UTILITY FUNCTIONS =================

// Wrap angle to [-π, π]
float wrapAngle(float angle){
  while(angle > PI)  angle -= 2*PI;
  while(angle < -PI) angle += 2*PI;
  return angle;
}

// Apply PWM to motors
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

  // Interrupts
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
  if(millis() - lastTime < (unsigned long)(DT * 1000)) return;
  lastTime = millis();

  // ===== ENCODER DELTA =====
  static long prevLeftCount = 0, prevRightCount = 0;

  long currLeftCount  = encoderLeftCount;
  long currRightCount = encoderRightCount;

  long deltaLeft  = currLeftCount  - prevLeftCount;
  long deltaRight = currRightCount - prevRightCount;

  prevLeftCount  = currLeftCount;
  prevRightCount = currRightCount;

  // ===== ODOMETRY UPDATE =====
  float distLeft  = (deltaLeft  / CPR) * (PI * WHEEL_DIAMETER);
  float distRight = (deltaRight / CPR) * (PI * WHEEL_DIAMETER);

  float ds     = (distLeft + distRight) * 0.5f;
  float dTheta = (distRight - distLeft) / WHEEL_BASE;

  theta = wrapAngle(theta + dTheta);
  posX += ds * cos(theta);
  posY += ds * sin(theta);

  // ===== RPM ESTIMATION =====
  static int stepCounter = 0;
  static long sumLeftCounts = 0, sumRightCounts = 0;
  static float rpmLeft = 0, rpmRight = 0;

  sumLeftCounts  += deltaLeft;
  sumRightCounts += deltaRight;
  stepCounter++;

  if(stepCounter >= RPM_STEPS){
    float windowTime = stepCounter * DT;

    rpmLeft  = (sumLeftCounts  * 60.0f) / (CPR * windowTime);
    rpmRight = (sumRightCounts * 60.0f) / (CPR * windowTime);

    sumLeftCounts = 0;
    sumRightCounts = 0;
    stepCounter = 0;
  }

  // ===== CONTROL VARIABLES =====
  float v = 0.0f; // linear velocity
  float w = 0.0f; // angular velocity

  // Distance to goal
  float dx = goalX - posX;
  float dy = goalY - posY;
  float rho = sqrt(dx*dx + dy*dy);

  // ===== CONTROL LOGIC =====
  if(currentMode == MOVE_TO_POINT){

    float targetHeading = atan2(dy, dx);
    float headingError = wrapAngle(targetHeading - theta);

    if(rho < 0.05f){
      currentMode = ALIGN_HEADING;

      headingIntegral = 0;
      prevHeadingError = 0;
      integralLeftRPM = integralRightRPM = 0;
    }
    else{
      v = Kp_rho * rho * cos(headingError);
      v = constrain(v, -0.18f, 0.18f);

      // Heading PID
      headingIntegral += headingError * DT;
      headingIntegral = constrain(headingIntegral, -2.0f, 2.0f);

      float dError = (headingError - prevHeadingError) / DT;

      w = Kp_heading * headingError +
          Ki_heading * headingIntegral +
          Kd_heading * dError;

      prevHeadingError = headingError;

      w = constrain(w, -2.0f, 2.0f);
    }
  }

  else if(currentMode == ALIGN_HEADING){

    float headingError = wrapAngle(goalTheta - theta);

    if(fabs(headingError) < 0.05f){
      currentMode = COMPLETE;
      setMotorPWM(0,0);
    }
    else{
      v = 0.0f;

      headingIntegral += headingError * DT;
      headingIntegral = constrain(headingIntegral, -2.0f, 2.0f);

      float dError = (headingError - prevHeadingError) / DT;

      w = Kp_heading * headingError +
          Ki_heading * headingIntegral +
          Kd_heading * dError;

      prevHeadingError = headingError;

      w = constrain(w, -2.0f, 2.0f);
    }
  }

  else{
    setMotorPWM(0,0);
  }

  // ===== CONVERT (v, w) → WHEEL VELOCITIES =====
  float velRight = v + (w * WHEEL_BASE * 0.5f);
  float velLeft  = v - (w * WHEEL_BASE * 0.5f);

  // Convert to RPM
  float rpmRefRight = (velRight / WHEEL_RADIUS) * (60.0f / (2.0f * PI));
  float rpmRefLeft  = (velLeft  / WHEEL_RADIUS) * (60.0f / (2.0f * PI));

  // ===== PI SPEED CONTROL =====
  float errorRightRPM = rpmRefRight - rpmRight;
  float errorLeftRPM  = rpmRefLeft  - rpmLeft;

  integralRightRPM += errorRightRPM * DT;
  integralLeftRPM  += errorLeftRPM  * DT;

  integralRightRPM = constrain(integralRightRPM, -30.0f, 30.0f);
  integralLeftRPM  = constrain(integralLeftRPM,  -30.0f, 30.0f);

  int pwmRight = 70 + (int)(Kp_speed * errorRightRPM + Ki_speed * integralRightRPM);
  int pwmLeft  = 70 + (int)(Kp_speed * errorLeftRPM  + Ki_speed * integralLeftRPM);

  if(currentMode == COMPLETE){
    pwmLeft = 0;
    pwmRight = 0;
  }

  setMotorPWM(pwmLeft, pwmRight);

  // ===== DEBUG OUTPUT =====
  Serial.print("mode=");
  Serial.print((int)currentMode);
  Serial.print(" x=");
  Serial.print(posX, 2);
  Serial.print(" y=");
  Serial.print(posY, 2);
  Serial.print(" theta=");
  Serial.print(theta, 2);
  Serial.print(" rpmL=");
  Serial.print(rpmLeft, 2);
  Serial.print(" rpmR=");
  Serial.print(rpmRight, 2);
  Serial.print(" pwmL=");
  Serial.print(pwmLeft);
  Serial.print(" pwmR=");
  Serial.println(pwmRight);
}
