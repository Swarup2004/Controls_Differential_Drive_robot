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
const float WHEEL_RADIUS   = WHEEL_DIAMETER / 2.0;
const float WHEEL_BASE     = 0.202;

const float CPR = 208.0;

// control loop for pose update + control
const float DT = 0.02; // 20 ms

// rpm measurement window (bigger = smoother)
const float RPM_DT = 0.10; // 100 ms
const int   RPM_STEPS = (int)(RPM_DT / DT + 0.5f);

// ---------------- GOAL ----------------
float x_goal = 1;
float y_goal = 0.22;   // <-- DO NOT write 0.-3
float theta_goal = 0; // desired final heading (rad)

// ---------------- NAVIGATION GAINS ----------------
float K_rho = 0.7;

// ---------------- HEADING PID (for alpha and for final rotate) ----------------
float Kp_heading = 1.3;
float Ki_heading = 0.0;
float Kd_heading = 0.15;

float headingIntegral = 0;
float prevErrHeading = 0;

// ---------------- SPEED PI ----------------
float Kp_speed = 2.0;
float Ki_speed = 5.0;

float intL = 0;
float intR = 0;

// ---------------- STATE ----------------
volatile long leftCount  = 0;
volatile long rightCount = 0;

float x = 0;
float y = 0;
float theta = 0;

// ---------------- MODE ----------------
enum Mode { GO_TO_POINT, TURN_TO_HEADING, DONE };
Mode mode = GO_TO_POINT;

// ---------------- ENCODER ISR ----------------
void leftISR(){
  if(digitalRead(L_ENC_B)) leftCount++;
  else leftCount--;
}
void rightISR(){
  if(digitalRead(R_ENC_B)) rightCount++;
  else rightCount--;
}

// wrap angle to [-pi, pi]
float wrapPi(float a){
  while(a > PI)  a -= 2*PI;
  while(a < -PI) a += 2*PI;
  return a;
}

void setMotorPWM(int pwmL, int pwmR){
  pwmL = constrain(pwmL, 0, 255);
  pwmR = constrain(pwmR, 0, 255);
  analogWrite(L_EN, pwmL);
  analogWrite(R_EN, pwmR);
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

  // forward direction
  digitalWrite(R_IN1,HIGH);
  digitalWrite(R_IN2,LOW);
  digitalWrite(L_IN1,HIGH);
  digitalWrite(L_IN2,LOW);

  setMotorPWM(0,0);
}

void loop(){
  static unsigned long last=0;
  if(millis()-last < (unsigned long)(DT*1000)) return;
  last=millis();

  // encoder delta (20ms)
  static long prevL=0, prevR=0;
  long curL = leftCount;
  long curR = rightCount;
  long dL = curL - prevL;
  long dR = curR - prevR;
  prevL = curL;
  prevR = curR;

  // pose update (odometry)
  float distL = (dL/CPR) * (PI*WHEEL_DIAMETER);
  float distR = (dR/CPR) * (PI*WHEEL_DIAMETER);

  float ds     = (distL + distR) * 0.5f;
  float dtheta = (distR - distL) / WHEEL_BASE;

  theta = wrapPi(theta + dtheta);
  x += ds * cos(theta);
  y += ds * sin(theta);

  // ---------- RPM measurement over 100 ms ----------
  static int stepCount = 0;
  static long sumDL = 0, sumDR = 0;
  static float rpmL = 0, rpmR = 0;

  sumDL += dL;
  sumDR += dR;
  stepCount++;

  if(stepCount >= RPM_STEPS){
    float windowT = stepCount * DT;
    rpmL = (sumDL * 60.0f) / (CPR * windowT);
    rpmR = (sumDR * 60.0f) / (CPR * windowT);
    sumDL = 0;
    sumDR = 0;
    stepCount = 0;
  }

  // ---------- MODE LOGIC ----------
  float v = 0.0f; // forward m/s
  float w = 0.0f; // yaw rad/s

  // distance to goal
  float dx = x_goal - x;
  float dy = y_goal - y;
  float rho = sqrt(dx*dx + dy*dy);

  if(mode == GO_TO_POINT){
    // heading to goal
    float theta_target = atan2(dy, dx);
    float alpha = wrapPi(theta_target - theta);

    if(rho < 0.05f){
      // switch to turn-in-place at the goal
      mode = TURN_TO_HEADING;
      headingIntegral = 0;
      prevErrHeading = 0;
      intL = intR = 0;
    } else {
      // forward speed
      v = K_rho * rho * cos(alpha);
      v = constrain(v, -0.18f, 0.18f);

      // heading PID on alpha
      headingIntegral += alpha * DT;
      headingIntegral = constrain(headingIntegral, -2.0f, 2.0f);
      float dErr = (alpha - prevErrHeading) / DT;
      w = Kp_heading*alpha + Ki_heading*headingIntegral + Kd_heading*dErr;
      prevErrHeading = alpha;

      w = constrain(w, -2.0f, 2.0f);
    }
  }
  else if(mode == TURN_TO_HEADING){
    // stop forward, rotate to theta_goal
    float err = wrapPi(theta_goal - theta);

    if(fabs(err) < 0.05f){
      mode = DONE;
      setMotorPWM(0,0);
    } else {
      v = 0.0f;
      headingIntegral += err * DT;
      headingIntegral = constrain(headingIntegral, -2.0f, 2.0f);
      float dErr = (err - prevErrHeading) / DT;
      w = Kp_heading*err + Ki_heading*headingIntegral + Kd_heading*dErr;
      prevErrHeading = err;

      w = constrain(w, -2.0f, 2.0f);
    }
  }
  else { // DONE
    setMotorPWM(0,0);
  }

  // wheel linear velocities
  float vR = v + (w*WHEEL_BASE*0.5f);
  float vL = v - (w*WHEEL_BASE*0.5f);

  // rpm references
  float rpmRefR = (vR/WHEEL_RADIUS)*(60.0f/(2.0f*PI));
  float rpmRefL = (vL/WHEEL_RADIUS)*(60.0f/(2.0f*PI));

  // PI speed control
  float errR = rpmRefR - rpmR;
  float errL = rpmRefL - rpmL;

  intR += errR * DT;
  intL += errL * DT;
  intR = constrain(intR, -30.0f, 30.0f);
  intL = constrain(intL, -30.0f, 30.0f);

  int pwmR = 70 + (int)(Kp_speed*errR + Ki_speed*intR);
  int pwmL = 70 + (int)(Kp_speed*errL + Ki_speed*intL);

  // if done, force 0
  if(mode == DONE){
    pwmL = 0; pwmR = 0;
  }

  setMotorPWM(pwmL, pwmR);

  // debug
  Serial.print("mode=");
  Serial.print((int)mode);
  Serial.print(" x=");
  Serial.print(x, 2);
  Serial.print(" y=");
  Serial.print(y, 2);
  Serial.print(" theta=");
  Serial.print(theta, 2);
  Serial.print(" rpmL=");
  Serial.print(rpmL, 2);
  Serial.print(" rpmR=");
  Serial.print(rpmR, 2);
  Serial.print(" pwmL=");
  Serial.print(pwmL);
  Serial.print(" pwmR=");
  Serial.println(pwmR);
}