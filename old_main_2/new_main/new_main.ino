#include <Wire.h>

#define MOTOR_I2C_ADDRESS 0x50
#define MOTOR1_COMMAND 0x85  

// Angle encoder pins
const int angleEncPinA = 3;
const int angleEncPinB = 4;

// Distance encoder pins
const int distEncPinA = 2;
const int distEncPinB = 7;

// Encoder counters
volatile long angleCount = 0;
volatile long distCount = 0;

volatile unsigned long minIntervalAngle = 1000000;
volatile unsigned long minIntervalDist = 1000000;
volatile unsigned long lastPulseTimeAngle = 0;
volatile unsigned long lastPulseTimeDist = 0;

// Timing
double currentTime;
double measureInterval = 50; // ms

double lastMeasureTime = 0;
double lastAngleCount = 0;
double lastDist = 0;
double lastAngleVelocity = 0.0;
double lastDistVelocity = 0.0;
double lastAngleRad;
double lastDeltaRad;

// Motion state variables
double deltaAngleCount;
double deltaTime;
double angleRad;
double angleVelocity; 
double angleAccel; 
double angleVelocityRad;
double angleAccelRad;
double deltaDist;
double distVelocity;
double distAccel;
long currentAngleCount, currentDistCount;
double currentDist;

bool centered = false;
int motorPWM;

// Constant terms
const double m_p = 0.03;
const double m_c = 0.2;
const double g = 9.81;
const double l = 0.3;

// Dynamic state
double theta = angleRad;
double dtheta = angleVelocity;
double ddtheta = angleAccel;
double dx = distVelocity;
double ddx = distAccel;
double hamilton_initial;
double hamilton_initial_partial;
double hamilton_final;
double new_ddx;
double new_dx;
volatile double targetedFinalTheta = radians(60);
double stage = 0;

bool motorStop = false;
bool onCorrectSide;
bool passedPeak;
double direction;

void setup() {
  pinMode(angleEncPinA, INPUT_PULLUP);
  pinMode(angleEncPinB, INPUT_PULLUP);
  pinMode(distEncPinA, INPUT_PULLUP);
  pinMode(distEncPinB, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(115200);  
  Serial.println("System initialized");

  attachInterrupt(digitalPinToInterrupt(angleEncPinA), readAngleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(distEncPinA), readDistEncoder, CHANGE);
}

void loop() {
  currentTime = millis();
  noInterrupts();
  currentAngleCount = angleCount;
  currentDistCount = distCount;
  interrupts();
  
  if (currentTime - lastMeasureTime >= measureInterval) { 
    measure();

    if (motorStop && onCorrectSide && passedPeak){
      targetedFinalTheta = (abs(angleRad) + radians(10)) * pow(-1, stage);
      motorStop = false;
    }

    if (!motorStop){
      calculateVelocityEnergyEquation(targetedFinalTheta);
      changeVelocity(new_dx);
      // accelerate(new_ddx);
    }
    else{
      // motorPWM += (motorPWM > 0) ? -10: 10;
      // if (abs(motorPWM) < 60){
      //   motorPWM = 0;
      // }
      motor1(0);
    } 
    motor1(motorPWM); 
    Serial.println(" ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ ");
  }

  
}

void accelerate(double accel) {
  double deltaV = accel * measureInterval / 1000.0; 
  double virtualVelo = distVelocity;

  double targetV = virtualVelo + deltaV;

  motorPWM = (int)(abs(targetV) * 176.81 + 65.253);

  if (targetV < 0) motorPWM *= -1;

  // Debug
  Serial.print("targetV: "); Serial.print(targetV); Serial.print(" ");
  Serial.print("deltaV: "); Serial.print(deltaV); Serial.print(" ");
  Serial.print("accel: "); Serial.print(accel); Serial.print(" ");
  Serial.print("motorPWM: "); Serial.println(motorPWM);
}

void measure() {
  deltaAngleCount = currentAngleCount - lastAngleCount;
  deltaTime = (currentTime - lastMeasureTime) / 1000.0;

  angleRad = currentAngleCount / 2048.0 * 2 * PI;
  angleVelocity = deltaAngleCount / deltaTime;
  angleAccel = (angleVelocity - lastAngleVelocity) / deltaTime;

  angleVelocityRad = (angleVelocity / 2048.0) * 2 * PI;
  angleAccelRad = (angleAccel / 2048.0) * 2 * PI;

  currentDist = (double)currentDistCount / 50.378 / 100.0;
  deltaDist = currentDist - lastDist;
  distVelocity = deltaDist / deltaTime;
  distAccel = (distVelocity - lastDistVelocity) / deltaTime;

  passedPeak = (angleRad - lastAngleRad) * lastDeltaRad < 0;
  onCorrectSide = angleRad * targetedFinalTheta > 0;

  Serial.print("θ: "); Serial.print(angleRad, 4); Serial.print(" rad, ");
  Serial.print("θ̇: "); Serial.print(angleVelocityRad, 4); Serial.print(" rad/s, ");
  Serial.print("θ̈: "); Serial.print(angleAccelRad, 4); Serial.print(" rad/s², ");
  Serial.print("x: "); Serial.print(currentDist, 3); Serial.print(" m, ");
  Serial.print("ẋ: "); Serial.print(distVelocity, 3); Serial.print(" m/s, ");
  Serial.print("ẍ: "); Serial.print(distAccel, 3); Serial.print(" m/s², ");
 
  Serial.print("H_f: "); Serial.print(hamilton_initial, 3); Serial.print(", ");
  Serial.print("H_i: "); Serial.print(hamilton_final, 3); Serial.print(", ");
  Serial.print("ΔE: "); Serial.print(hamilton_final- hamilton_initial, 3); Serial.print(", ");
  Serial.print("θf: "); Serial.print(targetedFinalTheta); Serial.print("rad, ");
  Serial.print("new dx: "); Serial.print(new_dx, 3); Serial.print(", ");

  Serial.print("Stop: "); Serial.print(motorStop); Serial.print(", ");
  Serial.print("onCorrectSide: "); Serial.print(onCorrectSide); Serial.print(", ");
  Serial.print("passedPeak: "); Serial.print(passedPeak); Serial.print(", ");
  Serial.print("PWM: "); Serial.println(motorPWM);


  lastDeltaRad = angleRad - lastAngleRad;
  lastAngleRad = angleRad;
  lastMeasureTime = currentTime;
  lastAngleCount = currentAngleCount;
  lastDist = currentDist;
  lastAngleVelocity = angleVelocity;
  lastDistVelocity = distVelocity;
  
}

void calculateEnergyEquation(double theta_final){

  theta = angleRad;
  dtheta = angleVelocityRad;
  dx = distVelocity;

  hamilton_initial = 0.5 * (m_c + m_p) * pow(dx,2) + 0.5 * m_p * dx * dtheta * l * cos(theta) + 7.0/24.0 * m_p * pow(l,2) * pow(dtheta,2) + 0.5 * m_p * l * g * (1 - cos(theta)) + 0.5 * m_c * g * l;
  hamilton_final = 0.5 * m_p * g * l * (1 - cos(theta_final)) + 0.5 * m_c * g * l;

  hamilton_initial_partial = 0.5 * m_p * dx * dtheta * l * cos(theta) + 7.0/24.0 * m_p * pow(l,2) * pow(dtheta,2) + 0.5 * m_p * l * g * (1 - cos(theta)) + 0.5 * m_c * g * l;

  if (hamilton_final > hamilton_initial){
    new_ddx = (1000 / measureInterval) * (sqrt( (2 / (m_c + m_p)) * (hamilton_final - hamilton_initial)) - dx);
    new_dx = sqrt( 2 / (m_c + m_p) * (hamilton_final - hamilton_initial_partial) );

    if (targetedFinalTheta < theta) {
      new_ddx *= -1;
      new_dx *= -1;
    }
  }

  else{
    stage += 1;
    motorStop = true;
  }
  
  
}


void changeVelocity(double velo) {
  motorPWM = (int)(abs(velo) * 176.81 + 65.253);
  if (velo < 0) motorPWM *= -1;
}

void calculateVelocityEnergyEquation(double theta_final){
  theta = angleRad;
  dtheta = angleVelocityRad;
  dx = distVelocity;

  hamilton_initial = 0.5 * (m_c + m_p) * pow(dx,2) + 0.5 * m_p * dx * dtheta * l * cos(theta) + 7.0/24.0 * m_p * pow(l,2) * pow(dtheta,2) + 0.5 * m_p * l * g * (1 - cos(theta)) + 0.5 * m_c * g * l;
  hamilton_final = 0.5 * m_p * g * l * (1 - cos(theta_final)) + 0.5 * m_c * g * l;

  double a = 0.5 * (m_c + m_p); 
  double b = 0.5 * m_p * dtheta * l * cos(theta);
  double c = 7.0/24.0 * m_p * pow(l,2) * pow(dtheta,2) + 0.5 * m_p * l * g * (1 - cos(theta)) - 0.5 * m_p * g * l * (1 - cos(theta_final));

  if (hamilton_final > hamilton_initial){

    new_dx = (-b + sqrt(b*b - 4*a*c)) / (2*a);

    if (targetedFinalTheta < theta){
      new_dx = (-b - sqrt(b*b - 4*a*c)) / (2*a);
    }

  }

  else{
    stage += 1;
    motorStop = true;
  }

  
}
































void motor1(int tSpeed) {
  int tPWM = abs(tSpeed);
  int tDir;

  if (tSpeed > 0)      tDir = 1;
  else if (tSpeed < 0) tDir = 2;
  else                 tDir = 3;

  if (tPWM > 255) tPWM = 255;

  Wire.beginTransmission(MOTOR_I2C_ADDRESS);
  Wire.write(MOTOR1_COMMAND);
  Wire.write(tDir);
  Wire.write(tPWM);
  Wire.endTransmission();
}

void readAngleEncoder() {
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTimeAngle;
  if (interval < minIntervalAngle) minIntervalAngle = interval;
  lastPulseTimeAngle = now;

  bool A = digitalRead(angleEncPinA);
  bool B = digitalRead(angleEncPinB);
  angleCount += (B == A) ? 1 : -1;
}

void readDistEncoder() {
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTimeDist;
  if (interval < minIntervalDist) minIntervalDist = interval;
  lastPulseTimeDist = now;

  bool A = digitalRead(distEncPinA);
  bool B = digitalRead(distEncPinB);
  distCount += (B == A) ? 1 : -1;
}



double centerTarget = 0.67 / 2;
void moveToCenter() {
  if (currentDist < centerTarget) {
    motor1(80);
  } 
  else {
    motor1(0);
    centered = true;
  }   



}
