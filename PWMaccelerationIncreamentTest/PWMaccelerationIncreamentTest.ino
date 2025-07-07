#include <Wire.h>

#define MOTOR_I2C_ADDRESS 0x50
#define MOTOR1_COMMAND 0x85  

// Angle encoder pins
const int angleEncPinA = 3;
const int angleEncPinB = 4;

// Distance encoder pins
const int distEncPinA = 2;
const int distEncPinB = 7;

// input pin
const int increaseIncreamentPin = 13;
const int decreaseIncreamentPin = 12;
const int movePin = 8;

// Encoder counters
volatile long angleCount = 0;
volatile long distCount = 0;
volatile long currentAngleCount;
volatile long currentDistCount;

volatile unsigned long minIntervalAngle = 1000000;
volatile unsigned long minIntervalDist = 1000000;
volatile unsigned long lastPulseTimeAngle = 0;
volatile unsigned long lastPulseTimeDist = 0;

// Timing
double currentTime;
double measureInterval = 100; 
double lastMeasureTime;
double accelerateInterval = 200;
double lastAccelerateTime;
double lastDebounceTime;
double debounceDelay = 500;
//Constants
double m_c = 0.2;
double m_p = 0.03;
double l = 0.3;
double g = 9.81;
double x_max = 0.67;
double centerDistance = x_max / 2; 
double offsetDistance = 0.15;
double targetedFinalDistance;

//Variables
double theta;
double dtheta;
double ddtheta;
double x;
double dx;
double ddx;

//Term for calculating time variant variables
double x_old;
double dx_old;
double theta_old;
double dtheta_old;

//Motor Control
int motorPWM = 75;
int PWMincreament = 25;
bool motorRunning = false;
int currentDirection = 0;
bool isTuning = true;

void setup() {
  pinMode(angleEncPinA, INPUT_PULLUP);
  pinMode(angleEncPinB, INPUT_PULLUP);
  pinMode(distEncPinA, INPUT_PULLUP);
  pinMode(distEncPinB, INPUT_PULLUP);
  pinMode(increaseIncreamentPin, INPUT_PULLUP); 
  pinMode(decreaseIncreamentPin, INPUT_PULLUP); 
  pinMode(movePin, INPUT_PULLUP); 

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

  //measure x and theta
  measureTimeInvariantVaribles();
  
  if (currentTime - lastMeasureTime > measureInterval){
    measureTimeVariantVariables();
    printLog();
  }

  int increasePinVal = digitalRead(increaseIncreamentPin);
  if( increasePinVal == LOW && (currentTime - lastDebounceTime) > debounceDelay){
    lastDebounceTime = currentTime;
    PWMincreament += 1;
  }

  int decreasePinVal = digitalRead(decreaseIncreamentPin);
  if( decreasePinVal == LOW && (currentTime - lastDebounceTime) > debounceDelay){
    lastDebounceTime = currentTime;
    PWMincreament -= 1;
  }

  int movePinVal = digitalRead(movePin);
  if( movePinVal == LOW && (currentTime - lastDebounceTime) > debounceDelay && !motorRunning){
    lastDebounceTime = currentTime;
    motorRunning = true;
  }

  if (motorRunning){
    motor(motorPWM);
  }
  
  if (currentTime - lastAccelerateTime > accelerateInterval && motorRunning){
    acceleratePWM(PWMincreament,currentDirection);
  }

  if (motorRunning && !isTuning){
    if (currentDirection == 0 && x > 0.5){
      motorRunning = false;
      isTuning = true;
      currentDirection == 1; 
    }
    if (currentDirection == 1 && x < 0.1){
      motorRunning = false;
      isTuning = true;
      currentDirection == 0; 
    }
  }

  if (!motorRunning && isTuning) {
    tuneTo((currentDirection == 0) ? 0.1 : 0.5, 80);
  }
}

void measureTimeInvariantVaribles(){
  x = (double)currentDistCount / 50.378 / 100.0;
  theta = (double)currentAngleCount / 2048 * 2 * PI;
}

void measureTimeVariantVariables(){
  double deltaTime = (currentTime - lastMeasureTime) / 1000;
  dx = (x - x_old) / deltaTime;
  ddx = (dx - dx_old) / deltaTime;
  dtheta = (theta - theta_old) / deltaTime;
  ddtheta = (dtheta - dtheta_old) / deltaTime;

  x_old = x;
  dx_old = dx;
  theta_old = theta; 
  dtheta_old = dtheta; 

  lastMeasureTime = currentTime;
} 

void acceleratePWM(int increament, bool direction){
  motorPWM = abs(motorPWM) + increament;
  if (motorPWM < 75) motorPWM = 75;
  if (motorPWM > 255) motorPWM = 255;
  if (direction == 1){
    motorPWM *= -1;
  }
  
  lastAccelerateTime = currentTime;
}


void motor(int tSpeed) {
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

void tuneTo(double loc,int PWM){
  if (abs(loc-x) > 0.01){
    motor(x > loc ? -PWM : PWM );
  }
  else{
    motor(0);
    isTuning = false;
  }
}

void printLog(){
  Serial.print(x);Serial.print(",");
  Serial.print(dx);Serial.print(",");
  Serial.print(ddx);Serial.print(",");
  Serial.print(theta);Serial.print(",");
  Serial.print(dtheta);Serial.print(",");
  Serial.print(ddtheta);Serial.print(",");
  Serial.print(motorPWM);Serial.print(",");
  Serial.println(PWMincreament);
}