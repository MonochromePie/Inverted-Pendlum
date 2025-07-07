#include <Wire.h>


#define MOTOR_I2C_ADDRESS 0x50
#define MOTOR1_COMMAND 0x85  

//angle encoder
const int encoderPinA = 3;
const int encoderPinB = 4;

//distance encoder
const int encoderPinC = 2;
const int encoderPinD = 7;

volatile long encoderPosCount = 0;
volatile long encoderDisCount = 0;

volatile unsigned long minIntervalAngle = 1000000;
volatile unsigned long minIntervalDist = 1000000;
volatile unsigned long lastPulseTimeAngle = 0;
volatile unsigned long lastPulseTimeDist = 0;

unsigned long currentTime;
long incrementDuration = 100;


unsigned long lastTime = 0;
long lastPos = 0;
double lastDis = 0;
double lastVelocity = 0.0;
double lastDistanceVelocity = 0.0;

long deltaPos;
double deltaTime;
double anglePos;
double velocity; 
double acceleration; 
double velocityRad;
double accelerationRad;
double deltaDis;
double velocityDistance;
double accelerationDistance;
long currentPos, currentDis;
double realDis;

  

bool initiated = false;
int motorPWM;


double swingTime = 500; //time needed for each of the swing

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinC, INPUT_PULLUP);
  pinMode(encoderPinD, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(115200);  
  Serial.println("Encoder test initiated");

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoderAngle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinC), readEncoderDistance, CHANGE);
}

void loop() {

  currentTime = millis();
  // Read encoder values safely
  noInterrupts();
  currentPos = encoderPosCount;
  currentDis = encoderDisCount;
  interrupts();

  //all measurement done at 100 ms increment to reduce encoder noise
  if (currentTime - lastTime >= incrementDuration) { 
    measure();   
  }

  motor1(motorPWM); // motor always running
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

void readEncoderAngle() {
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTimeAngle;
  if (interval < minIntervalAngle) minIntervalAngle = interval;
  lastPulseTimeAngle = now;

  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);
  encoderPosCount += (B == A) ? 1 : -1;
}

void readEncoderDistance() {
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTimeDist;
  if (interval < minIntervalDist) minIntervalDist = interval;
  lastPulseTimeDist = now;

  bool A = digitalRead(encoderPinC);
  bool B = digitalRead(encoderPinD);
  encoderDisCount += (B == A) ? 1 : -1;
}

double halfPoint = 0.67/2; //system half point

void moveToCenter(){
  if (realDis < halfPoint){
    motor1(100);
  }
  else{
    motor1(0);
    initiated = true;
  }   
}

void accelerate(double acceleration){
  double addedVelocity = acceleration * incrementDuration / 1000; 
  double targetVelocity = velocityDistance + addedVelocity; //the new velocity that need to be adjusted to correlate to the acceleration
  motorPWM = (int)(abs(targetVelocity) * 176.81 + 65.253); //velocity to PWM equation;

  if (targetVelocity < 0){
    motorPWM *= -1; 
  }

}

// measure all value connected to the encoder
void measure(){
  deltaPos = currentPos - lastPos;
  deltaTime = (double)(currentTime - lastTime) / 1000.0;

  anglePos = currentPos/2048.0*2*PI; // angle encoder 2048 P/R
  velocity = deltaPos / deltaTime;
  acceleration = (velocity - lastVelocity) / deltaTime;

  velocityRad = (velocity / 2048.0) * 2 * PI;
  accelerationRad = (acceleration / 2048.0) * 2 * PI;

  // double velocityDeg = (velocity / 2048.0) * 360;
  // double accelerationDeg = (acceleration / 2048.0) * 360;

  realDis = (double)currentDis / 50.378 / 100.0; // pulse to meter conversion 100 P/R
  deltaDis = realDis - lastDis;
  velocityDistance = deltaDis / deltaTime;
  accelerationDistance = (velocityDistance - lastDistanceVelocity) / deltaTime;

  // Serial.print("Angle Pos: "); Serial.print(anglePos,2); Serial.print(" rad");
  // Serial.print(" | Ang Vel: "); Serial.print(velocityRad, 2); Serial.print(" rad/s");
  // Serial.print(" | Ang Accel: "); Serial.print(accelerationRad, 2); Serial.print(" rad/s²");
  // Serial.print(" | Dis: "); Serial.print(realDis, 3); Serial.print(" m");
  // Serial.print(" | Dis Vel: "); Serial.print(velocityDistance, 3); Serial.print(" m/s");
  // Serial.print(" | Dis Accel: "); Serial.print(accelerationDistance, 3); Serial.print(" m/s²");
  // Serial.print(" | Time: "); Serial.print(currentTime/1000.00); Serial.println(" sec");

  // serial print for serial plot
  Serial.print(currentPos / 2048.0 * 2 * PI);  //ang in rad
  Serial.print(",");                       
  Serial.print(velocityRad);              // ang Velocity in rad/s
  Serial.print(",");
  Serial.print(accelerationRad);          // ang Acceleration in rad/s^2
  Serial.print(",");
  Serial.print(realDis, 3);                  // Distance in m
  Serial.print(",");
  Serial.print(velocityDistance, 3);         // Distance velo in m/s
  Serial.print(",");
  Serial.print(accelerationDistance, 3);        // Distance accel in m/s²
  Serial.print(",");
  Serial.print(20);                 //bound
  Serial.print(",");
  Serial.print(-20);               //bound
  Serial.print(",");

  lastTime = currentTime;
  lastPos = currentPos;
  lastDis = realDis;
  lastVelocity = velocity;
  lastDistanceVelocity = velocityDistance;
}


//path in which the pendulum is destined to take
double theta_p;
double dtheta_p;
double ddtheta_p;

void definePath(double theta_initial, double theta_final, long){
  theta_p = theta_initial + (theta_final - theta_initial) * sin(PI * )

}