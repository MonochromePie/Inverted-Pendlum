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
volatile long currentAngleCount;
volatile long currentDistCount;

volatile unsigned long minIntervalAngle = 1000000;
volatile unsigned long minIntervalDist = 1000000;
volatile unsigned long lastPulseTimeAngle = 0;
volatile unsigned long lastPulseTimeDist = 0;

// Timing
double currentTime;
double measureInterval = 10; 
double lastMeasureTime;

//Constants
double m_c = 0.2;
double m_p = 0.03;
double l = 0.3;
double g = 9.81;
double x_max = 0.67;
double centerDistance = x_max / 2; 
double offsetDistance = 0.05;
double targetedFinalDistance;
double tolerable_x_min = 0.2;
double tolerable_x_max = 0.5;


//Variables
double theta;
double dtheta;
double ddtheta;
double x;
double dx;
double ddx;
double distanceToCenter;


//Term for calculating time variant variables
double x_old;
double dx_old;
double theta_old;
double dtheta_old;

//Terms for energy equation
double hamilton_initial;
double hamilton_final;
double delta_E;
double final_dx_new;
double ddx_new;
double targetedFinalTheta;
double tempTargetedFinalTheta;

//use for marking beginning and ending position for swinging phase
double x_begin;
double x_end;

//use for marking deceleration point in stablizing phase
double x_i;
double x_f;


//Motor Control
int motorPWM;
int motorMaxSpeed;
int initialPWMbeforeDeceleration;
bool isMoving = false;
bool isSwinging = false;
bool calculated = false;
bool beginStablization = false;
bool decelerationPointMarked = false;
bool adjustmentPeriod = true;
bool decelerationPeriod = false;
bool targetThetaCrossed = false;

//boolean for movement control
bool travellingFromDirection = false; //false for travelling from right side

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

  //measure x and theta
  measureNoiseInsensitiveVariables();
  
  //measure time variant vairables at intervals to reduce noise
  if (currentTime - lastMeasureTime >= measureInterval){
    measureTimeVariantVariables();
  }

  //Swing pendulum to upward control phase
  if (!beginStablization){

    //setting and calculation final swinging trajectory goals for upward position for both sides
    if (!isMoving && !isSwinging && !calculated && !travellingFromDirection){
      targetedFinalTheta  = PI - radians(20);
      if (targetedFinalTheta > PI) targetedFinalTheta = PI;
      calculateEnergyEquation(theta, dtheta, targetedFinalTheta, x, dx, centerDistance + offsetDistance);
      isMoving = true;
      calculated = true;
    }

    if (!isMoving && !isSwinging && !calculated && travellingFromDirection){
      targetedFinalTheta  = - PI + radians(20) ;
      if (targetedFinalTheta < -PI) targetedFinalTheta = -PI;
      calculateEnergyEquation(theta, dtheta, targetedFinalTheta, x, dx, centerDistance - offsetDistance);
      isMoving = true;
      calculated = true;
    }

    //only change PWM when conditions were met
    if (isMoving && !isSwinging && calculated){
      accelerateTo(final_dx_new, x, x_begin, x_end);
    }

    //stopping motor when travelling from right at offset distance from center
    if(!travellingFromDirection && isMoving && x >= centerDistance + offsetDistance){
      isSwinging = true; 
      isMoving = false;
    }

    //wait for pendulum to to come to peak first at the correct side and theta not above abs(90 deg) before moving 
    if(!travellingFromDirection && !isMoving && isSwinging && theta > 0 && theta <= radians(120) && dtheta <= 0){
      travellingFromDirection = true;
      calculated = false;
      isSwinging = false;
    }

    //stopping motor when travelling from left at offset distance from center
    if(travellingFromDirection && isMoving && x <= centerDistance - offsetDistance){
      isSwinging = true; 
      isMoving = false;
    }

    //wait for pendulum to to come to peak first at the correct side and theta not above abs(90 deg) before moving 
    if(travellingFromDirection && !isMoving && isSwinging && theta < 0 && theta >= -radians(120) && dtheta >= 0){
      travellingFromDirection = false;
      calculated = false;
      isSwinging = false;
    }

    //set flags for shifting into stablizing phase
    if (theta >= PI - radians(2)){
      beginStablization = true;
      targetedFinalTheta = PI;
      tempTargetedFinalTheta = targetedFinalTheta;
      calculated = false;
      isMoving = true;
      isSwinging = false;
    }

    if (theta <= -PI + radians(2)){
      beginStablization = true;
      targetedFinalTheta = -PI;
      tempTargetedFinalTheta = targetedFinalTheta;
      calculated = false;
      isMoving = true;
      isSwinging = false;
    }

  }


  //Maintain upward position stability at center phase
  if (beginStablization){

    // 0 left, 1 right; based on cart position
    int currentSide = (x < centerDistance)? 0: 1;
    double toleranceAngle = radians(0.01);
    double offsetAngle = radians(5) * (x - centerDistance) / (centerDistance );
    targetedFinalTheta= tempTargetedFinalTheta + offsetAngle;

    motorMaxSpeed = 255 * 2.25 * abs(theta - targetedFinalTheta)/radians(10);

    //checking whether pendulum crosses upward position
    if ((theta_old - targetedFinalTheta) * (theta - targetedFinalTheta) <= 0) {
      targetThetaCrossed = true;
    }
    
    else{
      targetThetaCrossed = false;
    }


    // attemp the speed up to maintain stability if unstable
    if (!decelerationPeriod && adjustmentPeriod){

      if (theta < targetedFinalTheta){
        motorPWM = motorMaxSpeed;
        initialPWMbeforeDeceleration = motorMaxSpeed;
      }

      if (theta > targetedFinalTheta){
        motorPWM = -motorMaxSpeed;
        initialPWMbeforeDeceleration = -motorMaxSpeed;
      }

    }


    //case for upward position crossed on the left side
    if (adjustmentPeriod && targetThetaCrossed && currentSide == 0){

      decelerationPeriod = true;
      adjustmentPeriod = false;

      if(theta > targetedFinalTheta){
        x_i = x;
        x_f = tolerable_x_min;
        decelerationPointMarked = true;
      }

      if(theta <= targetedFinalTheta){
        x_i = x;
        x_f = centerDistance;
        decelerationPointMarked = true;
      }

    }

    //decelerate according to the markings
    if (decelerationPeriod && decelerationPointMarked){
      decelerateTo(100 , x, x_i, x_f);
    }


    //stop decelerating and go to adjustment when theta is not maintaining upward position well
    if (decelerationPeriod && abs(theta - targetedFinalTheta) > toleranceAngle){
      decelerationPeriod = false;
      decelerationPointMarked = false;
      adjustmentPeriod = true;
    }

    //case for upward position crossed on the right side
    if (adjustmentPeriod && targetThetaCrossed && currentSide == 1){

      decelerationPeriod = true;
      adjustmentPeriod = false;

      if(theta > targetedFinalTheta){
        x_i = x;
        x_f = tolerable_x_max;
        decelerationPointMarked = true;
      }

      if(theta <= targetedFinalTheta){
        x_i = x;
        x_f = centerDistance;
        decelerationPointMarked = true;
      }

    }
    
    //safety stop
    if (adjustmentPeriod && (x < 0.03 || x > 0.65)){
      isMoving = false;
    }

    //flags for when deceleration is completed
    if (x_f > x_i && x >= x_f){
      adjustmentPeriod = true;
      decelerationPeriod = false;
      decelerationPointMarked = false;
    }

    if (x_f < x_i && x <= x_f){
      adjustmentPeriod = true;
      decelerationPeriod = false;
      decelerationPointMarked = false;
    }

    Serial.println(motorPWM);

    
  }

  // safe gaurd
  if (!isMoving){
    motorPWM = 0; 
  }

  //motor always running technically
  motor(motorPWM);

  //serial print slow down process time too much
  // printLog();
}


void measureNoiseInsensitiveVariables(){
  x = (double)currentDistCount / 50.378 / 100.0;
  theta = (double)currentAngleCount / 2048 * 2 * PI;
  distanceToCenter = centerDistance - x;
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


//Calculation based on initial conditions and final condition
void calculateEnergyEquation(double theta_initial, double dtheta_initial, double theta_final, double x_initial, double dx_initial, double x_final){

  //calculate the initial and final total energy of the system
  hamilton_initial = 0.5 * (m_c + m_p) * dx_initial * dx_initial + 0.5 * m_p * dx_initial * dtheta_initial * l * cos(theta) + 7.0/24.0 * m_p * l * l * dtheta_initial * dtheta_initial + 0.5 * m_p * l * g * (1 - cos(theta_initial)) + 0.5 * m_c * g * l;
  hamilton_final = 0.5 * m_p * g * l * (1-cos(theta_final)) + 0.5 * m_c * g * l;

  //delta_E refer to missing energy from the system
  delta_E = hamilton_final - hamilton_initial;
  
  //calculate new acceleration from conservatoin of work ahd energy theorem
  ddx_new = delta_E / ( (x_final - x_initial) * (m_p + m_c) );
  
  //calculate to final velocity before stopping the cart [ads = vdv]
  final_dx_new = sqrt(abs(2*ddx_new*(x_final - x_initial) + x_initial * x_initial));

  if (ddx_new < 0){
    final_dx_new *= -1;
  }

  x_begin = x_initial;
  x_end =  x_final;
}

//create acceleration by changing veocity on distance
void accelerateTo(double finalVelocity, double x_current, double x_initial, double x_final){
  double PWM_final = abs(finalVelocity) * 176.81 + 65;
  motorPWM = int( PWM_final - PWM_final * abs( (x_final - x_current) / ( x_final - x_initial ) ) + 150);

  //deadzone control
  if (motorPWM > 255){
    motorPWM = 255;
  }
  if (motorPWM < 75){
    motorPWM = 75;
  }

  if (x_final < x_initial){
    motorPWM = - motorPWM;
  }

}

void decelerateTo(double initialPWM, double x_current, double x_initial, double x_final){
  double pwmDeadzone = 75;
  if (initialPWM > 0 && x_final > x_initial){
    motorPWM = (initialPWM - pwmDeadzone) / (x_initial - x_final) * (x_current - x_initial) + initialPWM;
  }

  if (initialPWM < 0 && x_final < x_initial){
    pwmDeadzone *= -1;
    motorPWM = (initialPWM - pwmDeadzone) / (x_initial - x_final) * (x_current - x_initial) + initialPWM;
  }

}

double calculateDecelerationMagnitude(double initialPWM, double x_final, double x_initial ){
  double velocity = ( abs(initialPWM) - 80 ) / 176.81 ;
  double decelerateMagnitude = abs( (pow(velocity,3)) / (3 * (x_final - x_initial) ) );
  return(decelerateMagnitude);
}



//create acceleration and deceleration according to sine wave function
void sineWaveAccelerate(double peakVelocity, double x_current, double x_initial, double x_final) {
  double verticalShift = 250;
  double deadzoneShift = 150;
  double peakPWM = abs(peakVelocity) * 176.81 + 80;
  
  if (x_final > x_initial){
    motorPWM = int( peakPWM * sin ( PI / abs( x_final - x_initial ) * (x_current - x_initial)) ) + verticalShift;
    if (motorPWM > 0 && motorPWM < deadzoneShift){
      motorPWM = deadzoneShift;
    }

    if (x_current > x_final){
      motorPWM = deadzoneShift;
    }
  }

  if (x_final < x_initial){
    motorPWM = int( peakPWM * sin ( PI / abs( x_final - x_initial ) * (x_current - x_initial)) ) - verticalShift;
    if (motorPWM < 0 && motorPWM > -deadzoneShift){
      motorPWM = -deadzoneShift;
    }
  }
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

//print debugging log
void printLog(){
  Serial.print("| x_begin = "); Serial.print(x_begin); Serial.print(" m ");
  Serial.print("| x_end = "); Serial.print(x_end); Serial.print(" m ");
  Serial.print("| extended_end = "); Serial.print(( 2*(x_end - x_begin) + x_begin )); Serial.print(" m ");
  Serial.print("| x = "); Serial.print(x); Serial.print(" m ");
  Serial.print("| dx = "); Serial.print(dx); Serial.print(" m/s ");
  Serial.print("| ddx = "); Serial.print(ddx); Serial.print(" m/s^2 ");
  Serial.print("| Target velocity = "); Serial.print(final_dx_new); Serial.print(" m/s ");
  Serial.print("| Target accel = "); Serial.print(ddx_new); Serial.print(" m/s^2 ");
  Serial.print("| theta = "); Serial.print(theta); Serial.print(" rad ");
  Serial.print("| targetedFinalTheta = ");Serial.print(targetedFinalTheta); Serial.print(" rad ");
  Serial.print("| dtheta = "); Serial.print(dtheta); Serial.print(" rad/s ");
  Serial.print("| ddtheta = "); Serial.print(ddtheta); Serial.print(" rad/s^2 ");
  Serial.print("| PWM = "); Serial.print(motorPWM); Serial.print(" ");
  Serial.print("| isMoving = "); Serial.print((isMoving)? "true":"false");
  Serial.print("| isSwinging = "); Serial.print((isSwinging)? "true":"false");
  Serial.print("| beginStablization = "); Serial.println((beginStablization)? "true":"false");
}
