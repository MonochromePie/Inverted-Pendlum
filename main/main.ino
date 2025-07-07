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
double accelerationInterval = 200;
double lastAccelerationTime;
//Constants
double m_c = 0.2;
double m_p = 0.03;
double l = 0.3;
double g = 9.81;
double x_max = 0.67;
double centerDistance = x_max / 2; 
double offsetDistance = 0.05;
double stableOffsetDistance = 0.3;
double targetedFinalDistance;

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
double x_begin;
double x_end;
double x_end_decel;

//Motor Control
int motorPWM;
int motorMaxSpeed = 255;
int initialPWMbeforeDecelleration;
bool isMoving = false;
bool isSwinging = false;
bool calculated = false;
bool beginStablization = false;


//boolean for movement control
bool travellingFromDirection = false; //false for travelling from right side
bool returnToCenter = true;
bool moveToStable = false;
bool travelFurther = false;
bool safegaurd = false;

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
      targetedFinalTheta  = PI;
      if (targetedFinalTheta > PI) targetedFinalTheta = PI;
      calculateEnergyEquation(theta, dtheta, targetedFinalTheta, x, dx, centerDistance + offsetDistance);
      isMoving = true;
      calculated = true;
    }

    if (!isMoving && !isSwinging && !calculated && travellingFromDirection){
      targetedFinalTheta  = - PI ;
      if (targetedFinalTheta < -PI) targetedFinalTheta = -PI;
      calculateEnergyEquation(theta, dtheta, targetedFinalTheta, x, dx, centerDistance - offsetDistance);
      isMoving = true;
      calculated = true;
    }

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
    if (theta >= PI){
      beginStablization = true;
      targetedFinalTheta = PI;
      calculated = false;
      isMoving = true;
      isSwinging = false;
      returnToCenter = true;
    }

    if (theta <= -PI){
      beginStablization = true;
      targetedFinalTheta = -PI;
      calculated = false;
      isMoving = true;
      isSwinging = false;
      returnToCenter = true;
    }

    if(!isMoving){
      motorPWM = 0;
    }

    motor(motorPWM);

  }

  //Maintain upward position stability phase
  if (beginStablization){

    // 0 left, 1 right; based on cart position
    int currentSide = (x < centerDistance)? 0: 1;
    bool overstated = false;
  }












  
  // safe gaurd
  if (!isMoving || safegaurd){
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
  // x_end_decel = 10 * (x_end - x_begin) + x_begin;
}

//create acceleration by changing veocity on distance; CANNOT DECELERATE ONLY ACCELERATE IN ONE DIRECTION AT A TIME
void accelerateTo(double finalVelocity, double x_current, double x_initial, double x_final){
  double PWM_final = abs(finalVelocity) * 176.81 + 80;
  motorPWM = int( PWM_final - PWM_final * abs( (x_final - x_current) / ( x_final - x_initial ) ) + 120);

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

    if (x_current > x_final + 0.2){
      motorPWM = 0;
    }

  }

  if (x_final < x_initial){
    motorPWM = int( peakPWM * sin ( PI / abs( x_final - x_initial ) * (x_current - x_initial)) ) - verticalShift;
    if (motorPWM < 0 && motorPWM > -deadzoneShift){
      motorPWM = -deadzoneShift;
    }

    if (x_current < x_final - 0.2){
      motorPWM = 0;
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
  Serial.print("| beginStablization = "); Serial.print((beginStablization)? "true":"false");
  Serial.print("| returnToCenter = "); Serial.print((returnToCenter)? "true":"false");
  Serial.print("| extended_end = "); Serial.print(( 2*(x_end - x_begin) + x_begin )); Serial.println(" m ");
}








// if (returnToCenter && !calculated && abs(theta-targetedFinalTheta) > radians(1)) {
//       calculateEnergyEquation( (theta > targetedFinalTheta)? theta + radians(30): theta - radians(30) , dtheta, targetedFinalTheta, x, dx, centerDistance);
//       isMoving = true;
//       calculated = true;
//     }
    
//     //stop moving flags when returning to center
//     if (returnToCenter && calculated && isMoving){
//       if (x_begin < centerDistance && x >= centerDistance){
//         calculated = false;
//         returnToCenter = false;
//       }
//       if (x_begin > centerDistance && x <= centerDistance){
//         calculated = false;
//         returnToCenter = false;
//       }
//     }

//     if (isMoving && returnToCenter && calculated){
//       x_end_decel = 2 * (x_end - x_begin) + x_begin;
//       sineWaveAccelerate(final_dx_new, x, x_begin, centerDistance);
//     }

//     if (!returnToCenter && !calculated){
//       calculateEnergyEquation( theta - radians(30), dtheta, targetedFinalTheta, x, dx, x + (l * sin(theta) / 2 ));
//       isMoving = true;
//       calculated = true;
//     }

//     if (isMoving && !returnToCenter && calculated){
//       sineWaveAccelerate(final_dx_new, x, x_begin, x_end);
//     }
    
//     if (isMoving && !returnToCenter && calculated){
//       if ( x_end_decel > x_begin && x >= x_end_decel && motorPWM == 40){
//         isMoving = false;
//         calculated = false;
//         returnToCenter = true;
//     }
//       if ( x_end_decel < x_begin && x <= x_end_decel && motorPWM == -40){
//         isMoving = false;
//         calculated = false;
//         returnToCenter = true;
//       }
//     }

//   }
  
//   if (theta > 2*PI || theta < -2*PI ){
//     safegaurd = true;

