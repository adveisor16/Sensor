
#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <PID_v1.h>
#include <AFMotor.h>

#define LSM9DS0_G 0x6B
#define LSM9DS0_XM 0x1D

#define dt 0.01

/* Create and initialize the SFE_LSM9DS0 instance */
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

/* Initialise the motor objects */
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);


unsigned long previousMillis = 0;
unsigned long currentMillis;
int interval = 10;

int motorGain = 2;

float Accel_X, Accel_Y, Accel_Z;
float Gyro_X, Gyro_Y, Gyro_Z;

float x, y, z;
float PitchAccel, RollAccel;
float mixX, mixY;

/* Declare PID regulator */
double Setpoint_X, Input_X, Output_X;
double Setpoint_Y, Input_Y, Output_Y;

float biasGyroX = 3.1;
float biasGyroY = 1.5;

float biasAccelX = 0.1;
float biasAccelY = 0.08;

float biasPitchAccel = 0;
float biasRollAccel = 0;

double Kp = 6;
double Ki = 0.5;
double Kd = 0.4;

PID myPID_X(&Input_X, &Output_X, &Setpoint_X, Kp, Ki, Kd, DIRECT); 
PID myPID_Y(&Input_Y, &Output_Y, &Setpoint_Y, Kp, Ki, Kd, DIRECT);

const int _MIN = -255;
const int _MAX = 255;

int SampleTime = 10;


/**
 * 
 */
void setup() {
  /* Initialize Setpoint */
  Setpoint_X = 0;
  Setpoint_Y = 0;

  myPID_X.SetMode(AUTOMATIC);
  myPID_X.SetOutputLimits(_MIN, _MAX);
  myPID_X.SetSampleTime(SampleTime);

  myPID_Y.SetMode(AUTOMATIC);
  myPID_Y.SetOutputLimits(_MIN, _MAX);
  myPID_Y.SetSampleTime(SampleTime);

  /* Initialize motors */
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  Serial.begin(115200);

  /* Initialize SFE_LSM9DS0 */
  uint16_t status = dof.begin();
}


/**
 * 
 */
void loop() {
  /* Motor speed */
  int speedMotor1 = 0;
  int speedMotor2 = 0;
  int speedMotor3 = 0;
  int speedMotor4 = 0;
  
  dof.readGyro(); // store the new data in gx, gy, gz
  Gyro_X = dof.calcGyro(dof.gx);
  Gyro_Y = dof.calcGyro(dof.gy);
  Gyro_Z = dof.calcGyro(dof.gz);

  dof.readAccel();
  Accel_X = dof.calcAccel(dof.ax);
  Accel_Y = dof.calcAccel(dof.ay);
  Accel_Z = dof.calcAccel(dof.az);

   
  /**
   * Reduce those values to zero. Eliminate any noise.
   */  
  if ( abs(Gyro_X) <= biasGyroX )  Gyro_X = 0;
  if ( abs(Gyro_Y) <= biasGyroY )  Gyro_Y = 0;

  if ( abs(Accel_X) <= biasAccelX )  Accel_X = 0;
  if ( abs(Accel_Y) <= biasAccelY )  Accel_Y = 0;

  
  Serial.print("G: ");
  Serial.print(packData(Gyro_X,Gyro_Y, Gyro_Z));

  Serial.print("XM: ");
  Serial.println(packData(Accel_X, Accel_Y, Accel_Z));
  

  /*
  if ( gyroY > 0 ) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);    
  } else if ( gyroY < 0 ) {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);    
  }
  delay(_DELAY);*/

  x = Accel_X;
  y = Accel_Y;
  z = Accel_Z; 

  PitchAccel = atan2(x, sqrt(y*y + z*z)) * 180.0 / PI;
  RollAccel = atan2(y, sqrt(x*x + z*z)) * 180.0 / PI;

  if ( abs(PitchAccel) <= biasPitchAccel )
    PitchAccel = 0;
  if ( abs(RollAccel) <= biasRollAccel )
    RollAccel = 0;

  /*
  Serial.print("Pitch: ");
  Serial.print(PitchAccel);
  Serial.print(" || Roll: ");
  Serial.println(RollAccel);
  */

  currentMillis = millis();  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //Complementary filter
    mixX = 0.98 *(mixX + Gyro_X * dt) + 0.02 * PitchAccel;
    mixY = 0.98 *(mixY + Gyro_Y * dt) + 0.02 * RollAccel;

    Input_X = mixX;
    myPID_X.Compute();
    speedMotor1 = (int) abs(Output_X);
    speedMotor1 *= motorGain;
    speedMotor1 = map(speedMotor1, 0, 255, 0, 255);
    speedMotor1 = constrain(speedMotor1, 0, 255);
    speedMotor2 = speedMotor1;

    Input_Y = mixY;
    myPID_Y.Compute();
    speedMotor3 = (int) abs(Output_Y);
    speedMotor3 *= motorGain;
    speedMotor3 = map(speedMotor3, 0, 255, 0, 255);
    speedMotor3 = constrain(speedMotor3, 0, 255);
    speedMotor4 = speedMotor3;

    motor1.setSpeed(speedMotor1);
    motor2.setSpeed(speedMotor2);
    motor3.setSpeed(speedMotor3);
    motor4.setSpeed(speedMotor4);
    
    if ( Output_X < 0 ) {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
    } else if ( Output_X > 1 ) {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
    } else {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
    }

    if ( Output_Y < 0 ) {
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    } else if ( Output_Y > 1 ) {
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    } else {
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }
  }
  
  //delay(300);
}

/**
 * Helper functions
 */
String packData(float dx, float dy, float dz) {
  return String(dx) + String(", ")
         + String(dy) + String(", ")
         + String(dz);
}

/**               
 *                 
 * XM_Y :  ------- + ------>
 *         <------ - -------
 */
