#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <PID_v1.h>
#include <AFMotor.h>

#define LSM9DS0_G 0x6B
#define LSM9DS0_XM 0x1D

#define dt 0.01 // for complementary filter
#define WAIT 700


/* Create and initialize the SFE_LSM9DS0 instance */
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

/* Initialize the motor objects */
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

unsigned long previousMillis = 0;
unsigned long currentMillis;

float Accel_X, Accel_Y, Accel_Z;
float Gyro_X, Gyro_Y, Gyro_Z;
float x, y, z;

float PitchAccel, RollAccel;
float filterX, filterY;

/* Declare PID controller */
double Setpoint_X, Input_X, Output_X;
double Setpoint_Y, Input_Y, Output_Y;

float biasGyroX = 0;
float biasGyroY = 0;

float biasAccelX = 0;
float biasAccelY = 0;

float biasPitchAccel = 0;
float biasRollAccel = 0;

double Kp = 3.5; // Kp<5 bei MotorGain=2
double Ki = 0.5;  
double Kd = 0.4;

PID myPID_X(&Input_X, &Output_X, &Setpoint_X, Kp, Ki, Kd, DIRECT);
PID myPID_Y(&Input_Y, &Output_Y, &Setpoint_Y, Kp, Ki, Kd, DIRECT);

int SampleTime = 10;
const int MIN = -255;
const int MAX = 255;

int motorGain = 2;



/**
 * 
 */
void setup() {
  /* Initialize Setpoint */
  Setpoint_X = 0;
  Setpoint_Y = 0;

  myPID_X.SetMode(AUTOMATIC);
  myPID_X.SetOutputLimits(MIN, MAX);
  myPID_X.SetSampleTime(SampleTime);

  myPID_Y.SetMode(AUTOMATIC);
  myPID_Y.SetOutputLimits(MIN, MAX);
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
  int speedX = 0;
  int speedY = 0;  
  /* Read gyroscope data */
  dof.readGyro();
  Gyro_X = dof.calcGyro(dof.gx);
  Gyro_Y = dof.calcGyro(dof.gy);
  Gyro_Z = dof.calcGyro(dof.gz);

  /* Read acceleration data */
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
  Serial.print(packData(Gyro_X, Gyro_Y, Gyro_Z));

  Serial.print("XM: ");
  Serial.println(packData(Accel_X, Accel_Y, Accel_Z));

  x = Accel_X;
  y = Accel_Y;
  z = Accel_Z;

  RollAccel = atan2(x, sqrt(y*y + z*z)) * 180.0 / PI;
  PitchAccel = atan2(y, sqrt(x*x + z*z)) * 180.0 / PI;

  if ( abs(RollAccel) <= biasRollAccel )
    RollAccel = 0;
  if ( abs(PitchAccel) <= biasPitchAccel )
    PitchAccel = 0;
  /*
  Serial.print("Roll: ");
  Serial.print(RollAccel);
  Serial.print(" || Pitch: ");
  Serial.println(PitchAccel);
  */

  currentMillis = millis();
  if ( currentMillis - previousMillis >= SampleTime ) {
    previousMillis = currentMillis;

    // Complementary filter
    filterX = 0.98 * (filterX + Gyro_X * dt) + 0.02 * RollAccel;
    filterY = 0.98 * (filterY + Gyro_Y * dt) + 0.02 * PitchAccel;

    /*
    Serial.print("Input_X: ");
    Serial.println(Input_X);

    Serial.print("Input_Y: ");
    Serial.println(Input_Y);
    */
    
    // PID controller
    Input_X = filterX;
    myPID_X.Compute();

    Input_Y = filterY;
    myPID_Y.Compute();

    float sqrt2 = sqrt(2)/2;
    speedX = (int) (sqrt2 * Output_X + sqrt2 * Output_Y);
    speedY = (int) (-sqrt2 * Output_X + sqrt2 * Output_Y);

    byte forwardX = ( speedX >= 0 ) ? 1 : 0;
    byte forwardY = ( speedY >= 0 ) ? 1 : 0;    
    
    //speedX = (int) abs(Output_X);
    speedX *= motorGain;
    speedX = map(abs(speedX), 0, 255, 0, 255);
    speedX = constrain(speedX, 0, 255);

    //speedY = (int) abs(Output_Y);
    speedY *= motorGain;
    speedY = map(abs(speedY), 0, 255, 0, 255);
    speedY = constrain(speedY, 0, 255);

    Serial.print("Output_X: ");
    Serial.println(Output_X);

    Serial.print("Output_Y: ");
    Serial.println(Output_Y);

    // 1,3 : Output_Y
    // 2,4 : Output_X

    motor1.setSpeed(speedY);
    motor2.setSpeed(speedX);
    motor3.setSpeed(speedY);
    motor4.setSpeed(speedX);
    
    if ( forwardY > 0 ) { 
      motor1.run(FORWARD);
      motor3.run(FORWARD);
    } else {
      motor1.run(BACKWARD);
      motor3.run(BACKWARD);
    }

    if ( forwardX == 0 ) { 
      motor2.run(FORWARD);
      motor4.run(FORWARD);
    } else {
      motor2.run(BACKWARD);
      motor4.run(BACKWARD);
    }
    
    /*
    if ( Output_X < 0 ) {
      motor1.setSpeed(speedX);
      motor2.setSpeed(speedX);
      motor3.setSpeed(speedX);
      motor4.setSpeed(speedX);
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    } else if ( Output_X > 0 ) {
      motor1.setSpeed(speedX);
      motor2.setSpeed(speedX);
      motor3.setSpeed(speedX);
      motor4.setSpeed(speedX);
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
     motor4.run(FORWARD);
    } else {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }

    if ( Output_Y < 0 ) {
      motor1.setSpeed(speedY);
      motor2.setSpeed(speedY);
      motor3.setSpeed(speedY);
      motor4.setSpeed(speedY);
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    } else if ( Output_Y > 0 ) {
      motor1.setSpeed(speedY);
      motor2.setSpeed(speedY);
      motor3.setSpeed(speedY);
      motor4.setSpeed(speedY);
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    } else {
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
    }*/
  }

  //delay(WAIT);
}


/**
 * Helper functions
 */
String packData(float dx, float dy, float dz) {
  return String(dx) + String(", ")
         + String(dy) + String(", ")
         + String(dz);
}
