#include <PID_v1.h>
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
#define PRINT_CALCULATED
#define PRINT_SPEED 100
  
  double Setpoint, Input1, Input2, Input3, Output1, Output2, Output3;
  PID meinPID1(&Input1, &Output1, &Setpoint,1,0.01,0.01, DIRECT);
  PID myPID2(&Input2, &Output2, &Setpoint,1,0.01,0.01, DIRECT);
  PID benimPID3(&Input3, &Output3, &Setpoint,1,0.01,0.01, DIRECT);

  int motor1 = 3;
  int motor2 = 5;
  int motor3 = 6;
  int motor4 = 9;

void setup() {

  Serial.begin(115200);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  uint16_t status = dof.begin();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  Input1 = dof.calcGyro(dof.gz);
  Setpoint = 0;
  meinPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  benimPID3.SetMode(AUTOMATIC);
}

void loop() {
dof.readGyro();
dof.readAccel();
Input1 =  dof.calcGyro(dof.gz);
  
if(Input1 > 0){
  Input1 = - Input1;
  meinPID1.Compute();
  Output1 = - Output1;
    Serial.print("Input1 = ");
  Serial.print(-Input1);
  Serial.print("Output1 = ");
  Serial.print(Output1);
}
else{
  meinPID1.Compute();
    Serial.print("Input1 = ");
  Serial.print(Input1);
  Serial.print(" Output1 = ");
  Serial.print(Output1);
}

  float x = dof.calcAccel(dof.ax);
  float y = dof.calcAccel(dof.ay);
  float z = dof.calcAccel(dof.az);
  float pitch, roll;
  
  
  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  Input2 = map(pitch, 0,90,0,255);
  Input3 = map(roll,0,90,0,255);
  
  Serial.print(" Pitch = ");
  Serial.print(pitch, 2);
  if(Input2 > 0){
  Input2 = - Input2;
  myPID2.Compute();
  Output2 = - Output2;
  Serial.print("Output2 = ");
  Serial.print(Output2);
}
else{
  myPID2.Compute();
  Serial.print(" Output 2 = ");
  Serial.print(Output2);
}
  Serial.print(" Roll =  ");
  Serial.print(roll, 2);
  
 if(Input3 > 0){
  Input3 = - Input3;
  benimPID3.Compute();
  Output3 = - Output3;
    Serial.print(" Output 3 = ");
  Serial.println(Output3);
}
else{
  benimPID3.Compute();
  Serial.print(" Output 3 = ");
  Serial.println(Output3);
}
  delay(PRINT_SPEED);
}
