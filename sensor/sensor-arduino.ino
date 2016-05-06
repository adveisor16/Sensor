#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

String myData;

// ic2
// Initialize the Adafruit_LSM9DS0 object
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(); // uses i2c



void setupSensor() {
  // Setup the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  // Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}


void setup() {
#ifndef ESP8266
  while (!Serial);
#endif
  Serial.begin(9600);
  Serial.println("LSM Demo");

  if (!lsm.begin()) {
    Serial.println("No LSM9DS0 can be detected.");
    while (1);
  }
  Serial.println("LSM9DS0 found");
  Serial.println("");
  Serial.println("");
}

void loop() {
  // Read the sensor data.
  lsm.read();

  myData = packData((int) lsm.accelData.x, (int) lsm.accelData.y, (int) lsm.accelData.z);
  Serial.println(myData);
  delay(80);
}

String packData(int accel_x, int accel_y, int accel_z) {
  String sX = String(accel_x);
  String sY = String(accel_y);
  String sZ = String(accel_z);
  String data = sX + String(",") + sY + String(",") + sZ;
  return data;
}

