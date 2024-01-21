#include "Arduino.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU9250.h>

MPU9250 IMU(Wire, 0x68);

// ---- MPU9250 ----
float roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI;

// ---- Kalman Filter ---- 
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

// Control Variable
bool dragPisOn = false;

typedef struct {
  float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, aci;
} Signal;

Signal data;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  int statusIMU = IMU.begin();
  if (statusIMU < 0) {
    Serial.println("MPU9250 connected failed...!");
    while (1) {
      delay(10);
    }
  }

}

unsigned long startMs, endMs, diffMs;
void loop() {

  startMs = millis();

  get_IMU();

  if (!dragPisOn) {
    if (data.acc_z < 0 || (roll > 10 || roll < 10)) {
      drag_parachute();
    }
  }

  endMs = millis();
  diffMs = endMs - startMs;
  delay(200 - diffMs);

  printParameters();
  
}

void drag_parachute() {
  Serial.println(F("Suruklenme Parasutu Acildi."));
  dragPisOpen = true;
}

void get_IMU() {
  IMU.readSensor();
  data.acc_x = IMU.getAccelX_mss();
  data.acc_y = IMU.getAccelY_mss();
  data.acc_z = IMU.getAccelZ_mss();
  data.gyro_x = IMU.getGyroX_rads();
  data.gyro_y = IMU.getGyroY_rads();
  data.gyro_z = IMU.getGyroZ_rads();
  roll = (atan2(data.acc_y * accelScale, data.acc_z * accelScale)) * rad2deg;
  data.aci = roll;
}

void printParameters() {

  Serial.print("AccX : "); Serial.println(data.acc_x);
  Serial.print("AccY : "); Serial.println(data.acc_y);
  Serial.print("AccZ : "); Serial.println(data.acc_z);
  Serial.print("GyroX : "); Serial.println(data.gyro_x);
  Serial.print("GyroY : "); Serial.println(data.gyro_y);
  Serial.print("GyroZ : "); Serial.println(data.gyro_z);
}

float kalman_filter(float input) {
 
  kalman_new = kalman_old; 
  cov_new = cov_old + 0.50;
  kalman_gain = cov_new / (cov_new + 0.9);

  kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new));

  cov_new = (1 - kalman_gain) * cov_old;
  cov_old = cov_new;

  kalman_old = kalman_calculated;
  return kalman_calculated;
}

