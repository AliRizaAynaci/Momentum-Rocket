#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEA_LEVEL_PRESSURE 102550

MPU9250 IMU(Wire, 0x68);
Adafruit_BME280 bme; 

float altitude, pressure, temp, humidity, first_alt, max_alt = 0; // for BME280
float roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI; // for MPU9250
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

bool dragPisOpen = false, mainPisOpen = false;

typedef struct {
  float altitude, pressure, temp, humidity, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, aci;
} Signal;
Signal data;

void setup() {

  Serial.begin(9600);
  Wire.begin();

  int statusBME = bme.begin();
  if(statusBME) {
    Serial.println("bme280 calismadi...!");
    // while(1) {
    //   delay(10);
    // }
  }
  else {
    first_alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
  }

  int statusIMU = IMU.begin();
  if (statusIMU < 0) {
    Serial.println("MPU9250 connected failed...!");
    // while (1) {
    //   delay(10);
    // }
  }


}

unsigned long startMs, endMs, diffMs;
void loop() {

  startMs = millis();

  get_altitude();
  get_pressure();
  get_temp();
  get_humidity();
  get_IMU();

  if(!dragPisOpen) {

    if (altitude > max_alt) {
      max_alt = altitude;
    }
    else if (max_alt > altitude && (data.acc_z < 0 || (roll > 10 || roll < 10))) {
      Serial.println("Alcalma Basladi...");
      delay(50);
      drag_parachute();
    }
  }
  else if (!mainPisOpen) {
    if (altitude < 600) {
      main_parachute();
    }
  }

  printParameters();
  
  endMs = millis();
  diffMs = endMs - startMs;
  delay(200 - diffMs);


}

void drag_parachute() {
  Serial.println(F("Suruklenme Parasutu Acildi."));
  dragPisOpen = true;
}

void main_parachute() {
  Serial.println(F("Ana Parasut Acildi."));
  mainPisOpen = true;
}

// ------ BME280 functions ----- 
void get_altitude() {
  altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
  altitude = altitude - first_alt;
  altitude = kalman_filter(altitude);
  data.altitude = altitude;
}

void get_pressure() {
  pressure = bme.readPressure();
  data.pressure = pressure;
}

void get_temp() {
  temp = bme.readTemperature();
  data.temp = temp;
}

void get_humidity() {
  humidity = bme.readHumidity();
  data.humidity = humidity;
}

// ------ MPU9250 functions -----
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

  Serial.print("Altitude : "); Serial.print(altitude); Serial.print("  -----   Max Altitude : "); Serial.println(max_alt); 
  Serial.print("Pressure : "); Serial.println(pressure);
  Serial.print("Temp : "); Serial.println(temp);
  Serial.print("Humidity : "); Serial.println(humidity);

  // Serial.print("Altitude : "); Serial.print(data.altitude); Serial.print("Max Altitude : "); Serial.println(max_alt); 
  // Serial.print("Pressure : "); Serial.println(data.pressure);
  // Serial.print("Temp : "); Serial.println(data.pressure);
  // Serial.print("Humidity : "); Serial.println(data.humidity);

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



