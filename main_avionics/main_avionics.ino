





#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250.h>
#include <SD.h>

#define SEA_LEVEL_PRESSURE 102550

#define LED_TX 5 // LED indicating a packed is transmitted.
#define LED_RX 6 // LED indicating a packed is received.
#define LED_RUN 8 // LED indicating the system is running.

#define LORA_AUX 37
#define LORA_M0 35
#define LORA_M1 36

#define BUZZER 9

#define LORA_CHANNEL 0x12 // LoRa communication channel

#define SAMPLE_RATE 5

#define DRAG_P 44
#define MAIN_P 46

File file; // SD CARD
Adafruit_BME280 bme;
MPU9250 IMU(Wire, 0x68);

SoftwareSerial portgps(14, 15);
TinyGPSPlus gps;

SoftwareSerial mySerial(2, 3); // 
LoRa_E22 e22ttl(&mySerial, LORA_AUX, LORA_M0, LORA_M1);

float lat, lon; // GPS data
float altitude, pressure, temp, humidity, firt_alt, max_alt = 0; // for BME280 
float roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI; // for MPU9250
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

bool dragPisOpen = false, mainPisOpen = false;

struct Data {
  int package_id; // Package id
  byte temp[4]; // Temperature data
  byte altitude[4]; // Altitude
  byte pressure[4]; // Pressure
  byte humidity[4]; // Humidity
  byte gps_lat[4]; // GPS Latitude
  byte gps_long[4]; // GPS Longtitude
  byte accl_x[4]; // X axis acceleration
  byte accl_y[4]; // Y axis acceleration
  byte accl_z[4]; // Z axis acceleration
  byte gyro_x[4]; // X axis gyroscope
  byte gyro_y[4]; // Y axis gyroscope
  byte gyro_z[4]; // Z axis gyroscope
  byte roll[4]; // Roll 
} data;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize leds
  pinMode(LED_TX, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  digitalWrite(LED_RUN, HIGH);
  
  // Initialize BUZZER
  pinMode(BUZZER, OUTPUT);

  // Init BME280
  if(!bme.begin()) {
    Serial.println(F("BME280 connection failed!"));
    while (1) {
      delay(10);
    }
  } else {
    firt_alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
  }

  // Init GPS
  portgps.begin(9600);
  
  // Init IMU
  int statusIMU = IMU.begin();
  if (statusIMU < 0) {
    Serial.println(F("MPU9250 connected failed!"));
    while (1) {
      delay(10);
    }
  }

  // Init LoRa
  e22ttl_init();
}

void e22ttl_init() {
  e22ttl.begin();
  ResponseStructContainer c;
	c = e22ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;

  if (c.status.code == 1) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER, HIGH);
      delay(50);
      digitalWrite(BUZZER, LOW);
      delay(50);
    }
  }
}

void loop() {
  get_altitude();

  get_IMU();

  if (!dragPisOpen) {
    
    if (altitude > max_alt) {
      max_alt = altitude;
    }
    else if (max_alt - altitude > 15 && (roll > 10 || roll < 10)) {
      drag_parachute();
    }
  }
  else if (!mainPisOpen) {
    if (altitude < 600) {
      main_parachute();
    }
  }

}

void drag_parachute() {
  Serial.println(F("Suruklenme Parasutu Acildi."));
  digitalWrite(DRAG_P, HIGH);
  dragPisOpen = true;
}

void main_parachute() {
  Serial.println(F("Ana Parasut Acildi."));
  digitalWrite(MAIN_P, HIGH);
  mainPisOpen = true;
}

// ------ BME280 functions ----- 
void get_altitude() {
  altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
  altitude = altitude - firt_alt;
  altitude = kalman_filter(altitude);
  *(float*)(data.altitude) = altitude;
}

void get_pressure() {
  pressure = bme.readPressure();
  *(float*)(data.pressure) = pressure;
}

void get_temp() {
  temp = bme.readTemperature();
  *(float*)(data.temp) = temp;
}

void get_humidity() {
  humidity = bme.readHumidity();
  *(float*)(data.humidity) = humidity;
}
// -----------------------------


// ------ MPU9250 function ----- 
void get_IMU() {
  IMU.readSensor();
  *(float*)(data.accl_x) = IMU.getAccelX_mss();
  *(float*)(data.accl_y) = IMU.getAccelY_mss();
  *(float*)(data.accl_z) = IMU.getAccelZ_mss();
  *(float*)(data.gyro_x) = IMU.getGyroX_rads();
  *(float*)(data.gyro_y) = IMU.getGyroY_rads();
  *(float*)(data.gyro_z) = IMU.getGyroZ_rads();
 
  // double accl_y = byteArrayToDouble(data.accl_y);
  // double accl_z = byteArrayToDouble(data.accl_z);

  // roll = (atan2(accl_y * accelScale, accl_z * accelScale)) * rad2deg;
}

// ------ GPS function ----- 
void get_gps() {
  digitalWrite(LED_RX, HIGH);
  
  digitalWrite(LED_RX, LOW);
  smartdelay(1100 - (SAMPLE_RATE * 200));  
}

// double byteArrayToDouble(const byte* byteArray) {
//     // Assuming little-endian byte order
//     uint32_t intValue = (byteArray[0] << 0) | (byteArray[1] << 8) | (byteArray[2] << 16) | (byteArray[3] << 24);
//     double result;
//     memcpy(&result, &intValue, sizeof(result));

//     return result;
// }


/*
  Transmits the data struct over the LoRa network.
*/
void transmit_data() {
  digitalWrite(LED_TX, HIGH);
  
}





/**
  Smart delay function for GPS.

  @param ms required milliseconds.
*/
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (portgps.available())
      gps.encode(portgps.read());
  } while (millis() - start < ms);
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









