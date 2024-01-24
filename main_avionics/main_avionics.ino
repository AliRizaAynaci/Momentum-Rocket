



#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250.h>
#include "LoRa_E22.h"
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
#define LORA_ADDL 21
#define LORA_ADDH 179

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
float altitude, pressure, temp, humidity, first_alt, max_alt = 0; // for BME280 
float accl_y, accl_z, roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI; // for MPU9250
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

bool dragPisOpen = false, mainPisOpen = false;

int counter = 0;

struct Data {
  int package_id; // Package id
  byte temp[4]; // Temperature data
  byte altitude[4]; // Altitude
  byte pressure[4]; // Pressure
  byte humidity[4]; // Humidity
  byte gps_lat[4]; // GPS Latitude
  byte gps_lng[4]; // GPS Longtitude
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
    first_alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
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

unsigned long start, end, diff;
void loop() {

  start = millis();

  get_altitude();

  get_pressure();
  
  get_temp();

  get_humidity();

  // get_gps();

  get_IMU();

  increment_counter();

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

  print_data_to_serial();

  transmit_data();

  end = millis();
  diff = end - start; 
  delay(200 - diff);

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
  altitude = altitude - first_alt;
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

// ------ MPU9250 function ----- 
void get_IMU() {
  IMU.readSensor();
  *(float*)(data.accl_x) = IMU.getAccelX_mss();
  *(float*)(data.accl_y) = IMU.getAccelY_mss();
  *(float*)(data.accl_z) = IMU.getAccelZ_mss();
  *(float*)(data.gyro_x) = IMU.getGyroX_rads();
  *(float*)(data.gyro_y) = IMU.getGyroY_rads();
  *(float*)(data.gyro_z) = IMU.getGyroZ_rads();
  byteArrayToFloat();
  roll = (atan2(accl_y * accelScale, accl_z * accelScale)) * rad2deg;
}

// ------ GPS function ----- 
void get_gps() {
  digitalWrite(LED_RX, HIGH);
  
  *(float*)(data.gps_lat) = gps.location.lat();
  *(float*)(data.gps_lng) = gps.location.lng();  
  

  digitalWrite(LED_RX, LOW);
  smartdelay(1100 - (SAMPLE_RATE * 200));  
}


void byteArrayToFloat() {
  memcpy(&accl_y, data.accl_y, 4);
  memcpy(&accl_z, data.accl_z, 4);
}


/**
  Transmits the data struct over the LoRa network.
*/
void transmit_data() {
  // digitalWrite(LED_TX, HIGH);
  // e22ttl.sendFixedMessage(LORA_ADDL, LORA_ADDH, LORA_CHANNEL, &data, sizeoF(Data)); // if i know adress and channel
  e22ttl.sendBroadcastFixedMessage(LORA_CHANNEL, &data, sizeof(Data));

  Serial.print("Package ID: "); Serial.println(data.package_id);
  delay(50);
  // digitalWrite(LED_TX, LOW);
}

/**
  Increments data package ID from 1 to 255.
  Counter resets to 1 after 255.
*/
void increment_counter() {
  if (((counter + 1) % 256) == 0) {
    counter = 1;
    data.package_id = counter;
  }else {
    counter++;
    data.package_id = counter;
  }
}

/**
  Prints the gathered information from the sensors to Serial monitor.
*/
void print_data_to_serial() {
  Serial.print("Package ID: ");
  Serial.print(data.package_id);
  Serial.print(", Sicaklik: ");
  Serial.print(*(float*)data.temp, 2);
  Serial.print(", Irtifa: ");
  Serial.print(*(float*)data.altitude);
  Serial.print(", Basinc: ");
  Serial.print(*(float*)data.pressure);
  Serial.print(", Enlem: ");
  Serial.print(*(float*)data.gps_lat, 6);
  Serial.print(", Boylam: ");
  Serial.print(*(float*)data.gps_lng, 6);
  Serial.println();
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