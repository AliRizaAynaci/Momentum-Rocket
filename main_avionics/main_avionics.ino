



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

SoftwareSerial serial_gps(3, 4); // RX, TX
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

SoftwareSerial mySerial(6, 7); // RX, TX
LoRa_E22 e22ttl(&mySerial, LORA_AUX, LORA_M0, LORA_M1);

float lat, lon; // GPS data
float altitude, pressure, temp, humidity, first_alt, max_alt = 0; // for BME280 
float accl_y, accl_z, roll, accelScale = 9.81 / 16384.0, gyroScale = 1.0 / 131.0, rad2deg = 180.0 / PI; // for MPU9250
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

bool dragPisOpen = false, mainPisOpen = false;

int counter = 0;

struct Signal {
  int package_id; // Package id
  float temp; // Temperature data
  float altitude; // Altitude
  float pressure; // Pressure
  float humidity; // Humidity
  float gps_lat; // GPS Latitude
  float gps_lng; // GPS Longtitude
  float accl_x; // X axis acceleration
  float accl_y; // Y axis acceleration
  float accl_z; // Z axis acceleration
  float gyro_x; // X axis gyroscope
  float gyro_y; // Y axis gyroscope
  float gyro_z; // Z axis gyroscope
  float roll; // Roll 
} data;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize leds
  pinMode(LED_TX, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_RUN, OUTPUT);
  
  // Initialize BUZZER
  pinMode(BUZZER, OUTPUT);

  // Init BME280
  if(!bme.begin()) {
    Serial.println(F("BME280 connection failed!"));
    // while (1) {
    //   delay(10);
    // }
  } else {
    first_alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
  }

  // Init GPS
  serial_gps.begin(GPSBaud);
  
  // Init IMU
  int statusIMU = IMU.begin();
  if (statusIMU < 0) {
    Serial.println(F("MPU9250 connected failed!"));
    // while (1) {
    //   delay(10);
    // }
  }

  // Init LoRa
  e22ttl_init();
  
  digitalWrite(LED_RUN, HIGH);
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

  get_gps();

  get_IMU();

  increment_counter();

  if (!dragPisOpen) {
    
    if (altitude > max_alt) {
      max_alt = altitude;
    }
    else if (max_alt - altitude > 5 && (roll > 10 || roll < 10)) {
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

// ------ MPU9250 function ----- 
void get_IMU() {
  IMU.readSensor();
  data.accl_x = IMU.getAccelX_mss();
  data.accl_y = IMU.getAccelY_mss();
  data.accl_z = IMU.getAccelZ_mss();
  data.gyro_x = IMU.getGyroX_rads();
  data.gyro_y = IMU.getGyroY_rads();
  data.gyro_z = IMU.getGyroZ_rads();
  roll = (atan2(data.accl_y * accelScale, data.accl_z * accelScale)) * rad2deg;
}

// ------ GPS function ----- 
void get_gps() {
  digitalWrite(LED_RX, HIGH);
  
  data.gps_lat = gps.location.lat();
  data.gps_lng = gps.location.lng();  
  

  digitalWrite(LED_RX, LOW);
  smartdelay(1100 - (SAMPLE_RATE * 200));  
}



/**
  Transmits the data struct over the LoRa network.
*/
void transmit_data() {
  // digitalWrite(LED_TX, HIGH);
  // e22ttl.sendFixedMessage(LORA_ADDL, LORA_ADDH, LORA_CHANNEL, &data, sizeoF(Signal)); // if i know adress and channel
  e22ttl.sendBroadcastFixedMessage(LORA_CHANNEL, &data, sizeof(Signal));
  Serial.print(F("Package ID: ")); Serial.println(data.package_id);
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
  Serial.print(F("Package ID: "));
  Serial.print(data.package_id);
  Serial.print(F(", Sicaklik: "));
  Serial.print(data.temp, 2);
  Serial.print(F(", Irtifa: "));
  Serial.print(data.altitude);
  Serial.print(F(", Basinc: "));
  Serial.print(data.pressure);
  Serial.print(F(", Enlem: "));
  Serial.print(data.gps_lat, 6);
  Serial.print(F(", Boylam: "));
  Serial.print(data.gps_lng, 6);
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