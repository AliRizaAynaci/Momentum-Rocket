#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEA_LEVEL_PRESSURE 102550

Adafruit_BME280 bme; 

float altitude, pressure, temp, humidity, first_alt, max_alt = 0; // for BME280 
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0, kalman_old = 0 , cov_old = 0; // for Kalman Filter

bool dragPisOpen = false, mainPisOpen = false;

typedef struct {
  float altitude, pressure, temp, humidity;
} Signal;
Signal data;

void setup() {

  Serial.begin(9600);
  Wire.begin();

  if(!bme.begin()) {
    Serial.println("bme280 calismadi...!")
    while(1) {
      delay(10);
    }
  }
  else {
    first_alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
  }
}

unsigned long startMs, endMs, diffMs;
void loop() {

  start = millis();

  get_altitude();
  get_pressure();
  get_temp();
  get_humidity();

  if (!dragPisOpen) {

    if (altitude > max_alt) {
      max_alt = altitude;
      Serial.print("Max Altitude : "); Serial.println(max_alt);
    }
    else if (max_alt > altitude) {
      Serial.println("Alcalma Basladi...")
      drag_parachute();
    }
    
  }
  else if (!mainPisOpen) {
    if (altitude < 600) {
      main_parachute();
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

void main_parachute() {
  Serial.println(F("Ana Parasut Acildi."));
  mainPisOpen = true;
}

// ------ BME280 functions ----- 
void get_altitude() {
  altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
  altitude = altitude - first_alt;
  altitude = kalman_filter(altitude);
  data.altitude = altitude
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

void printParameters() {

  Serial.print("Altitude : "); Serial.print(altitude); Serial.print("Max Altitude : "); Serial.println(max_alt); 
  Serial.print("Pressure : "); Serial.println(pressure);
  Serial.print("Temp : "); Serial.println(temp);
  Serial.print("Humidity : "); Serial.println(humidity);

  // Serial.print("Altitude : "); Serial.print(data.altitude); Serial.print("Max Altitude : "); Serial.println(max_alt); 
  // Serial.print("Pressure : "); Serial.println(data.pressure);
  // Serial.print("Temp : "); Serial.println(data.pressure);
  // Serial.print("Humidity : "); Serial.println(data.humidity);
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