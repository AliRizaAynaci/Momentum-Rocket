#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

SoftwareSerial serial_gps(0, 1); // RX, TX
TinyGPSPlus gps;
const int gpsBaud = 9600;


typedef struct {
  float lat, lng, satellites; // Enlem, Boylam
  // byte lat[4];
  // byte lng[4];
} Signal;

Signal data;

void setup() {

  Serial.begin(9600);

  serial_gps.begin(gpsBaud);

}


unsigned long start, end, diff;
void loop() {

  start = millis();

  get_gps();

  printParameters();

  end = millis();
  diff = end - start;
  delay(1000 - diff);
}

void get_gps() {

  data.lat = gps.location.lat();
  data.lng = gps.location.lng();
  data.satellites = gps.satellites.value();
  // *(float*)data.lat = gps.location.lat();
  // *(float*)data.lng = gps.location.lng();

  smartDelay(150);
}

void printParameters() {

  Serial.print("Enlem : "); Serial.println(data.lat);
  Serial.print("Boylam : "); Serial.println(data.lng);
  Serial.print("Uydu : "); Serial.println(data.satellites);
  Serial.println("---------------------");
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (serial_gps.available())
      gps.encode(serial_gps.read());

  } while (millis() - start < ms);
}


