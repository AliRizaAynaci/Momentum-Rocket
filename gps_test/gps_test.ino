#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

SoftwareSerial serial_gps(3, 4); // RX, TX
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;


typedef struct {
  float lat, lng, satellites; // Enlem, Boylam
  // byte lat[4];
  // byte lng[4];
} Signal;

Signal data;

void setup() {

  Serial.begin(9600);

  serial_gps.begin(GPSBaud);

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

  Serial.print(F("Location : "));
  Serial.print(data.lat, 6);
  Serial.print(",");
  Serial.print(data.lng, 6);
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


