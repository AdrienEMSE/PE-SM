/*----------INCLUDES----------*/


#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <TinyGPSPlus.h> // Lib GPS

#include <SoftwareSerial.h>

#include "Adafruit_VEML6070.h"
#include <Adafruit_Sensor.h>

#include <DHT.h> // LIB Température Humidité
#include <DHT_U.h>

#include <Adafruit_MLX90614.h>
#include <Adafruit_CCS811.h> // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h> // include main library for BMP280 - Sensor

#include "ClosedCube_HDC1080.h"

#include "msg_ESP.h"

/*----------MACRO----------*/

#define DEBUG

#ifdef DEBUG

#define safePrintSerial(x) Serial.print(x)
#define safePrintSerialln(x) Serial.println(x)

#else

#define safePrintSerial(x)
#define safePrintSerialln(x)

#endif

// Uncomment the type of sensor in use:
// #define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)

/*----------PINS----------*/


static const uint32_t pinPluvioAnalog = A4;
static const uint32_t pinPluvioGPIO = D2;
static const int RXPin = D4, TXPin = D7; // PIN Liaison série GPS
static const int sleep_gpio_ccs_Pin = D5;
static const int dht_pin = D3; // Digital pin connected to the DHT sensor
static const int anemo_phase_1 = A1, anemo_phase_2 = A2;
static const int capteur_de_foudre = A3;

/*----------PARAMS----------*/

float seuil_haut = 815.0; // pas de pluie
float seuil_bas = 425.0;  // beaucoup de pluie

static const uint32_t GPSBaud = 9600;    // BAUD GPS

/*----------STRUCT ET CLASSES----------*/

TinyGPSPlus gps;                 // GPS
SoftwareSerial ss(RXPin, TXPin); // Liaison série vers GPS

DHT_Unified capteurTempHum(dht_pin, DHTTYPE);


Adafruit_VEML6070 capteurUV = Adafruit_VEML6070();       // une résistance de RSET=270kOhms est soudée sur le capteur
Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); // par défaut addr=0x5A

// Capteur qualité de l'air :
Adafruit_CCS811 ccs;        // CO2, TOV,
Adafruit_BMP280 bmp280;     // Temperature et Pression                // I2C
ClosedCube_HDC1080 hdc1080; // Temperature et Humidité

HardwareSerial Serial6(D0, D1); // Liaison série vers ESP
TwoWire Wire2(PB11, PB10);
msg_ESP_class aenvoyer;

/*----------PROTOTYPES----------*/

void smartDelay(unsigned long ms);
void dateTimePrint(TinyGPSDate &d, TinyGPSTime &t);
void printCapteurs();

/*----------CODE----------*/

void setup()
{

  Serial.begin(9600);    // Liaison serie vers Ordinateur
  aenvoyer._msg.uv_index_level = 1;
  aenvoyer._msg.tvoc_index = 1;
  aenvoyer._msg.temperature_celsius_hdc = 30.0;
  aenvoyer._msg.temperature_celsius_bmp = 30.0;
  aenvoyer._msg.temp_object_celsius_sky = 20.0;
  aenvoyer._msg.temp_ambiant_celsius_sky = 25.0;
  aenvoyer._msg.pression_Pa_bmp = 5.0;
  aenvoyer._msg.pluie_pourcentage = 0.3;
  aenvoyer._msg.pluie_gpio = 1;
  aenvoyer._msg.humidite_relative_hdc = 0.4;
  aenvoyer._msg.dht_temp_celsius = 30.0;
  aenvoyer._msg.dht_humidite_relative = 0.5;
  aenvoyer._msg.co2_ppm = 30;
  aenvoyer._msg.msg_gps.msg_date.a = 2033;
  aenvoyer._msg.msg_gps.msg_date.j = 7;
  aenvoyer._msg.msg_gps.msg_date.m = 4;
  aenvoyer._msg.msg_gps.msg_location.lat = 30.0;
  aenvoyer._msg.msg_gps.msg_location.lng = 40.0;
  aenvoyer._msg.msg_gps.msg_time.hour = 10;
  aenvoyer._msg.msg_gps.msg_time.min = 50;
  aenvoyer._msg.msg_gps.msg_time.sec = 35;
  aenvoyer._msg.crc = 0xffff;
  uint32_t timer = micros();
  aenvoyer.updateCrc();
  Serial.print("time ellapsed :");
  Serial.println(micros()-timer);
  Serial.print("CRC :");
  Serial.println(aenvoyer._msg.crc,HEX);
  aenvoyer.updateCrc();
  Serial.print("CRC 2 :");
  Serial.println(aenvoyer._msg.crc,HEX);
  aenvoyer._msg.tvoc_index =4;
  aenvoyer.updateCrc();
  Serial.print("CRC 2 :");
  Serial.println(aenvoyer._msg.crc,HEX);

}

void loop()
{



  delay(40000);
}

/*---------------------Fonctions utilitaires----------------------*/



// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  safePrintSerial(sz);
  smartDelay(0);
}

void dateTimePrint(TinyGPSDate &d, TinyGPSTime &t)
{
  char toprint[50];
  if(!d.isValid())
  {

    sprintf(toprint, "********** ");
    safePrintSerial(toprint);
  }
  else
  {
    sprintf(toprint, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    safePrintSerial(toprint);
  }

  if (!t.isValid())
  {
    sprintf(toprint, "********** ");
    safePrintSerial(toprint);
  }
  else
  {
    sprintf(toprint, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    safePrintSerial(toprint);
  }

  smartDelay(0);
}

void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    safePrintSerial(i < slen ? str[i] : ' ');
  smartDelay(0);
}

