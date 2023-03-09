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
#include "Adafruit_TCS34725.h" // capteur de luminosite

#include "ClosedCube_HDC1080.h"

#include "msg_ESP.h"


/*----------MACRO----------*/


// Uncomment the type of sensor in use:
// #define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)



/*----------PINS----------*/


static const uint32_t pinPluvioAnalog = A4;
static const uint32_t pinPluvioGPIO = D2;
static const int RXPin = D4, TXPin = D7; // PIN Liaison série GPS
static const int wake_ccs = D5;
static const int dht_pin = D3; // Digital pin connected to the DHT sensor
static const int anemo_phase_1 = A1, anemo_phase_2 = A2;
static const int capteur_de_foudre = A3;

static const int alimentation_gps = D8;
static const int alimentation_dht = D9;
static const int alimentation_skytemp = D10;
static const int alimentation_pluvio = D11;
static const int alimentation_lumi = D12;
static const int alimentation_ESP = D13;
static const int alimentation_anemo = PE10;

/*----------PARAMS----------*/

float seuil_haut = 815.0; // pas de pluie
float seuil_bas = 425.0;  // beaucoup de pluie

static const uint32_t GPSBaud = 9600;    // BAUD GPS

/*----------VAR----------*/

  // String receive_string;
  // String send_string;

/*----------STRUCT ET CLASSES----------*/

TinyGPSPlus gps;                 // GPS
SoftwareSerial ss(RXPin, TXPin); // Liaison série vers GPS

DHT_Unified capteurTempHum(dht_pin, DHTTYPE);

Adafruit_TCS34725 capteurLum = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

Adafruit_VEML6070 capteurUV = Adafruit_VEML6070();       // une résistance de RSET=270kOhms est soudée sur le capteur
Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); // par défaut addr=0x5A

// Capteur qualité de l'air :
Adafruit_CCS811 ccs;        // CO2, TOV,
Adafruit_BMP280 bmp280;     // Temperature et Pression                // I2C
ClosedCube_HDC1080 hdc1080; // Temperature et Humidité

HardwareSerial Serial6(D0, D1); // Liaison série vers ESP
TwoWire Wire2(PB11, PB10);
msg_ESP_class aenvoyer(&Serial6);

/*----------PROTOTYPES----------*/

void smartDelay(unsigned long ms);
void dateTimePrint(TinyGPSDate &d, TinyGPSTime &t);
void printCapteurs();

/*----------CODE----------*/

void setup()
{

  Serial.begin(9600);    // Liaison serie vers Ordinateur
  Serial6.begin(9600); // Liaison serie ESP



  aenvoyer._msg_location.lat =2.0;
  aenvoyer._msg_location.lng =3.0;

  aenvoyer.updateCrc_gps();
  safePrintSerialln("CRC1:");
  safePrintSerialln(aenvoyer._msg_location.crc);
  aenvoyer.updateCrc_gps();
  safePrintSerialln("CRC2");
  safePrintSerialln(aenvoyer._msg_location.crc);
  safePrintSerialln(aenvoyer.iscrcOk(msg_type::gps_msg));
  
  if(aenvoyer.safeSendX1(msg_type::gps_msg))
  {
    safePrintSerialln("Successfully sent GPS");
  }
  else
  {
    safePrintSerialln("failed to sent GPS");
  }

  aenvoyer._msg_sensor.uv_index_level = 1;
  aenvoyer._msg_sensor.tvoc_index = 1;
  aenvoyer._msg_sensor.temperature_celsius_hdc = 30.0;
  aenvoyer._msg_sensor.temperature_celsius_bmp = 30.0;
  aenvoyer._msg_sensor.temp_object_celsius_sky = 20.0;
  aenvoyer._msg_sensor.temp_ambiant_celsius_sky = 25.0;
  aenvoyer._msg_sensor.pression_Pa_bmp = 5.0;
  aenvoyer._msg_sensor.pluie_pourcentage = 0.3;
  aenvoyer._msg_sensor.pluie_gpio = 1;
  aenvoyer._msg_sensor.humidite_relative_hdc = 0.4;
  aenvoyer._msg_sensor.dht_temp_celsius = 30.0;
  aenvoyer._msg_sensor.dht_humidite_relative = 0.5;
  aenvoyer._msg_sensor.co2_ppm = 30;
  aenvoyer._msg_sensor.lux = 37;
  aenvoyer._msg_sensor.crc = 0xffff;
  aenvoyer.updateCrc_sensor();
  safePrintSerialln("CRC1:");
  safePrintSerialln(aenvoyer._msg_sensor.crc);
  aenvoyer.updateCrc_sensor();
  safePrintSerialln("CRC2");
  safePrintSerialln(aenvoyer._msg_sensor.crc);
  safePrintSerialln(aenvoyer.iscrcOk(msg_type::gps_msg));

}

void loop()
{



  aenvoyer.updateCrc_sensor();

  
  digitalWrite(alimentation_ESP,HIGH);
  if(aenvoyer.safeSendX1(msg_type::sensor_msg))
  {
    safePrintSerialln("Successfully sent");
  }
  else
  {
    safePrintSerialln("failed to sent");
  }
  digitalWrite(alimentation_ESP,LOW);


  delay(5000);
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



void printCapteurs()
{
  dateTimePrint(gps.date,gps.time);
  safePrintSerial(gps.location.lat());
  safePrintSerial(F(","));
  safePrintSerialln(gps.location.lng());
  safePrintSerial("UV light level: ");
  safePrintSerialln(aenvoyer._msg_sensor.uv_index_level);
  safePrintSerial(F("Temperature: "));
  safePrintSerial(aenvoyer._msg_sensor.dht_temp_celsius);
  safePrintSerialln(F("°C"));
  safePrintSerial(F("Humidity: "));
  safePrintSerial(aenvoyer._msg_sensor.dht_humidite_relative);
  safePrintSerialln(F("%"));
  safePrintSerial("Temperature Objet:"); // celle à utiliser avec capteur pointé vers le ciel
  safePrintSerialln(aenvoyer._msg_sensor.temp_object_celsius_sky);
  safePrintSerial("Temperature Ambiente:");
  safePrintSerialln(aenvoyer._msg_sensor.temp_ambiant_celsius_sky);
  safePrintSerial(" GPIO : ");
  safePrintSerial(!aenvoyer._msg_sensor.pluie_gpio); // read GPIO at 1 when no rain, 0 when rain
  safePrintSerial(" rain % : ");
  safePrintSerialln(aenvoyer._msg_sensor.pluie_pourcentage);
  safePrintSerial("Pressure = ");
  safePrintSerial(aenvoyer._msg_sensor.pression_Pa_bmp);
  safePrintSerialln(" Pa, ");
  safePrintSerial("BMP280 => Temperature = ");
  safePrintSerial(aenvoyer._msg_sensor.temperature_celsius_bmp);
  safePrintSerial(" °C, ");
  safePrintSerial("T=");
  safePrintSerial(aenvoyer._msg_sensor.temperature_celsius_hdc);
  safePrintSerial("C, RH=");
  safePrintSerial(aenvoyer._msg_sensor.humidite_relative_hdc);
  safePrintSerialln("%");
  safePrintSerial("CO2: ");
  safePrintSerial(aenvoyer._msg_sensor.co2_ppm);
  safePrintSerial("ppm, TVOC: ");
  safePrintSerialln(aenvoyer._msg_sensor.tvoc_index);
  safePrintSerial("Lux: "); 
  safePrintSerialln(aenvoyer._msg_sensor.lux);
}