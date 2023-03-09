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
  Serial6.begin(115200); // Liaison serie ESP
  ss.begin(GPSBaud);     // Liaison serie GPS
  Wire2.begin();         // I2C Skytemp

  //pinMode(pinPluvioAnalog, INPUT_ANALOG);
  pinMode(pinPluvioGPIO, INPUT_PULLDOWN);

  pinMode(wake_ccs,OUTPUT);
  digitalWrite(wake_ccs,HIGH);
  
  pinMode(alimentation_anemo,OUTPUT);
  pinMode(alimentation_dht,OUTPUT);
  pinMode(alimentation_skytemp,OUTPUT);
  pinMode(alimentation_lumi,OUTPUT);
  pinMode(alimentation_ESP,OUTPUT);
  pinMode(alimentation_gps,OUTPUT);
  
  
  digitalWrite(alimentation_anemo,LOW);
  digitalWrite(alimentation_dht,LOW); 
  digitalWrite(alimentation_skytemp,LOW);
  digitalWrite(alimentation_pluvio,LOW);
  digitalWrite(alimentation_lumi,LOW);
  digitalWrite(alimentation_ESP,LOW);
  digitalWrite(alimentation_gps,LOW);//POWER GPS ON (PMOS)
  



  bool gps_ok = false;
  uint32_t timer = millis();
  while(!gps_ok)
  {
    smartDelay(1);
    if(gps.date.day() != 0)
    {
      gps_ok = true;
      safePrintSerialln("Le gps a atteint le satellite");
    }
    if(millis() > timer + 10000)
    {
      timer = millis();
      safePrintSerialln("Le gps n'a toujours pas atteint le GPS...");
    }


  }   
  digitalWrite(alimentation_gps,HIGH); //POWER GPS OFF (PMOS)





  safePrintSerialln("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);

  safePrintSerialln(F("DHTxx Unified Sensor Example"));
  capteurTempHum.begin();

  digitalWrite(alimentation_skytemp,HIGH);
  safePrintSerialln("Adafruit MLX90614 Emissivity Setter.\n");
  if (!capteurTempCiel.begin(0x69, &Wire2)) // il faut re-init si on coupe l'alimentation au capteur (en premiere approche)
  {
    safePrintSerialln("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
  digitalWrite(alimentation_skytemp,LOW);

  digitalWrite(wake_ccs, LOW);
  safePrintSerialln("CCS811 test"); /* --- SETUP CCS811 on 0x5A ------ */
  if (!ccs.begin(0x5B))
  {
    safePrintSerialln("Failed to start sensor! Please check your wiring.");
    while (1);
  }
  while (!ccs.available());
  ccs.setDriveMode(CCS811_DRIVE_MODE_60SEC);
  digitalWrite(wake_ccs, HIGH);

  safePrintSerialln("BMP280 test"); /* --- SETUP BMP on 0x76 ------ */
  if (!bmp280.begin(0x76))
  {
    safePrintSerialln("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

  safePrintSerialln("ClosedCube HDC1080 Arduino Test");
  hdc1080.begin(0x40);

  digitalWrite(alimentation_lumi,HIGH);
  if (capteurLum.begin()) 
  {
    safePrintSerialln("Found sensor");
  } 
  else 
  {
    safePrintSerialln("No TCS34725 found ... check your connections");
    while (1);
  }
  digitalWrite(alimentation_lumi,LOW);


}

void loop()
{

  capteurUV.sleep(false);                       // pas besoin de réinitialiser
  aenvoyer._msg.uv_index_level = capteurUV.readUV(); // pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true);                        // diminue la conso à 1 microA

  // attention fonctionnement de la librairie basé sur HAL_GetTick (parce que protocole de communication non conventionnel)
  // en particulier pour vérifier que 2s se sont écoulées depuis le dernier échantillonnage
  // donc prudence si on désactive les ticks si on passe en mode économie d'énergie le microcontrôleur

  // Get temperature event and print its value.
  digitalWrite(alimentation_dht,HIGH);
  sensors_event_t event;
  capteurTempHum.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    safePrintSerialln(F("Error reading temperature DHT!"));
  }
  else
  {
    aenvoyer._msg.dht_temp_celsius = event.temperature;
  }
  // Get humidity event and print its value.
  capteurTempHum.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    safePrintSerialln(F("Error reading humidity DHT!"));
  }
  else
  {
    aenvoyer._msg.dht_humidite_relative = event.relative_humidity;
  }
  digitalWrite(alimentation_dht,LOW);


  digitalWrite(alimentation_skytemp,HIGH);
  capteurTempCiel.begin(0x69, &Wire2);
  aenvoyer._msg.temp_object_celsius_sky = capteurTempCiel.readObjectTempC();
  aenvoyer._msg.temp_ambiant_celsius_sky = capteurTempCiel.readAmbientTempC();
  digitalWrite(alimentation_skytemp,LOW);


  digitalWrite(alimentation_pluvio,HIGH);
  uint32_t rain_read = analogRead(pinPluvioAnalog);
  int rain_GPIO = digitalRead(pinPluvioGPIO);
  digitalWrite(alimentation_pluvio,LOW);
  float rainPercent = (seuil_haut / (seuil_haut - seuil_bas)) - ((float)rain_read) / (seuil_haut - seuil_bas);
  if (rainPercent > 1.0)
    rainPercent = 1.0;
  else if (rainPercent < 0.0)
    rainPercent = 0.0;
  

  aenvoyer._msg.pluie_gpio = rain_GPIO;
  aenvoyer._msg.pluie_pourcentage = rainPercent;

  aenvoyer._msg.pression_Pa_bmp = bmp280.readPressure();
  aenvoyer._msg.temperature_celsius_bmp = bmp280.readTemperature();

  aenvoyer._msg.temperature_celsius_hdc = hdc1080.readTemperature();
  aenvoyer._msg.humidite_relative_hdc = hdc1080.readHumidity();

  digitalWrite(wake_ccs, LOW);
  if (ccs.available())
  {
    if (!ccs.readData())
    {
      aenvoyer._msg.co2_ppm = ccs.geteCO2();
      aenvoyer._msg.tvoc_index = ccs.getTVOC();
    }
    else
    {
      safePrintSerialln("ERROR air quality CCS!");
      while (1);
    }
  }
  digitalWrite(wake_ccs, HIGH);

  digitalWrite(alimentation_lumi,HIGH);
  capteurLum.enable();
  delay(1000);
  uint16_t r, g, b, c, lux;

  capteurLum.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  aenvoyer._msg.lux = capteurLum.calculateLux(r, g, b);

  capteurLum.disable();
  digitalWrite(alimentation_lumi,LOW);
  


  digitalWrite(alimentation_gps,LOW); //Power on GPS
  smartDelay(2000);//DELAY + permet d'utiliser le GPS
  digitalWrite(alimentation_gps,HIGH);//Power off GPS

  aenvoyer._msg.msg_gps.msg_location.lat = gps.location.lat();
  aenvoyer._msg.msg_gps.msg_location.lng = gps.location.lng();
  aenvoyer._msg.msg_gps.msg_date.a = gps.date.year();
  aenvoyer._msg.msg_gps.msg_date.j = gps.date.day();
  aenvoyer._msg.msg_gps.msg_date.m = gps.date.month();
  aenvoyer._msg.msg_gps.msg_time.hour = gps.time.hour();
  aenvoyer._msg.msg_gps.msg_time.min = gps.time.minute();
  aenvoyer._msg.msg_gps.msg_time.sec = gps.time.second();
  safePrintSerialln();
  

  printCapteurs();


  // aenvoyer._msg.uv_index_level = 1;
  // aenvoyer._msg.tvoc_index = 1;
  // aenvoyer._msg.temperature_celsius_hdc = 30.0;
  // aenvoyer._msg.temperature_celsius_bmp = 30.0;
  // aenvoyer._msg.temp_object_celsius_sky = 20.0;
  // aenvoyer._msg.temp_ambiant_celsius_sky = 25.0;
  // aenvoyer._msg.pression_Pa_bmp = 5.0;
  // aenvoyer._msg.pluie_pourcentage = 0.3;
  // aenvoyer._msg.pluie_gpio = 1;
  // aenvoyer._msg.humidite_relative_hdc = 0.4;
  // aenvoyer._msg.dht_temp_celsius = 30.0;
  // aenvoyer._msg.dht_humidite_relative = 0.5;
  // aenvoyer._msg.co2_ppm = 30;
  // aenvoyer._msg.msg_gps.msg_date.a = 2033;
  // aenvoyer._msg.msg_gps.msg_date.j = 7;
  // aenvoyer._msg.msg_gps.msg_date.m = 4;
  // aenvoyer._msg.msg_gps.msg_location.lat = 30.0;
  // aenvoyer._msg.msg_gps.msg_location.lng = 40.0;
  // aenvoyer._msg.msg_gps.msg_time.hour = 10;
  // aenvoyer._msg.msg_gps.msg_time.min = 50;
  // aenvoyer._msg.msg_gps.msg_time.sec = 35;
  // aenvoyer._msg.crc = 0xffff;
  // aenvoyer.updateCrc();

  
  digitalWrite(alimentation_ESP,HIGH);
  if(aenvoyer.safeSendX1())
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
  safePrintSerialln(aenvoyer._msg.uv_index_level);
  safePrintSerial(F("Temperature: "));
  safePrintSerial(aenvoyer._msg.dht_temp_celsius);
  safePrintSerialln(F("°C"));
  safePrintSerial(F("Humidity: "));
  safePrintSerial(aenvoyer._msg.dht_humidite_relative);
  safePrintSerialln(F("%"));
  safePrintSerial("Temperature Objet:"); // celle à utiliser avec capteur pointé vers le ciel
  safePrintSerialln(aenvoyer._msg.temp_object_celsius_sky);
  safePrintSerial("Temperature Ambiente:");
  safePrintSerialln(aenvoyer._msg.temp_ambiant_celsius_sky);
  safePrintSerial(" GPIO : ");
  safePrintSerial(!aenvoyer._msg.pluie_gpio); // read GPIO at 1 when no rain, 0 when rain
  safePrintSerial(" rain % : ");
  safePrintSerialln(aenvoyer._msg.pluie_pourcentage);
  safePrintSerial("Pressure = ");
  safePrintSerial(aenvoyer._msg.pression_Pa_bmp);
  safePrintSerialln(" Pa, ");
  safePrintSerial("BMP280 => Temperature = ");
  safePrintSerial(aenvoyer._msg.temperature_celsius_bmp);
  safePrintSerial(" °C, ");
  safePrintSerial("T=");
  safePrintSerial(aenvoyer._msg.temperature_celsius_hdc);
  safePrintSerial("C, RH=");
  safePrintSerial(aenvoyer._msg.humidite_relative_hdc);
  safePrintSerialln("%");
  safePrintSerial("CO2: ");
  safePrintSerial(aenvoyer._msg.co2_ppm);
  safePrintSerial("ppm, TVOC: ");
  safePrintSerialln(aenvoyer._msg.tvoc_index);
  safePrintSerial("Lux: "); 
  safePrintSerialln(aenvoyer._msg.lux);
}