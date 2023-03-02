// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified capteurTempHum(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {
  Serial.begin(9600);
  capteurTempHum.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  delayMS = 2000; //2s entre chaque mesure
}

void loop() {
  //attention fonctionnement de la librairie basé sur HAL_GetTick (parce que protocole de communication non conventionnel)
  //en particulier pour vérifier que 2s se sont écoulées depuis le dernier échantillonnage
  //donc prudence si on désactive les ticks si on passe en mode économie d'énergie le microcontrôleur

 
  // Get temperature event and print its value.
  sensors_event_t event;
  capteurTempHum.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  capteurTempHum.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  delay(delayMS);
}