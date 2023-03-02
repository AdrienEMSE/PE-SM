#include <Wire.h>
#include "Adafruit_VEML6070.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor
#include "ClosedCube_HDC1080.h"


#include <Wire.h>                       // This library allows you to communicate with I2C


#define DHTPIN 2     // Digital pin connected to the DHT sensor 

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified capteurTempHum(DHTPIN, DHTTYPE);

uint32_t delayMS;


Adafruit_VEML6070 capteurUV = Adafruit_VEML6070(); //une résistance de RSET=270kOhms est soudée sur le capteur
Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); //par défaut addr=0x5A

Adafruit_CCS811 ccs;
Adafruit_BMP280 bmp280;                // I2C
ClosedCube_HDC1080 hdc1080;

uint32_t pinPluvioAnalog = A0;
uint32_t pinPluvioGPIO = D7;
float seuil_haut = 815.0; // no rain
float seuil_bas = 425.0; // full rain

void printSerialNumber() {
	Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}

void setup() {

  Serial.begin(9600);


  while (!Serial);

  Serial.println("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);  // clear ACK's et écrit 0x06 dans le registre de commande conformement au mode d'emploi

  capteurTempHum.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  delayMS = 2000; //2s entre chaque mesure

  Serial.println("Adafruit MLX90614 Emissivity Setter.\n");

  // init sensor
  if (!capteurTempCiel.begin()) //il faut re-init si on coupe l'alimentation au capteur (en premiere approche)
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  pinMode(pinPluvioAnalog,INPUT_ANALOG);
  pinMode(pinPluvioGPIO,INPUT_PULLDOWN);




	Serial.println("ClosedCube HDC1080 Arduino Test");

	// Default settings: 
	//  - Heater off
	//  - 14 bit Temperature and Humidity Measurement Resolutions
	hdc1080.begin(0x40);

	Serial.print("Manufacturer ID=0x");
	Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	Serial.print("Device ID=0x");
	Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
	
	printSerialNumber();
 

  
}


void loop() {
  
  Serial.print("UV light level: "); Serial.println(capteurUV.readUV()); //pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true); //diminue la conso à 1 microA
  delay(1000);
  capteurUV.sleep(false); //pas besoin de réinitialiser
  }

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

  Serial.print("Temperature Objet:"); //celle à utiliser avec capteur pointé vers le ciel
  Serial.println(capteurTempCiel.readObjectTempC());
  Serial.print("Temperature Ambiente:");
  Serial.println(capteurTempCiel.readAmbientTempC());
  delay(1000);

  uint32_t rain_read = analogRead(pinPluvioAnalog);
  int rain_GPIO = digitalRead(pinPluvioGPIO);
  float rainPercent = (seuil_haut/(seuil_haut-seuil_bas)) -((float)rain_read)/(seuil_haut-seuil_bas);
  if(rainPercent > 1.0)
    rainPercent = 1.0;
  else if (rainPercent < 0.0)
    rainPercent = 0.0;
  Serial.print("rain_raw :");
  Serial.print(rain_read);
  Serial.print(" GPIO : ");
  Serial.print(!rain_GPIO); // read GPIO at 1 when no rain, 0 when rain
  Serial.print(" rain % : ");
  Serial.println( rainPercent );

  delay(1000);
  

  Serial.print("Pressure = ");
  Serial.print(bmp280.readPressure() / 100);
  Serial.println(" Pa, ");

	Serial.print("T=");
	Serial.print(hdc1080.readTemperature());
	Serial.print("C, RH=");
	Serial.print(hdc1080.readHumidity());
	Serial.println("%");



  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }
  delay(5000);
}

