#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor
#include "ClosedCube_HDC1080.h"
#include "Adafruit_VEML6070.h" //Lib capteur UV



#include <Wire.h>                       // This library allows you to communicate with I2C

Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); //par défaut addr=0x5A
Adafruit_VEML6070 capteurUV = Adafruit_VEML6070();       // une résistance de RSET=270kOhms est soudée sur le capteur
Adafruit_TCS34725 capteurLum = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X); //capteur luminosité


Adafruit_CCS811 ccs;
Adafruit_BMP280 bmp280;                // I2C
ClosedCube_HDC1080 hdc1080;

float seuil_haut = 815.0; // no rain
float seuil_bas = 425.0; // full rain

TwoWire Wire2(PB11, PB10); 

uint16_t r, g, b, c, lux; //capteur de luminosité
static const uint32_t pinPluvioAnalog = A4; //valeur analogique proportionnelle à la quantité de pluie
static const uint32_t pinPluvioGPIO = D2; //booléen indiquant la présence de pluie ou nom en fonction d'un seuil paramétré par potentiomètre

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

  Serial.println("Adafruit MLX90614 Emissivity Setter.\n");

  // init sensor
  if (!capteurTempCiel.begin(0x69,&Wire2)) //il faut re-init si on coupe l'alimentation au capteur (en premiere approche)
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  pinMode(pinPluvioAnalog,INPUT_ANALOG);
  pinMode(pinPluvioGPIO,INPUT_PULLDOWN);

  Serial.print("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);
  capteurUV.sleep(true);//le capteur UV est toujours alimenté et passe en low-power par commande I2C

  if (capteurLum.begin()) 
  {
    Serial.println("Found luminosity sensor");
  } 
  else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

	// Serial.println("ClosedCube HDC1080 Arduino Test");

	// // Default settings: 
	// //  - Heater off
	// //  - 14 bit Temperature and Humidity Measurement Resolutions
	// hdc1080.begin(0x40);

	// Serial.print("Manufacturer ID=0x");
	// Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
	// Serial.print("Device ID=0x");
	// Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
	
	// printSerialNumber();
 

  
}

void loop() {

  Serial.print("Temperature Objet:"); //celle à utiliser avec capteur pointé vers le ciel
  Serial.println(capteurTempCiel.readObjectTempC());
  Serial.print("Temperature Ambiente:");
  Serial.println(capteurTempCiel.readAmbientTempC());


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

  capteurUV.sleep(false);                       // pas besoin de réinitialiser la bibliothèque lorsque l'on sort du mode low-power
  Serial.print("VEML UV :"); 
  Serial.println(capteurUV.readUV()); // pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true);                        // diminue la conso à 1 microA


  capteurLum.enable();
  delay(10);
  capteurLum.getRawData(&r, &g, &b, &c);
  Serial.print("LUX :"); 
  Serial.println(capteurLum.calculateLux(r, g, b));
  capteurLum.disable();


  delay(1000);
  

  // Serial.print("Pressure = ");
  // Serial.print(bmp280.readPressure() / 100);
  // Serial.println(" Pa, ");

	// Serial.print("T=");
	// Serial.print(hdc1080.readTemperature());
	// Serial.print("C, RH=");
	// Serial.print(hdc1080.readHumidity());
	// Serial.println("%");



  // if(ccs.available()){
  //   if(!ccs.readData()){
  //     Serial.print("CO2: ");
  //     Serial.print(ccs.geteCO2());
  //     Serial.print("ppm, TVOC: ");
  //     Serial.println(ccs.getTVOC());
  //   }
  //   else{
  //     Serial.println("ERROR!");
  //     while(1);
  //   }
  // }
  // delay(5000);
}

