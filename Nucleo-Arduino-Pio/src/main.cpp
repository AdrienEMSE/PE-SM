#include <Arduino.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor
#include "ClosedCube_HDC1080.h"


#include <Wire.h>                       // This library allows you to communicate with I2C

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
//  
  Serial.begin(9600);
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
