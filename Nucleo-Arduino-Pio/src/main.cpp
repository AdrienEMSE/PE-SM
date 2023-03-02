#include <Arduino.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor
#include "ClosedCube_HDC1080.h"


#include <Wire.h>                       // This library allows you to communicate with I2C

Adafruit_CCS811 ccs;
Adafruit_BMP280 bmp280;                // I2C
ClosedCube_HDC1080 hdc1080;

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
  // Enable I2C
  Wire.begin();                  // put here the Pins of I2C
  Serial.println("CCS811 test");      /* --- SETUP CCS811 on 0x5A ------ */
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  while(!ccs.available());



  Serial.println("BMP280 test");     /* --- SETUP BMP on 0x76 ------ */
  if (!bmp280.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

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
  
  Serial.print("BMP280 => Temperature = ");
  Serial.print(bmp280.readTemperature());
  Serial.print(" °C, ");

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
