
#include <Arduino.h>
#include <Wire.h>                       // This library allows you to communicate with I2C
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 7; // PIN Liaison série GPS
static const uint32_t GPSBaud = 9600;  // BAUD GPS


TinyGPSPlus gps; // GPS
SoftwareSerial ss(RXPin, TXPin); // Liaison série vers GPS


void setup() {

  Serial.begin(9600);



  
}


void loop() {
  

  
}

