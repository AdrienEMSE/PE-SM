#include <Arduino.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor
#include <Wire.h>                       // This library allows you to communicate with I2C

Adafruit_CCS811 ccs;
Adafruit_BMP280 bmp280;                // I2C

uint32_t pinPluvioAnalog = A0;
uint32_t pinPluvioGPIO = D7;
float seuil_haut = 815.0; // no rain
float seuil_bas = 425.0; // full rain

void setup() {
//  
  Serial.begin(9600);
  pinMode(pinPluvioAnalog,INPUT_ANALOG);
  pinMode(pinPluvioGPIO,INPUT_PULLDOWN);





  
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
  

}
