#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); //par défaut addr=0x5A

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Adafruit MLX90614 Emissivity Setter.\n");

  // init sensor
  capteurTempCiel.begin(0x00)
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
  Serial.println("Erase");
  capteurTempCiel.write16(0x2E, 0); // erase
  delay(10);

  Serial.println("Write");
  capteurTempCiel.write16(0x2E, 0x69);
  delay(10);

  Serial.println("Read");
  Serial.println(capteurTempCiel.read16(0x2E));
  while(1);
}

void loop() {
  delay(1000);

}