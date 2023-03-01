#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); //par défaut addr=0x5A

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Adafruit MLX90614 Emissivity Setter.\n");

  // init sensor
  if (!capteurTempCiel.begin()) //il faut re-init si on coupe l'alimentation au capteur (en premiere approche)
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
}

void loop() {
  Serial.print("Temperature Objet:"); //celle à utiliser avec capteur pointé vers le ciel
  Serial.println(capteurTempCiel.readObjectTempC());
  Serial.print("Temperature Ambiente:");
  Serial.println(capteurTempCiel.readAmbientTempC());
  delay(1000);

}