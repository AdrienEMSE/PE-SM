#include <Wire.h>
#include "Adafruit_VEML6070.h"

Adafruit_VEML6070 capteurUV = Adafruit_VEML6070(); //une résistance de RSET=270kOhms est soudée sur le capteur

void setup() {
  Serial.begin(9600);
  Serial.println("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);  // clear ACK's et écrit 0x06 dans le registre de commande conformement au mode d'emploi
}


void loop() {
  
  Serial.print("UV light level: "); Serial.println(capteurUV.readUV()); //pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true); //diminue la conso à 1 microA
  delay(1000);
  capteurUV.sleep(false); //pas besoin de réinitialiser
  }