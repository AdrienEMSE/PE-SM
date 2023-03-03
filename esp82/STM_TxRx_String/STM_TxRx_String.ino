#include <ESP8266WiFi.h>

char s[20];

void setup() {  
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
      Serial.println("Hello STM-kun ! I'm ESP82-chan ! ^^");
      delay(1000);
      if (Serial.available()) {
        s = Serial.readString();
        Serial.print("You told me: ");
        Serial.print(s);
      }
}
