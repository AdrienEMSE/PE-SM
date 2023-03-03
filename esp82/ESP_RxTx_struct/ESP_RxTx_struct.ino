#include <ESP8266WiFi.h>

typedef struct _msg
{
  float_t val1;
  uint32_t val2;
}msg;

msg tosend;
msg toreceive;


void setup() {  
  Serial.begin(115200);

  tosend.val1 = 3.14;
  tosend.val2= 11;
}

void loop() {

  //Serial.print("Sending data to stm32");
  Serial.write((uint8_t *)&tosend,sizeof(msg));
  delay(2000);
  //while (Serial.available() == 0) {}     //wait for data available
  //Serial.readBytes((uint8_t*)&toreceive,sizeof(msg));      // remove any \r \n whitespace at the end of the String 
}
 
