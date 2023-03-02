#include <Arduino.h>


HardwareSerial Serial6(D0,D1);


typedef struct _msg_ESP
{
  float_t val1;
  uint32_t val2;
}msg_ESP;

msg_ESP tosend;
msg_ESP toreceive;

void setup() {
//  
  Serial.begin(9600);
  Serial6.begin(115200);
  tosend.val1 = PI;
  tosend.val2 = 0;



  
}

void loop() {

  tosend.val2++;

  Serial.println("sending msg_ESP to esp");
  Serial6.write((uint8_t *)&tosend,sizeof(msg_ESP));
  // while (Serial6.available() == 0) {}     //wait for data available
  // Serial6.readBytes((uint8_t*)&toreceive,sizeof(msg_ESP));      // remove any \r \n whitespace at the end of the String
  
  // Serial.print("received struct with : val1 = ");
  // Serial.print(toreceive.val1);
  // Serial.print(" val2 = ");
  // Serial.println(toreceive.val2);

  delay(1000);
  

}
