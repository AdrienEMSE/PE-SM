#include "msg_ESP.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

const char *ssid = "pi_meteo";
const char *password = "stationmeteo13120";


//R: ESP82 ready to listen to data
//K: Data was sent to MQTT server
//E1: Error 1: No Wifi. Cheeck Wifi connection, and reset
//E2: Error 2: No server
//E3: Error 3: checksum failed. Message is not valid

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char *mqtt_server = "10.42.0.1";

// Initializes the espClient
WiFiClient espClient;
PubSubClient client(espClient);

msg_ESP_class tosend(NULL);

int max_attempts = 5;
int attempts = 0;
uint16_t crc;


void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);
  attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts <= 20)
  {
    delay(500);
    //Serial.print(".");
    attempts ++;
  }
  if (attempts>20)
  {
    Serial.println("E1");//Error: no Wifi
  }
  //Serial.println("");
  //Serial.print("WiFi connected - ESP IP address: ");
  //Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected or the number of attempts is above limit
  while (!client.connected() && attempts <= 5)//Attempting MQTT connection
  {
    if (!client.connect("ESP8266"))
    {
      delay(2000);//2 seconds delays betweens trials
      attempts ++;    
    }
  }
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  while (Serial.available() == 0)
  {
    Serial.println("R");//Ready
    delay(2000);
  } // wait for data available

  Serial.readBytes((uint8_t *)&tosend._msg, sizeof(msg_ESP));//Read message from ST

  if(tosend.iscrcOk())//Check 
  {
    if (!client.connected())
    {
      attempts = 0;
      reconnect();
    }
    if (!client.connected())
    {
      Serial.println("E2");//Error: no server
    }
    else
    {
      client.publish("esp8266",(uint8_t *)&tosend._msg,sizeof(msg_ESP));
      client.disconnect();
      delay(500);
      Serial.println("K");
    }

  }
  else
  {
    Serial.println("E3");//Error: checksum failed
  }

}