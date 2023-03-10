#include "msg_ESP.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// Put SSID and password of raspberry pi.
//The raspberry must be configured as Hotspot.
const char *ssid = "pi_meteo";
const char *password = "stationmeteo13120";

// R: ESP82 ready to listen to data
// K: Data was sent to MQTT server
// E1: Error 1: No Wifi. Cheeck Wifi connection, and reset
// E2: Error 2: No server
// E3: Error 3: checksum failed. Message is not valid

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char *mqtt_server = "10.42.0.1";

// Initializes the espClient
WiFiClient espClient;
PubSubClient client(espClient);

msg_ESP_class tosend(NULL);

int attempts = 0;
uint16_t crc;
bool crc_OK;
uint8_t type_of_msg;

void setup_wifi()
{
  delay(10);
  WiFi.begin(ssid, password);
  attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts <= 10) // Loop until Wifi is found or the number of attempts is above limit
  {
    delay(2000); // 2 seconds delays betweens trials
    attempts++;
  }
  if (attempts > 20)
  {
    Serial.println("E1"); // Error: no Wifi
  }
}

void reconnect()
{
  // Loop until we're reconnected or the number of attempts is above limit
  while (!client.connected() && attempts <= 5) // Attempting MQTT connection
  {
    if (!client.connect("ESP8266"))
    {
      delay(2000); // 2 seconds delays betweens trials
      attempts++;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop()
{
  while (Serial.available() == 0)// wait for data available
  {
    Serial.println("R"); //Send Ready to STM
    delay(2000);
  }

  Serial.readBytes((uint8_t *)&type_of_msg, sizeof(uint8_t)); // STM send message type
  delay(10);

  if (type_of_msg == 0)// If message is sensors data
  {
    Serial.readBytes((uint8_t *)&tosend._msg_sensor, sizeof(msg_ESP)); // Read message from STM
    crc_OK = tosend.iscrcOk(msg_type::sensor_msg);// Check if message is not corrupted
  }
  else if (type_of_msg == 1)//If message is gps location
  {
    Serial.readBytes((uint8_t *)&tosend._msg_location, sizeof(location_msg)); // Read message from STM
    crc_OK = tosend.iscrcOk(msg_type::gps_msg);// Check if message is not corrupted
  }

  if (crc_OK)
  {
    if (!client.connected())
    {
      attempts = 0;
      reconnect();
    }
    if (!client.connected())
    {
      Serial.println("E2"); // Error: no server
    }
    else
    {
      if (type_of_msg == 0)
      {
        client.publish("esp8266", (uint8_t *)&tosend._msg_sensor, sizeof(msg_ESP)); //Send sensors data to raspberry
      }
      else
      {
        client.publish("esp8266", (uint8_t *)&tosend._msg_location, sizeof(location_msg)); //Send gps location to raspberry
      }
      client.disconnect();
      delay(500);
      Serial.println("K");//Send Ok to STM
    }
  }
  else
  {
    Serial.println("E3"); // Error: checksum failed
  }
}