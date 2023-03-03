#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "pi_meteo";
const char* password = "stationmeteo13120";

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char* mqtt_server = "10.42.0.1";

// Initializes the espClient
WiFiClient espClient;
PubSubClient client(espClient);

typedef struct _msg
{
  float_t val1;
  uint32_t val2;
}msg;

msg tosend;
int max_attempts = 5;
int attempts = 0;


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected or the number of attempts is above limit
  while (!client.connected() && attempts<=5) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP8266")) {
      Serial.println("connected");  
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
      attempts+=1;
    }
  }
}

void setup() {  
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  while (Serial.available() == 0) {}     //wait for data available
  tosend = Serial.readBytes((uint8_t*)&toreceive,sizeof(msg));
  
  if (!client.connected()) {
    attempts = 0
    reconnect();
  }
  client.publish("esp8266",tosend);
  Serial.println("Data sent.");
  client.disconnect();
  }
}
