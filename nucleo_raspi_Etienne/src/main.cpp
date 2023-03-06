#include "Adafruit_PM25AQI.h"
#include <SoftwareSerial.h>
#include <MHZ19.h>

typedef struct _indoor_sensors // Structure de message à envoyer à la raspberry pi
{
  PM25_AQI_Data PM25_data;
  int CO2;
} indoor_sensors;

HardwareSerial Serial_Qual(D0, D1); // Liaison série vers PM2.5
HardwareSerial Serial_Pi(A2, A7);   // Liaison série vers la raspberry PI
SoftwareSerial Serial_CO2(D2, D3);  // Liaison série vers le capteur de CO2

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

MHZ19 myMHZ19;
indoor_sensors sensors;
int incomingInt;

void setup()
{

  sensors.CO2 = 0;

  // Wait for serial monitor to open
  Serial_Pi.begin(9600);
  while (!Serial_Pi)
    delay(10);

  Serial_Pi.println("Adafruit PMSA003I Air Quality Sensor");

  // Wait one second for sensor to boot up!
  delay(1000);

  // If using serial, initialize it and set baudrate before starting!
  // Uncomment one of the following
  Serial_Qual.begin(9600);
  Serial_CO2.begin(9600);
  Serial_Pi.begin(9600);

  myMHZ19.begin(Serial_CO2);
  myMHZ19.autoCalibration();

  if (!aqi.begin_UART(&Serial_Qual))
  { // connect to the sensor over hardware serial
    Serial_Pi.println("Could not find PM 2.5 sensor!");
    while (1)
      delay(10);
  }

  Serial_Pi.println("PM25 found!");
}

void loop()
{

  if (Serial_Pi.available() > 0)
  {
    incomingInt = Serial_Pi.read();
    while (!aqi.read(&sensors.PM25_data))
    {
      // Serial_Pi.println("Could not read from AQI");
      delay(500); // try again in a bit!
    }
    // Serial_Pi.println("AQI reading success");
    sensors.CO2 = myMHZ19.getCO2();
    Serial_Pi.print(sensors.PM25_data.particles_100um);
    Serial_Pi.print(',');
    Serial_Pi.print(sensors.PM25_data.particles_50um);
    Serial_Pi.print(',');
    Serial_Pi.print(sensors.PM25_data.particles_25um);
    Serial_Pi.print(',');
    Serial_Pi.print(sensors.PM25_data.particles_10um);
    Serial_Pi.print(',');
    Serial_Pi.print(sensors.PM25_data.particles_05um);
    Serial_Pi.print(',');
    Serial_Pi.print(sensors.PM25_data.particles_03um);
    Serial_Pi.print(',');
    Serial_Pi.println(sensors.CO2);
  }
}
