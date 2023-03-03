#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <TinyGPSPlus.h> // Lib GPS

#include <SoftwareSerial.h>

#include "Adafruit_VEML6070.h"
#include <Adafruit_Sensor.h>

#include <DHT.h> // LIB Température Humidité
#include <DHT_U.h>


#include <Adafruit_MLX90614.h>
#include <Adafruit_CCS811.h>            // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h>            // include main library for BMP280 - Sensor

#include "ClosedCube_HDC1080.h"




uint32_t pinPluvioAnalog = A4;
uint32_t pinPluvioGPIO = D2;
float seuil_haut = 815.0; // pas de pluie
float seuil_bas = 425.0; // beaucoup de pluie

static const int RXPin = 4, TXPin = 7; // PIN Liaison série GPS
static const uint32_t GPSBaud = 9600;  // BAUD GPS


TinyGPSPlus gps; // GPS
SoftwareSerial ss(RXPin, TXPin); // Liaison série vers GPS


#define DHTPIN 3     // Digital pin connected to the DHT sensor 

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified capteurTempHum(DHTPIN, DHTTYPE);

uint32_t delayMS_DHT = 2000;  // Delay pour le DHT


Adafruit_VEML6070 capteurUV = Adafruit_VEML6070(); //une résistance de RSET=270kOhms est soudée sur le capteur
Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); //par défaut addr=0x5A

//Capteur qualité de l'air :
Adafruit_CCS811 ccs; // CO2, TOV,
Adafruit_BMP280 bmp280; //Temperature et Pression                // I2C
ClosedCube_HDC1080 hdc1080;// Temperature et Humidité

HardwareSerial Serial6(D0,D1); // Liaison série vers ESP
TwoWire Wire2(PB11,PB10);

typedef struct _msg_ESP // Structure de message à envoyer à l'ESP
{
  float_t val1;
  uint32_t val2;
}msg_ESP;

msg_ESP aenvoyer;
msg_ESP arecevoir;


//Protoptypes fonctions utilitaires
void printSerialNumber();
void smartDelay(unsigned long ms);
void printInt(unsigned long val, bool valid, int len);
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void printStr(const char *str, int len);

void setup() {

  Serial.begin(9600);
  Serial6.begin(115200);
  aenvoyer.val1 = PI;
  aenvoyer.val2 = 5;
  Wire2.begin();

  while (!Serial);

  Serial.println("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);
   Serial.println(F("DHTxx Unified Sensor Example"));
  capteurTempHum.begin();



  Serial.println("Adafruit MLX90614 Emissivity Setter.\n");
  if (!capteurTempCiel.begin(0x69,&Wire2)) //il faut re-init si on coupe l'alimentation au capteur (en premiere approche)
  {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }

  Serial.println("CCS811 test");      /* --- SETUP CCS811 on 0x5A ------ */
  if(!ccs.begin(0x5B)){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  while(!ccs.available());


  Serial.println("BMP280 test");     /* --- SETUP BMP on 0x76 ------ */
  if (!bmp280.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

	Serial.println("ClosedCube HDC1080 Arduino Test");
	hdc1080.begin(0x40);

  pinMode(pinPluvioAnalog,INPUT_ANALOG);
  pinMode(pinPluvioGPIO,INPUT_PULLDOWN);

  
}


void loop() {

  aenvoyer.val2++;

  Serial.println("sending msg_ESP to esp");
  Serial6.write((uint8_t *)&aenvoyer,sizeof(msg_ESP));
  // while (Serial6.available() == 0) {}     //wait for data available
  // Serial6.readBytes((uint8_t*)&arecevoir,sizeof(msg_ESP));      // remove any \r \n whitespace at the end of the String
  
  // Serial.print("received struct with : val1 = ");
  // Serial.print(arecevoir.val1);
  // Serial.print(" val2 = ");
  // Serial.println(arecevoir.val2);

  // printDateTime(gps.date, gps.time);
  // Serial.println();
  
  // smartDelay(1000);

  // if (millis() > 5000 && gps.charsProcessed() < 10)
  //   Serial.println(F("No GPS data received: check wiring"));
  
  capteurUV.sleep(false); //pas besoin de réinitialiser
  Serial.print("UV light level: "); 
  Serial.println(capteurUV.readUV()); //pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true); //diminue la conso à 1 microA


  

  //attention fonctionnement de la librairie basé sur HAL_GetTick (parce que protocole de communication non conventionnel)
  //en particulier pour vérifier que 2s se sont écoulées depuis le dernier échantillonnage
  //donc prudence si on désactive les ticks si on passe en mode économie d'énergie le microcontrôleur

 
  // Get temperature event and print its value.
  sensors_event_t event;
  capteurTempHum.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  capteurTempHum.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }



  Serial.print("Temperature Objet:"); //celle à utiliser avec capteur pointé vers le ciel
  Serial.println(capteurTempCiel.readObjectTempC());
  Serial.print("Temperature Ambiente:");
  Serial.println(capteurTempCiel.readAmbientTempC());
  

  uint32_t rain_read = analogRead(pinPluvioAnalog);
  int rain_GPIO = digitalRead(pinPluvioGPIO);
  float rainPercent = (seuil_haut/(seuil_haut-seuil_bas)) -((float)rain_read)/(seuil_haut-seuil_bas);
  if(rainPercent > 1.0)
    rainPercent = 1.0;
  else if (rainPercent < 0.0)
    rainPercent = 0.0;
  Serial.print("rain_raw :");
  Serial.print(rain_read);
  Serial.print(" GPIO : ");
  Serial.print(!rain_GPIO); // read GPIO at 1 when no rain, 0 when rain
  Serial.print(" rain % : ");
  Serial.println( rainPercent );


  

  Serial.print("Pressure = ");
  Serial.print(bmp280.readPressure() / 100);
  Serial.println(" Pa, ");
  Serial.print("BMP280 => Temperature = ");
  Serial.print(bmp280.readTemperature());
  Serial.print(" °C, ");

	Serial.print("T=");
	Serial.print(hdc1080.readTemperature());
	Serial.print("C, RH=");
	Serial.print(hdc1080.readHumidity());
	Serial.println("%");



  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }
  
    delay(delayMS_DHT);
    delay(3000);
}


/*---------------------Fonctions utilitaires----------------------*/

void printSerialNumber() {
	Serial.print("Device Serial Number=");
	HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
	char format[12];
	sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
	Serial.println(format);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}


void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}