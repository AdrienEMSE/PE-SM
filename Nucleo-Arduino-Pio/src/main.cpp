// #include "Wire.h"
// #include "STM32LowPower.h"

// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   LowPower.begin();
//   Serial.begin(9600);
// }

// void loop() {
//   safePrintSerialln("HIGH");
//   Serial.flush();
//   digitalWrite(LED_BUILTIN, HIGH);
//   LowPower.deepSleep(1000);
//   safePrintSerialln("LOW");
//   Serial.flush();
//   digitalWrite(LED_BUILTIN, LOW);
//   LowPower.deepSleep(1000);
// }

/*----------INCLUDES----------*/


#include <Arduino.h>
#include <assert.h>

#include <Wire.h> //I2C
#include <SPI.h>

#include <TinyGPSPlus.h> // Lib GPS

#include <SoftwareSerial.h>

#include "Adafruit_VEML6070.h" //Lib capteur UV
#include <Adafruit_Sensor.h>

#include <DHT.h> // Lib Température Humidité
#include <DHT_U.h>

#include <Adafruit_MLX90614.h> //Lib Skytemp

//les trois suivants sont sur le même circuit
#include <Adafruit_CCS811.h> // include library for CCS811 - Sensor from martin-pennings https://github.com/maarten-pennings/CCS811
#include <Adafruit_BMP280.h> // include main library for BMP280 - Sensor
#include "ClosedCube_HDC1080.h" // pour HDC1080

#include "Adafruit_TCS34725.h" // capteur de luminosite

#include "msg_ESP.h" //classe réalisée par l'équipe PE pour regrouper les données à envoyer au serveur

#include "STM32LowPower.h"
//#include "STM32LowPower.h" //passer en mode deepSleep
#include "STM32RTC.h" //la RTC permet de sortir du mode deepSleep

#include <WiFiEspClient.h> //communication avec l'ESP pour le wi-fi
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <ThingsBoard.h> //support Thingsboard


/*----------MACRO----------*/
//NB: #define DEBUG à activer/désactiver dans debug.h et msg_ESP.h pour activer/désactiver la sortie d'informations utiles sur Serial (UART de debug)
//TODO FIX

#define DHTTYPE DHT22 // DHT 22 (AM2302)

#define WIFI_AP "POCO_F3"
#define WIFI_PASSWORD "Sapristi"
#define TOKEN "CZKwRlNJJBqjGpNZ1kL4"


/*----------PINS----------*/


static const uint32_t pinPluvioAnalog = A4; //valeur analogique proportionnelle à la quantité de pluie
static const uint32_t pinPluvioGPIO = D2; //booléen indiquant la présence de pluie ou nom en fonction d'un seuil paramétré par potentiomètre
static const int RXPin = D4, TXPin = D7; // PIN Liaison série GPS
static const int wake_ccs = D5; //pour piloter le passage du CCS en mode économie d'énergie
static const int dht_pin = D3; //pour communiquer avec le DHT selon un protocole non-standard
static const int anemo_phase_1 = A1, anemo_phase_2 = A2;
static const int capteur_de_foudre = A3;

//tous les pins suivants servent à couper l'alimentation aux composants lorsqu'ils ne sont pas utilisés
static const int pin_big_blue_button = USER_BTN;

static const int alimentation_gps = D8;
static const int alimentation_dht = D9;
static const int alimentation_skytemp = D10;
static const int alimentation_pluvio = D11;
static const int alimentation_lumi = D12;
static const int alimentation_ESP = D13;
static const int alimentation_anemo = PE10;

/*----------PARAMS----------*/

//ces deux paramètres sont utilisés avec le pluviomètre
const float seuil_haut = 815.0; // pas de pluie
const float seuil_bas = 425.0;  // beaucoup de pluie

static const uint32_t GPSBaud = 9600;    // BAUD GPS


//ce qui suit est utilisé avec l'ESP
char thingsboardServer[] = "thingsboard.cloud";
int status = WL_IDLE_STATUS;

/*----------VAR----------*/

uint16_t r, g, b, c, lux; //capteur de luminosité
// String receive_string;
// String send_string;
bool thunder_detected = false;

/*----------STRUCT ET CLASSES----------*/

TinyGPSPlus gps;                 // GPS (latitude, longitude)
SoftwareSerial ss(RXPin, TXPin); // Liaison série vers GPS

DHT_Unified capteurTempHum(dht_pin, DHTTYPE); //DHT (Température, humidité)

Adafruit_TCS34725 capteurLum = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X); //capteur luminosité

Adafruit_VEML6070 capteurUV = Adafruit_VEML6070();       // une résistance de RSET=270kOhms est soudée sur le capteur

Adafruit_MLX90614 capteurTempCiel = Adafruit_MLX90614(); // par défaut addr=0x5A, nous l'avons changée à 0x69

// Capteur qualité de l'air :
Adafruit_CCS811 ccs;        // CO2, TOV
Adafruit_BMP280 bmp280;     // Temperature et Pression
ClosedCube_HDC1080 hdc1080; // Temperature et Humidité



HardwareSerial Serial6(D0, D1); // Liaison série vers ESP
TwoWire Wire2(PB11, PB10);  //le MLX90614 est sur sa propre ligne I2C car il entrait en conflit avec les autres capteurs
msg_ESP_class aenvoyer(&Serial6); //classe regroupant les données récupérées sur les capteurs


// Connection WiFi avec l'Esp
WiFiEspClient espClient;
ThingsBoard tb(espClient);

/*----------PROTOTYPES----------*/

void callback_foudre();
void smartDelay(unsigned long ms); //machine d'état GPS
void dateTimePrint(TinyGPSDate &d, TinyGPSTime &t);
void printCapteurs();

//machine d'état ESP
void reconnect();
void InitWiFi();

/*----------CODE----------*/

void setup()
{

#ifdef DEBUG //ne pas oublier de bien choisir le #define DEBUG pour avoir accès ou non à des informations via l'UART du ST-LINK!
  Serial.begin(9600);    // Liaison série vers ordinateur
#endif
  Serial6.begin(9600); // Liaison serie ESP
  ss.begin(GPSBaud);     // Liaison serie GPS
  Wire2.begin();         // I2C Skytemp

  pinMode(pinPluvioAnalog, INPUT_ANALOG);
  pinMode(pinPluvioGPIO, INPUT_PULLDOWN);

  pinMode(wake_ccs,OUTPUT);
  digitalWrite(wake_ccs,HIGH);

  pinMode(pin_big_blue_button,INPUT_PULLDOWN);
  
  pinMode(alimentation_anemo,OUTPUT);
  pinMode(alimentation_dht,OUTPUT);
  pinMode(alimentation_skytemp,OUTPUT);
  pinMode(alimentation_lumi,OUTPUT);
  pinMode(alimentation_ESP,OUTPUT);
  pinMode(alimentation_gps,OUTPUT);

  pinMode(capteur_de_foudre,INPUT_FLOATING);
  
  
  digitalWrite(alimentation_anemo,LOW);
  digitalWrite(alimentation_dht,LOW); 
  digitalWrite(alimentation_skytemp,LOW);
  digitalWrite(alimentation_pluvio,LOW);
  digitalWrite(alimentation_lumi,LOW);
  digitalWrite(alimentation_ESP,LOW);
  
  
  digitalWrite(alimentation_gps,HIGH);//POWER GPS ON 
  uint32_t timer = millis();
  while(true)
  {
    delayGPS(100);
    if(gps.date.day() != 0)
    {
      safePrintSerialln("Le gps a atteint le satellite");
      aenvoyer._msg_location.lat =gps.location.lat();
      aenvoyer._msg_location.lng =gps.location.lng();

      digitalWrite(alimentation_ESP,HIGH);

      //Protocole ThingsBoard
      InitWiFi(); //Initialisation WiFi
      reconnect();
      if ( tb.connected() ) {
        tb.sendTelemetryFloat("lattitude", aenvoyer._msg_location.lat);
        tb.sendTelemetryFloat("longitude", aenvoyer._msg_location.lng);
      }
      digitalWrite(alimentation_ESP,LOW);
      break;
    }
    if(millis() > timer + 10000)
    {
      timer = millis();
      safePrintSerialln("Le gps n'a toujours pas atteint le satellite...");
    }
    if( digitalRead(pin_big_blue_button)==HIGH) // Si l'utilisateur appuie sur le bouton bleu, on skip le gps à l'init du programme
    {
      break;
    }

  }   

  digitalWrite(alimentation_gps,LOW); //POWER GPS OFF 
  ss.end(); //il faut end ss avant de passer en mode deepSleep sinon cela introduit des réveils intempestifs car l'UART peut déranger le MCU en mode STOP


//il n'y a pas besoin d'alimenter le VEML6070 et le DHT pour initialiser la communication
  safePrintSerialln("VEML6070 Test");
  capteurUV.begin(VEML6070_1_T);
  capteurUV.sleep(true);//le capteur UV est toujours alimenté et passe en low-power par commande I2C

  safePrintSerialln(F("DHTxx Unified Sensor Example"));
  capteurTempHum.begin();


//par contre la librairie vérifie la présence du MLX90614 donc il faut l'alimenter pour qu'il réponde
  digitalWrite(alimentation_skytemp,HIGH);
  safePrintSerialln("Adafruit MLX90614 Emissivity Setter.\n");
  if (!capteurTempCiel.begin(0x69, &Wire2)) // il faut re-init si on coupe l'alimentation au capteur
  {
    safePrintSerialln("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
  digitalWrite(alimentation_skytemp,LOW);

//idem pour le CCS, sauf que lui est toujours alimenté, on vient lui indiquer s'il faut communiquer par I2C avec le pin WAKE
  digitalWrite(wake_ccs, LOW);
  safePrintSerialln("CCS811 test"); /* --- SETUP CCS811 on 0x5A ------ */
  if (!ccs.begin(0x5B))
  {
    safePrintSerialln("Failed to start sensor! Please check your wiring.");
    while (1);
  }
  while (!ccs.available());
  ccs.setDriveMode(CCS811_DRIVE_MODE_60SEC); //mode le plus économe en énergie
  digitalWrite(wake_ccs, HIGH);

//le bmp280 est sur la même carte que le CCS, et est toujours alimenté, parce que le CCS doit l'être
  safePrintSerialln("BMP280 test"); /* --- SETUP BMP on 0x76 ------ */
  if (!bmp280.begin(0x76))
  {
    safePrintSerialln("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }

//idem pour le HDC1080
  safePrintSerialln("ClosedCube HDC1080 Arduino Test");
  hdc1080.begin(0x40);


//même commentaire pour le capteur de luminosité que pour le MLX90614
  digitalWrite(alimentation_lumi,HIGH);
  if (capteurLum.begin()) 
  {
    safePrintSerialln("Found luminosity sensor");
  } 
  else 
  {
    safePrintSerialln("No TCS34725 found ... check your connections");
    while (1);
  }
  digitalWrite(alimentation_lumi,LOW);

  LowPower.begin(); //nécessaire pour utiliser le mode deepSleep par la suite
  LowPower.attachInterruptWakeup(capteur_de_foudre,callback_foudre,RISING,DEEP_SLEEP_MODE);
}

void loop()
{
//NB: la communication avec un capteur doit toujours être précédée de son alimentation (pin d'alimentation) ou de son réveil du mode low-power
//et suivie de la coupure de son alimentation (ou du retour dans son mode low-power)

//NB:pour interpréter les données, et comprendre plus en détail les modes low-power le cas échéant, se rapporter aux fiches techniques des composants

  capteurUV.sleep(false);                       // pas besoin de réinitialiser la bibliothèque lorsque l'on sort du mode low-power
  aenvoyer._msg_sensor.uv_index_level = capteurUV.readUV(); // pour interpréter: https://www.vishay.com/docs/84310/designingveml6070.pdf page 5
  capteurUV.sleep(true);                        // diminue la conso à 1 microA


//NB: protocole de communication du DHT se base sur HAL_GetTick()
  digitalWrite(alimentation_dht,HIGH);
  sensors_event_t event;
  capteurTempHum.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    safePrintSerialln(F("Error reading temperature DHT!"));
  }
  else
  {
    aenvoyer._msg_sensor.dht_temp_celsius = event.temperature;
  }

  capteurTempHum.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    safePrintSerialln(F("Error reading humidity DHT!"));
  }
  else
  {
    aenvoyer._msg_sensor.dht_humidite_relative = event.relative_humidity;
  }
  digitalWrite(alimentation_dht,LOW);


  digitalWrite(alimentation_skytemp,HIGH);
  capteurTempCiel.begin(0x69, &Wire2);
  aenvoyer._msg_sensor.temp_object_celsius_sky = capteurTempCiel.readObjectTempC();
  aenvoyer._msg_sensor.temp_ambiant_celsius_sky = capteurTempCiel.readAmbientTempC();
  digitalWrite(alimentation_skytemp,LOW);


  digitalWrite(alimentation_pluvio,HIGH);
  uint32_t rain_read = analogRead(pinPluvioAnalog);
  int rain_GPIO = digitalRead(pinPluvioGPIO);
  digitalWrite(alimentation_pluvio,LOW);
  float rainPercent = (seuil_haut / (seuil_haut - seuil_bas)) - ((float)rain_read) / (seuil_haut - seuil_bas);
  if (rainPercent > 1.0)
    rainPercent = 1.0;
  else if (rainPercent < 0.0)
    rainPercent = 0.0;
  

  aenvoyer._msg_sensor.pluie_gpio = rain_GPIO;
  aenvoyer._msg_sensor.pluie_pourcentage = rainPercent;

  aenvoyer._msg_sensor.pression_Pa_bmp = bmp280.readPressure();
  aenvoyer._msg_sensor.temperature_celsius_bmp = bmp280.readTemperature();

  aenvoyer._msg_sensor.temperature_celsius_hdc = hdc1080.readTemperature();
  aenvoyer._msg_sensor.humidite_relative_hdc = hdc1080.readHumidity();

  digitalWrite(wake_ccs, LOW);
  if (ccs.available())
  {
    if (!ccs.readData())
    {
      aenvoyer._msg_sensor.co2_ppm = ccs.geteCO2();
      aenvoyer._msg_sensor.tvoc_index = ccs.getTVOC();
    }
    else
    {
      safePrintSerialln("ERROR air quality CCS!");
    }
  }
  digitalWrite(wake_ccs, HIGH);




  capteurLum.enable();
  capteurLum.getRawData(&r, &g, &b, &c);
  aenvoyer._msg_sensor.lux = capteurLum.calculateLux(r, g, b);
  capteurLum.disable();


  printCapteurs(); //n'impriment rien si pas en mode #define DEBUG


  
  digitalWrite(alimentation_ESP,HIGH);


  // InitWiFi();

  // reconnect();

  // if ( tb.connected() ) //on n'envoie des données que si le serveur est accessible
  // {
  //   // ATTENTION: pour une raison inconnue, si la chaîne de caractères est trop longue, l'envoi échoue. Possiblement quelque chose à voir avec une vérification de taille mémoire qui échoue quelque part dans les méandres de la librairie ThingsBoard. Garder des noms courts.

  //   tb.sendTelemetryFloat("temp_amb_sky", aenvoyer._msg_sensor.temp_ambiant_celsius_sky);
  //   tb.sendTelemetryFloat("temp_obj_sky", aenvoyer._msg_sensor.temp_object_celsius_sky);
  //   tb.sendTelemetryFloat("humidite_hdc", aenvoyer._msg_sensor.humidite_relative_hdc);
  //   tb.sendTelemetryFloat("temp_hdc", aenvoyer._msg_sensor.temperature_celsius_hdc);
  //   tb.sendTelemetryFloat("humidite_dht", aenvoyer._msg_sensor.dht_humidite_relative);
  //   tb.sendTelemetryFloat("temp_dht", aenvoyer._msg_sensor.dht_temp_celsius);
  //   tb.sendTelemetryFloat("pluie_pourcent", aenvoyer._msg_sensor.pluie_pourcentage);
  //   tb.sendTelemetryFloat("pression_bmp", aenvoyer._msg_sensor.pression_Pa_bmp);
  //   tb.sendTelemetryFloat("temp_bmp", aenvoyer._msg_sensor.temperature_celsius_bmp);
    
  //   tb.sendTelemetryInt("lux", aenvoyer._msg_sensor.lux);
  //   tb.sendTelemetryInt("uv_level", aenvoyer._msg_sensor.uv_index_level);
  //   tb.sendTelemetryInt("co2_ppm", aenvoyer._msg_sensor.co2_ppm);
  //   tb.sendTelemetryInt("tvoc_index", aenvoyer._msg_sensor.tvoc_index);
  //   tb.sendTelemetryBool("pluie_gpio", aenvoyer._msg_sensor.pluie_gpio);
  //   tb.sendTelemetryBool("thunder",thunder_detected);
  //   thunder_detected = false;
  // }

#ifdef DEBUG
  Serial.flush(); 
#endif
  Serial6.flush(); //attendre que tout soit transmis par UART avant de passer en mode STOP. Sans cette ligne, le microcontrôleur passe en STOP avant d'avoir tout envoyé ce qui conduit à un message corrompu en partie.

  digitalWrite(alimentation_ESP,LOW);
  

  //indication de la durée à passer en STOP en ms
  LowPower.deepSleep(10000); //deepSleep est en fait le mode STOP, avec réveil piloté par RTC, configurée en low power par la librairie
}

/*---------------------Fonctions utilitaires----------------------*/




// This custom version of delay() ensures that the gps object
// is being "fed".
void delayGPS(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}



void dateTimePrint(TinyGPSDate &d, TinyGPSTime &t)
{
  char toprint[50];
  if(!d.isValid())
  {

    sprintf(toprint, "********** ");
    safePrintSerial(toprint);
  }
  else
  {
    sprintf(toprint, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    safePrintSerial(toprint);
  }

  if (!t.isValid())
  {
    sprintf(toprint, "********** ");
    safePrintSerial(toprint);
  }
  else
  {
    sprintf(toprint, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    safePrintSerial(toprint);
  }
}



void printCapteurs() //affiche toutes les données collectées pour comparer avec ce qui est reçu côté serveur
{
  dateTimePrint(gps.date,gps.time);
  safePrintSerial(gps.location.lat());
  safePrintSerial(F(","));
  safePrintSerialln(gps.location.lng());
  safePrintSerial("UV light level: ");
  safePrintSerialln(aenvoyer._msg_sensor.uv_index_level);
  safePrintSerial(F("Temperature: "));
  safePrintSerial(aenvoyer._msg_sensor.dht_temp_celsius);
  safePrintSerialln(F("°C"));
  safePrintSerial(F("Humidity: "));
  safePrintSerial(aenvoyer._msg_sensor.dht_humidite_relative);
  safePrintSerialln(F("%"));
  safePrintSerial("Temperature Objet:"); // celle à utiliser avec capteur pointé vers le ciel
  safePrintSerialln(aenvoyer._msg_sensor.temp_object_celsius_sky);
  safePrintSerial("Temperature Ambiante:");
  safePrintSerialln(aenvoyer._msg_sensor.temp_ambiant_celsius_sky);
  safePrintSerial(" GPIO : ");
  safePrintSerial(!aenvoyer._msg_sensor.pluie_gpio); // read GPIO at 1 when no rain, 0 when rain
  safePrintSerial(" rain % : ");
  safePrintSerialln(aenvoyer._msg_sensor.pluie_pourcentage);
  safePrintSerial("Pressure = ");
  safePrintSerial(aenvoyer._msg_sensor.pression_Pa_bmp);
  safePrintSerialln(" Pa, ");
  safePrintSerial("BMP280 => Temperature = ");
  safePrintSerial(aenvoyer._msg_sensor.temperature_celsius_bmp);
  safePrintSerial(" °C, ");
  safePrintSerial("T=");
  safePrintSerial(aenvoyer._msg_sensor.temperature_celsius_hdc);
  safePrintSerial("C, RH=");
  safePrintSerial(aenvoyer._msg_sensor.humidite_relative_hdc);
  safePrintSerialln("%");
  safePrintSerial("CO2: ");
  safePrintSerial(aenvoyer._msg_sensor.co2_ppm);
  safePrintSerial("ppm, TVOC: ");
  safePrintSerialln(aenvoyer._msg_sensor.tvoc_index);
  safePrintSerial("Lux: "); 
  safePrintSerialln(aenvoyer._msg_sensor.lux);
  safePrintSerial("THUNDER :");
  safePrintSerialln(thunder_detected);
  thunder_detected = false;

}

//notre configuration d'horloge permet de diminuer au maximum la fréquence (et donc la consommation) tout en gardant les fonctionnalités.
void SystemClock_Config(void) //NB si jamais il y a des problèmes avec le mode deepSleep, commenter cette fonction pour que celle définie en WEAK ailleurs prévale (configuration par défaut)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void InitWiFi()
{
  // initialize ESP module
  WiFi.init(&Serial6);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    safePrintSerialln("WiFi shield not present");
    // don't continue
    while (true){
      LowPower.deepSleep(0);
    } //si l'ESP ne répond plus, c'est un problème hardware, la station ne fonctionne plus, ça ne sert à rien de continuer
  }

  safePrintSerialln("Connecting to AP ...");
  // attempt to connect to WiFi network
  int attempts = 0;
  while ( status != WL_CONNECTED && attempts < 3) {
    safePrintSerial("Attempting to connect to WPA SSID: ");
    safePrintSerialln(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
    attempts++;
  }
  if(attempts==5)
  {
    safePrintSerialln("Couldn't connect to AP");
  }
  else
  {
    safePrintSerialln("Connected to AP");
  }

}

void reconnect() {
  int attempts = 0;
  while (!tb.connected() && attempts < 3) {
    safePrintSerial("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      safePrintSerialln( "[DONE]" );
    } else {
      safePrintSerial( "[FAILED]" );
      safePrintSerialln( " : retrying in 5 seconds" );
      // Wait 5 seconds before retrying
      delay( 5000 );
      attempts++;
    }
  }
}

void callback_foudre()
{
  thunder_detected = true;
}