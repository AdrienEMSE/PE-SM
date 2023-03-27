#pragma once

#include <Arduino.h>

//message struct avec verification d'integrité des données grâce au CCR16-CCITT;
//info @ https://srecord.sourceforge.net/crc16-ccitt.html

#define           poly     0x1021          /* crc-ccitt mask */

//#define DEBUG

#ifdef DEBUG

#define safePrintSerial(x) Serial.print(x)
#define safePrintSerialln(x) Serial.println(x)
#define safePrintSerial2(x,y) Serial.print(x,y)
#define safePrintSerialln2(x,y) Serial.println(x,y)

#else

#define safePrintSerial(x)
#define safePrintSerialln(x)
#define safePrintSerial2(x,y)
#define safePrintSerialln2(x,y) 


#endif

#define TIMEOUT_MS 5000 //timeout pour la communication liaison série ESP

// typedef struct _time
// {
//   uint16_t hour;
//   uint8_t min;
//   uint8_t sec;
// }time_msg;

// typedef struct _date
// {
//   uint16_t a;
//   uint8_t m;
//   uint8_t j;
// }date_msg;

// typedef struct _gps_msg
// {
//   time_msg msg_time;
//   date_msg msg_date;
//   location_msg msg_location;

// }gps_msg;

typedef struct _location_msg
{
  double lat; //8
  double lng; //+8
  uint32_t filler1; //+4
  uint16_t filler2; //+2
  uint16_t crc; //+2 = 24
  

}location_msg;

typedef enum _msg_type
{
  sensor_msg,
  gps_msg
}msg_type;

typedef struct _msg_ESP // Structure de message à envoyer à l'ESP
{
  float temp_ambiant_celsius_sky; //4 
  float temp_object_celsius_sky; //4
  float humidite_relative_hdc; // 4
  float temperature_celsius_hdc; //4
  float dht_temp_celsius; //4
  float dht_humidite_relative; //4
  float pluie_pourcentage; //4
  float pression_Pa_bmp; // 4
  float temperature_celsius_bmp; // 4
  uint16_t lux; // 2
  uint16_t uv_index_level; //2
  uint16_t co2_ppm; // 2
  uint16_t tvoc_index; // 2
  uint16_t pluie_gpio; // 2
  uint16_t crc; // 2 // crc doit être le dernier déclaré dans la struct
  uint16_t wind_speed;
  uint16_t wind_heading;

} msg_ESP; // le sizeof de msg_ESP correspond à la somme des sizeof de ce qui compose la struct
// la verification ne marche pas si la somme de ce qui est ajouté n'est pas un multiple de 8 octets 
// Actuellement : 48 octets, si il faut l'étendre, il faut aller jusqu'à 52 octets quitte à sur dimensionner les types
// crc doit être à la fin de la struct 

class msg_ESP_class
{
private:
  void augment_message_crc(unsigned short* crc);
  void iteration_crc(unsigned short ch, unsigned short* crc);
  void flush();
  HardwareSerial* Serial_ESP;
public:
  location_msg _msg_location;
  msg_ESP _msg_sensor;
  msg_type typeToSend;
  msg_ESP_class(HardwareSerial* Serial_comm);
  ~msg_ESP_class();
  uint16_t updateCrc_sensor(); // permet de calculer le crc
  uint16_t updateCrc_gps(); // permet de calculer le crc
  bool iscrcOk(msg_type type);
  void send_msg_sensor();
  void send_msg_location();
  bool safeSendX1(msg_type type);
  bool safeSendXN(msg_type type,int n);
};




