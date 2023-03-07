#pragma once

#include <Arduino.h>

//message struct avec verification d'integrité des données grâce au CCR16-CCITT;
//info @ https://srecord.sourceforge.net/crc16-ccitt.html

#define           poly     0x1021          /* crc-ccitt mask */

typedef struct _time
{
  uint16_t hour;
  uint8_t min;
  uint8_t sec;
}time_msg;

typedef struct _date
{
  uint16_t a;
  uint8_t m;
  uint8_t j;
}date_msg;

typedef struct _location_msg
{
  double lat;
  double lng;

}location_msg;

typedef struct _gps_msg
{
  time_msg msg_time;
  date_msg msg_date;
  location_msg msg_location;

}gps_msg;


typedef struct _msg_ESP // Structure de message à envoyer à l'ESP
{
  gps_msg msg_gps; // 24
  double temp_ambiant_celsius_sky; //8 
  double temp_object_celsius_sky; //8
  double humidite_relative_hdc; // 8
  double temperature_celsius_hdc; // 8
  float dht_temp_celsius; //4
  float dht_humidite_relative; //4
  float pluie_pourcentage; //4
  float pression_Pa_bmp; // 4
  float temperature_celsius_bmp; // 4
  int co2_ppm; // 4
  int tvoc_index; // 4
  bool pluie_gpio; // 1
  uint8_t uv_index_level; //1
  uint16_t crc; // 2

} msg_ESP; // le sizeof de msg_ESP correspond à la somme des sizeof de ce qui compose la struct
// la verification ne marche pas si la somme de ce qui est ajouté n'est pas un multiple de 8 octets 
// Actuellement : 88 octets, si il faut l'étendre, il faut aller jusqu'à 96 octets quitte à sur dimensionner les types
// crc doit être à la fin de la struct 

class msg_ESP_class
{
private:
  void augment_message_crc(unsigned short* crc);
  void iteration_crc(unsigned short ch, unsigned short* crc);
  HardwareSerial* Serial_ESP;
public:
  msg_ESP _msg;
  msg_ESP_class(HardwareSerial* Serial_comm);
  ~msg_ESP_class();
  uint16_t updateCrc(); // permet de calculer le crc
  bool iscrcOk();
};




