#pragma once

#include <Arduino.h>

#define           poly     0x1021          /* crc-ccitt mask */

typedef struct _time
{
  uint16_t hour;
  uint8_t min;
  uint8_t sec;
}time_msg;

typedef struct _date
{
  uint16_t m;
  uint8_t j;
  uint8_t a;
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

} msg_ESP;

void updateCrc(msg_ESP * msg);
void iteration_crc(unsigned short ch);
void iteration_crc(unsigned short ch, unsigned short* crc);
