#pragma once

#include <Arduino.h>

typedef struct _msg_ESP // Structure de message à envoyer à l'ESP
{
  uint8_t uv_index_level;
  float dht_temp_celsius;
  float dht_humidite_relative;
  double skyTemp_ambiant_celsius;
  double skyTemp_object_celsius;
  float pluie_pourcentage;
  bool pluie_gpio;
  float pression_bmp;
  float temperature_bmp;
  double humidite_relative_hdc;
  double temperature_hdc;
  uint16_t co2;
  uint16_t tvoc;
  char date_gps[32];
  char time_gps[32];
  uint16_t crc;

} msg_ESP;

