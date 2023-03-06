#pragma once

#include <Arduino.h>

#define           poly     0x1021          /* crc-ccitt mask */

typedef struct _msg_ESP // Structure de message à envoyer à l'ESP
{
  uint8_t uv_index_level;
  float dht_temp_celsius;
  float dht_humidite_relative;
  double temp_ambiant_celsius_sky;
  double temp_object_celsius_sky;
  float pluie_pourcentage;
  bool pluie_gpio;
  float pression_Pa_bmp;
  float temperature_celsius_bmp;
  double humidite_relative_hdc;
  double temperature_celsius_hdc;
  uint16_t co2_ppm;
  uint16_t tvoc_index;
  char date_gps[32];
  char time_gps[32];
  uint16_t crc;

} msg_ESP;

void updateCrc(msg_ESP * msg);
void iteration_crc(unsigned short ch);
