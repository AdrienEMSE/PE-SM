#pragma once
 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
 
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
 
} msg_ESP; // le sizeof de msg_ESP correspond à la somme des sizeof de ce qui compose la struct
// la verification ne marche pas si la somme de ce qui est ajouté n'est pas un multiple de 8 octets 
// Actuellement : 48 octets, si il faut l'étendre, il faut aller jusqu'à 52 octets quitte à sur dimensionner les types
// crc doit être à la fin de la struct 
 
