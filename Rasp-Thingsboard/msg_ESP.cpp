#include "msg_ESP.h"

using namespace std;

extern "C" void save_data(msg_ESP &msg)
{
	FILE * fp;
	
	fp = fopen("outdoor_station.csv","a");
	
	fprintf(fp,"%u,%u,%u,",msg.msg_gps.msg_time.hour,msg.msg_gps.msg_time.min,msg.msg_gps.msg_time.sec);//Time
	
	fprintf(fp,"%u,%d,%d,",msg.msg_gps.msg_date.j, msg.msg_gps.msg_date.m, msg.msg_gps.msg_date.a);//Date
	
	fprintf(fp,"%f,%f,",msg.msg_gps.msg_location.lat,msg.msg_gps.msg_location.lng);//Location

	fprintf(fp,"%f,%f,",msg.temp_ambiant_celsius_sky,msg.temp_object_celsius_sky);//Sky
	
	fprintf(fp,"%f,%f,",msg.humidite_relative_hdc,msg.temperature_celsius_hdc);//Hdc
	
	fprintf(fp,"%f,%f,",msg.dht_temp_celsius,msg.dht_humidite_relative);//DHT

	fprintf(fp,"%f,%d,",msg.pluie_pourcentage,msg.pluie_gpio);//pluie
	
	fprintf(fp,"%f,%f,",msg.pression_Pa_bmp,msg.temperature_celsius_bmp);//BMP

	fprintf(fp,"%d,",msg.co2_ppm);//C02
	
	fprintf(fp,"%d,",msg.tvoc_index);//TVOC
	
	fprintf(fp,"%u",msg.uv_index_level);//UV
	
	fprintf(fp,"\n",msg.uv_index_level);//Return to line
	
	
	
	fclose(fp);
	printf("Writing done.\n");
} 

extern "C" void prompt()
{
	printf("It works.\n");
} 
