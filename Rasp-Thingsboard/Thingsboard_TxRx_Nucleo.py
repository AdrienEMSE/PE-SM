import os
from time import sleep
import sys
import board
import paho.mqtt.client as mqtt
import json
import serial

THINGSBOARD_HOST = 'thingsboard.cloud' #Adresse du serveur Thingsboard
ACCESS_TOKEN = 'mxxDnECThka5KF4PU8vx' #Ecrire token Thingsboard

ser = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.25)#Initialisation connexion avec Nucleo

#Json envoyé sur Thingsboard. Ajouter un paramètre pour ajouter un capteur.
sensor_data = {'particles_100um': 0, 'particles_50um': 0, 'particles_25um': 0, 'particles_10um': 0, 'particles_05um': 0, 'particles_03um': 0, 'CO2_ppm': 0, }

#Initialisation client MQTT
client = mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)
client.connect(THINGSBOARD_HOST, 1883, 60)

client.loop_start()

while True:
    ser.write(1)#L'écriture vers la Nucleo déclenche la lecture des capteurs
    received_data= ser.read_until()
    ser.flush()
    
    data_to_send = received_data.decode("utf-8")
    data_to_send = data_to_send.replace('\r\n','')
    data_to_send = data_to_send.split(",")
    print(data_to_send)
    
    #Condition pour éviter d'envoyer des listes vides
    if len(data_to_send) != 1:
        
        #Ecriture des données dans le Json
        sensor_data['particles_100um'] = data_to_send[0]
        sensor_data['particles_50um'] = data_to_send[1]
        sensor_data['particles_25um'] = data_to_send[2]
        sensor_data['particles_10um'] = data_to_send[3]
        sensor_data['particles_05um'] = data_to_send[4]
        sensor_data['particles_03um'] = data_to_send[5]
        sensor_data['CO2_ppm'] = data_to_send[6]
        
        #Envoi des données vers Thingsboard
        client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)
        
    sleep(5)#Intervalle entre chaque lecture de données






