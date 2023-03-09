import paho.mqtt.client as mqtt
from ctypes import cdll

libC = cdll.LoadLibrary('./clib.so')

def on_connect(client, userdata, flags, rc):
    client.subscribe("esp8266")

def on_message(client, userdata, message):
    print("Received message " + str(message.payload))
    #clear_message = unpack('=H2BH2B6d5f2i?BH',message.payload)
    libC.save_data(message.payload)
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect('localhost',1883,60)
client.loop_start()
