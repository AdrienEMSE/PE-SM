
### Utilisation de mosquitto
Le serveur mosquitto s'exécute automatiquement lors de l'allumage de la raspberry.
Mettre la raspberry en mode "Hotspot" pour permettre à l'ESP8266 de se connecter.

lire ce qui est envoyé:
mosquitto_sub -d -t esp8266

tester le serveur:
mosquitto_pub -d -t esp8266 -m "It's ALIVE !"


Changer les paramètres de MQTT (par exemple ajouter mdp) :

sudo nano /etc/mosquitto/mosquitto.conf
