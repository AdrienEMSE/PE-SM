## Obtenir les données des capteurs indoor (Nouvelle version):

Lors de l'allumage de la Raspberry, brancher en USB la Nucleo. La LED rouge va rester allumée au lieu de clignoter, signifiant que la carte est prête.

Télécharger sur la Raspberry Pi le script "get_indoor_data.py", et l'exécuter. Le programme va automatiquement écrire sur ThingsBoard les données des capteurs.

Le délai entre chaque enregistrement de données peut être modifié dans le script.



## Obtenir les données des capteurs indoor (Ancienne version):

### Utilisation de mosquitto
Le serveur mosquitto s'exécute automatiquement lors de l'allumage de la raspberry.
Mettre la raspberry en mode "Hotspot" pour permettre à l'ESP8266 de se connecter.

Pour lire ce qui est envoyé, taper dans le terminal:

<code>mosquitto_sub -d -t esp8266</code>

Pour tester le serveur, taper:
<code>mosquitto_pub -d -t esp8266 -m "It's ALIVE !"</code>


Changer les paramètres de MQTT (par exemple ajouter mdp) :

<code>sudo nano /etc/mosquitto/mosquitto.conf</code>

### Retranscription des messages MQTT en csv

Le script python <strong>MQTT_to_csv.py</strong> enregistre les données reçues sur le serveur MQTT dans le fichier <strong>outdoor_station.csv</strong>.

<p>Le script utilise la librairie <strong>clib.so</strong> compilée à partir des codes en C++ <strong>msg_ESP.cpp</strong> & <strong>msg_ESP.h</strong>.</p>
Cette librairie doit être dans le même dossier que le script en python.

Pour compiler la librairie:

<code>g++ -shared -o clib.so msg_ESP.cpp</code>

### Enregistrement des données des capteurs 

Le script python <strong>TxRx_Nucleo.py</strong> permet de récuperer à intervalle régulier (modifiable dans le script) les données des capteurs connectées à la Nucleo L432KC. Ces données sont enregistrées dans <strong>indoor_sensors.csv</strong>.
