Ceci est le code de la Nucleo L432KC installée sur le serveur.

C'est elle qui va obtenir les données du détecteur de CO2 et de qualité de l'air, et les envoyer en UART à la Raspberry.

## Téléversement
Sur un PC, cloner ce dossier et utiliser PlatformIO pour téléverser en USB sur la Nucleo

## Activation
Lors de l'allumage de la Raspberry Pi la Nucleo clignote en rouge: elle n'est pas encore activée.

Pour l'activer, il faut la brancher puis débrancher en USB. La LED reste alors constamment allumée, signifiant qu'elle est prête.

La Nucleo reste dans cet état jusqu'à ce que la Raspberry soit débranchée.
