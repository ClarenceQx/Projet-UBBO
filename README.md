# Projet-UBBO

Le dossier "Planification de trajectoire" contient le notebook contenant le code détaillé et expliqué relatif à l'implémentation de la méthode des potentiels.
Il contient également le fichier my_map.pgm, qui est l'image de la carte locale utilisée dans le notebook. Le fichier my_map.yaml est une description de cette image.

Le dossier "Reconnaissance d'objets par IA" contient le notebook du code relatif au chargement du réseau ResNeT, pré-entrainé sur ImageNet. Les 3 images également présentes dans le dossier sont celles utilisées dans le notebook pour illustrer le fonctionnement du réseau de neurones ResNet en classification.

Le dossier "src" contient les différents packages permettant le déplacement du robot et sa localisation. Parmi ces derniers, le package "global_pose" permet d'obtenir la position globale du robot sur la carte enregistrée à partir du lidar. Le package "icp_pkg" détermine les déplacements du robot de manière itérative à partir des données du lidar.
