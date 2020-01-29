# Mission Completion

### Description

Ce pakage permet d'accomplir la mission de détection de fissure de la grotte.

### Lancer la mission

Pour lancer la mission (seulement gazebo) il faut lancer le fichier launch `phantomx_mission.launch`:

	$ roslaunch phantomx_mission_completion phantomx_mission.launch
	
Ce fichier launch lance la simulation gazebo ainsi que les nodes permettant l'exécution de la mission. En particulier les packages `hector_mapping` et `hector_imu_attitude_to_tf`.
	
Si l'on souhaite Rviz (déjà configuré) en plus:

	$ roslaunch phantomx_mission_completion phantomx_mission_rviz.launch
	
Il pourra ếtre nécessaire de changer la variable d'environnement suivante:

        export LC_NUMERIC="en_US.UTF-8"

### Infos Topics spécifiques au package

* `/phantomx/lidar`: Scan du lidar

* `/slam_out_pose`: Position estimée du robot par l'algorithme SLAM (_hector_mapping_)

* `/map`: Map générée par l'agorithme de SLAM

* `/phantomx/crack_image`: Image avec les fissures détectées (canal depth traité par un masque).

* `/phantomx/crack_markers`: Liste de Marker représentant les fissures visualisées.

### Scripts

Les codes développés sont présents dans le dossier `src/`. Voici une description sommaire des scripts:

* `controller.py`: implémentation de la navigation du robot afin d'explorer la grotte en fonction des distances aux murs.

* `crackDetection.py`: fonctions permettant de détecter une fissure dans une image.

* `crackPub.py`: crée un noeud ROS permettant de reprendre les images de la caméra et de publier chaque seconde les images dans lesquelles des fissures ont été détectées.

* `crackMapping.py`: crée un noeud ROS permettant de publier les fissures détectées dans le référentiel de la grotte.
