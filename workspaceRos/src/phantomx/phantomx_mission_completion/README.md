# Mission Completion

### Description

Ce pakage permet d'accomplir la mission de détection de fissure de la grotte.

### Lancer la mission

Pour lancer la mission il faut lancer le fichier launch `phantomx_mission.launch`:

	$ roslaunch phantomx_mission_completion phantomx_mission.launch
	
Ce fichier launch lance la simulation gazebo ainsi que les nodes permettant l'exécution de la mission.


### Scripts

Les codes développés sont présents dans le dossier `src/`. Voici une description sommaire des scripts:

* `controller.py`: implémentation de la navigation du robot afin d'explorer la grotte en fonction des distances aux murs.

* `crackDetection.py`: fonctions permettant de détecter une fissure dans une image

