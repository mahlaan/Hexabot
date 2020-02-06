# Hexabot

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique de l'UV 5.8 à l'ENSTA Bretagne en 2020.


## Lancer la simulation

### Dépendences

Package ros: `Effort controllers`

~~~shell
$ sudo apt-get install ros-melodic-effort-controllers
~~~

Package ros: `Openni-camera` (Kinect module)

	$ sudo apt-get install ros-melodic-openni-camera

Package ros: `Hector-mapping` (SLAM navigation)

	$ sudo apt-get install ros-melodic-hector-mapping
	
Package ros: `Hector-imu-attitude` (IMU rectification)

	$ sudo apt-get install ros-melodic-hector-imu-attitude-to-tf

Package ros: `Map Server` (sauvergarde la map)

	$ sudo apt-get install ros-melodic-map-server

	
### Démarrer la simulation

#### Controler manuellement l'hexapode

	$ roslaunch phantomx_gazebo phantomx_gazebo.launch
	-> 'Play' la simulation pour lancer le topic '/phantomx/cmd_vel'
	$ rosrun rqt_robot_steering rqt_robot_steering
	
#### Accomplissement de la mission

Le package associé à l'accomplissement de la mission est `workspaceRos/src/phantomx/phantomx_mission_completion`. Vous trouverez plus d'informations dans le  [README](workspaceRos/src/phantomx/phantomx_mission_completion/README.md) du package. Pour lancer la mission il faut lancer le fichier launch `phantomx_mission.launch`:

	$ roslaunch phantomx_mission_completion phantomx_mission.launch

Cette simulation ne donnera pas de rendu via RVIZ, pour lancer le RVIZ correspondant:

	$ roslaunch phantomx_mission_completion phantomx_mission_rviz.launch
	
Pour sauvegarder la map générée:

	$ rosrun map_server map_saver -f MapName
	
	
## Groupe

### Membres

* Anouar MAHLA
* Clément BICHAT
* Kévin BEDIN
* Erwann LANDAIS
* Nathan FOURNIOL
* Aurélien GRENIER

### Gestion de projet

*  Lien vers le [Taiga](https://tree.taiga.io/project/grenieau-uv-48-hexabot/us/1?milestone=251463) du groupe.

* Voir le dossier `reports/` pour voir les fichiers *goals* et *debrief*. 


## Structure du dépôt

### Workspace ROS

Le dossier `workspaceRos` est la racine du workspace `catkin` pour les packages ROS. Ces derniers doivent être placés sous `workspaceRos/src`.    
Consulter le [README](workspaceRos/README.md) du workspace pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS et Gazebo](docs/MemoROS.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](reports/GoalsTemplate.md) et de [rétrospectives](reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
