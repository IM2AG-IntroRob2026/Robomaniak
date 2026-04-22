# Robomaniak

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

Ce README.md se base uniquement sur le contenu du repertoire `/src/robot_vision/`.

## Table des matières
- [Robomaniak](#robomaniak)
- [Fonctionnalités principales](#fonctionnalité-principales)
- [Installation](#installation)
- [Documentation](#documentation)
- [Conception](#conception)
- [Docker](#docker)
- [Feedback](#feedback)
- [Auteurs](#auteurs)

## Robomaniak

**Robomaniak** est un projet ROS 2 pour le robot **iRobot Create 3** qui combine vision, suivi de personne, téléopération, commandes vocales et docking automatique.

L’objectif est de proposer un robot capable de passer facilement d’un contrôle manuel à des comportements plus autonomes, via une architecture modulaire basée sur plusieurs nœuds spécialisés (vidéo, détection, suivi, écoute, orchestrateur Behavior Tree, détection du dock).

Le système fonctionne autour de trois modes (`TELEOP`, `FOLLOW`, `LISTEN`) afin d’adapter le comportement du robot au contexte d’utilisation.

## Fonctionnalité principales
- Dirigeable à partir d'un clavier
- Dirigeable à partir d'une mannette
- Configurer des nouveaux mots
- Obéir à des ordres prédéfinis
- Détecter une personne
- Suivre une personne
- Jouer de la musique
- Changer de couleurs de leds en fonction de la situation
- Réinitialisation en temps réels
- Se docker de n'importe où
- S'undocker

## Installation

Les commandes ci-dessous permettent de compiler puis lancer le projet dans Docker.

- Cette commande construit l'image Docker à partir du `Dockerfile` du dépôt et la nomme `ros2` :

```bash
sudo docker buildx build -t ros2 .
```

- Cette commande démarre un conteneur temporaire avec le dossier courant monté dans `/ros2_ws`, puis compile le workspace ROS2 avec `colcon build`.

```bash
sudo docker run --rm -v $(pwd):/ros2_ws -w /ros2_ws ros2 \
          /ros_entrypoint.sh /bin/bash -c "source /root/create3_ws/install/setup.bash && colcon build"
```

- Cette commande lance le pipeline complet du robot dans Docker (vidéo, détection, suivi, behavior tree, écoute et dock detector), avec :
    - `--net=host` pour partager le réseau de la machine hôte,
    - `--device ...` pour accéder au son, à la manette et au périphérique HID,
    - les variables CycloneDDS pour la communication ROS2.

```bash
sudo docker run -it --rm \
    --net=host \
    --privileged \
    --device /dev/snd \
    --device /dev/input \
    --device /dev/hidraw0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml \
    -v $(pwd):/ros2_ws \
    -w /ros2_ws \
    ros2 /bin/bash -c "source install/setup.bash && ros2 launch robot_vision robot_complet.launch.py"
```

- Cette commande lance uniquement le nœud `teleop_node` dans Docker pour piloter le robot manuellement (clavier/manette).
  Le teleop a besoin d'être dans un terminal à part pour configurer son terminal.

```bash
sudo docker run -it --rm \
    --net=host \
    --privileged \
    --device /dev/snd \
    --device /dev/input \
    --device /dev/hidraw0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///ros2_ws/cyclonedds.xml \
    -v $(pwd):/ros2_ws \
    -w /ros2_ws \
    ros2 /bin/bash -c "source install/setup.bash && ros2 run robot_vision teleop_node"
```

- Alternativement, pour éviter la compilation manuelle de l'image qui peut prendre du temps, il est possible de récuperer l'image docker généré par la CD github avec la commande :

```bash
docker pull ghcr.io/im2ag-introrob2026/robomaniak:latest
```

## Documentation
![Github Pages](https://img.shields.io/badge/github%20pages-121013?style=for-the-badge&logo=github&logoColor=white)
![Doxygen](https://img.shields.io/badge/doxygen-2C4AA8?style=for-the-badge&logo=doxygen&logoColor=white)

Une documention a été mise en place grâce à l'outils doxygen.

Vous trouverez la documentaion en ligne à cette adresse : [Robomaniak Doxygen](https://im2ag-introrob2026.github.io/Robomaniak/)

## Conception

L’architecture du projet est organisée autour de **7 nœuds ROS2**, tous regroupés dans le package `robot_vision/`, qui contient l’intégralité de la logique du robot : **Flux Vidéo**, **Détection**, **Suivi**, **Téléop**, **Écoute**, **Behavior Tree** (chef d’orchestre) et **Détecteur de Dock**.

Le robot fonctionne selon **3 modes** :
- `TELEOP`
- `FOLLOW`
- `LISTEN`

Le changement de mode peut être déclenché via :
- le bouton `X` de la manette,
- la touche `Espace` au clavier,
- des commandes vocales.

### `video_node` (`video_node.hpp` / `video_node.cpp`)
Ce nœud lit un flux MJPEG distant et le publie en `sensor_msgs::msg::Image` sur le topic `/camera/image_raw`, afin d’alimenter le reste du pipeline.

Pour limiter la latence, un mécanisme de *frame dropping* est utilisé avec un buffer de taille 1, ce qui permet de supprimer les images trop anciennes.

### `detection_node`
Ce nœud détecte un ou plusieurs humains dans l’image. Il :
- écoute le topic `/camera/image_raw`,
- pré-traite les images,
- réalise l’inférence avec **YOLOv8s** via une session **ONNX Runtime**,
- applique un post-traitement.

Un algorithme **NMS** (*Non-Maximum Suppression*) a également été implémenté pour réduire les superpositions de boîtes de détection.

### `follow_node`
Ce nœud gère le suivi d’un humain avec :
- le choix et le verrouillage d’une cible,
- le déplacement du robot pour conserver cette cible dans la scène.

La commande repose sur :
- `cx` : centre horizontal de la boîte englobante (à recentrer),
- `h` : hauteur de la boîte (doit représenter un pourcentage cible de la hauteur de l’image).

Pour conserver le verrouillage d’une image à l’autre, le nœud compare la distance entre le centre de la boîte verrouillée à l’instant précédent et les centres détectés sur l’image courante.

### `teleop_node`
Ce nœud assure la conduite manuelle, disponible au clavier et à la manette. Son rôle est de transformer les actions utilisateur en commandes de vitesse ROS2.

Il :
- lit les entrées utilisateur (clavier via terminal, manette via `libevdev`),
- publie les demandes de déplacement sur `/teleop/cmd_vel`,
- publie les demandes de changement de mode sur `/teleop/mode_switch`,
- permet de déclencher les actions `/dock` et `/undock`,
- donne la priorité à la manette lorsqu’elle est active.

### `listen_node`
Ce nœud récupère les commandes vocales avec **Sherpa-ONNX** et **PortAudio**.

Il :
- capture le flux micro,
- analyse les entrées et les associe à des actions lorsqu’un mot-clé est reconnu,
- publie les ordres de mouvement sur `/listen/command`.

Pour éviter de limiter le choix des mots-clés, ceux-ci sont découpés en tokens, puis fournis à Sherpa pour le *Keyword Spotting*.

### `bt_manager_node` (Behavior Tree)
Ce nœud joue le rôle de chef d’orchestre. Il centralise les différentes sources de commandes et n’en laisse passer qu’une seule vers `/cmd_vel`.

La logique est implémentée avec un **Behavior Tree**, exécuté périodiquement (50 Hz par défaut). En pratique, il écoute les autres nœuds et choisit la commande à publier selon le mode actif.

Modes pris en charge :
- `TELEOP`
- `FOLLOW`
- `LISTEN`

Le mode Teleop a deux sous mode pour le dock et le undock.

Au démarrage, le mode actif est `TELEOP`. Lorsqu’une demande est reçue sur `/teleop/mode_switch`, le nœud effectue un cycle dans l’ordre :
`TELEOP -> FOLLOW -> LISTEN -> TELEOP`.

Il gère également la machine à états de docking pour permettre au robot de se dock depuis une distance importante.

### `dock_detector_node`
Ce nœud détecte un marqueur **ArUco** et calcule sa position relative au robot.

Cette position est utilisée pour approcher suffisamment le dock, afin de laisser ensuite les capteurs infrarouges du robot finaliser l’opération (action `/dock`).

Pour obtenir une estimation de position fiable, une calibration caméra est nécessaire à l’aide d’un motif damier dont la taille des carrés est connue.

## Docker
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)

A partir du moment où docker est installé, vous pouvez lancer tous les noeuds dans n'importe quel environnement.

Le docker utilise `ros2 iron` pour compiler

## Feedback
Nous voulons souligner la grande participation d'Antoine qui a été le pilier de ce projet.

## Auteurs
- Antoine Patron
- Lisa Zannettacci
- Adrien Coste
- Justine Reat

-----------------

[Back to top](#robomaniak)