# Turtle (ROS 2)

Ce README documente uniquement le contenu du répertoire `turtle/`.

## Table des matières
- [Vue d'ensemble](#vue-densemble)
- [Dossiers et implémentation](#dossiers-et-implémentation)
- [Packages présents](#packages-présents)
- [Pré-requis](#pré-requis)
- [Build](#build)
- [1) DrawSquare (serveur d'action)](#1-drawsquare-serveur-daction)
- [2) FSM couleur/bords](#2-fsm-couleurbords)
- [3) DrawBoundaries (suivi de contour)](#3-drawboundaries-suivi-de-contour)
- [4) Mode clavier (pause + reprise)](#4-mode-clavier-pause--reprise)
- [5) Contrats d'actions](#5-contrats-dactions)

## Vue d'ensemble
Le dossier `turtle/` contient 5 packages ROS 2 autour de `turtlesim` :
- un serveur d'action pour dessiner un carré (`DrawSquare`),
- un contrôleur FSM réactif à la couleur rouge et aux bords,
- un serveur d'action de suivi de contour (`DrawBoundaries`),
- un nœud clavier pour interrompre/reprendre le suivi de contour.

## Dossiers et implémentation
- `turtle/` : racine des packages ROS 2 liés à `turtlesim` pour ce projet.
- `turtle/turtle_square_interfaces/` : package d'interface qui définit l'action `DrawSquare`.
- `turtle/turtle_square_interfaces/action/` : fichier `DrawSquare.action` (Goal/Result/Feedback du tracé de carré).
- `turtle/turtle_square_controller/` : package applicatif qui implémente le serveur d'action `draw_square`.
- `turtle/turtle_square_controller/src/` : logique C++ du nœud `turtle_square_server` (déplacement et rotations de la tortue).
- `turtle/turtle_fsm_controller/` : package FSM qui pilote la tortue selon la pose et la couleur.
- `turtle/turtle_fsm_controller/src/` : implémentation du nœud `turtle_fsm_node` et de ses états.
- `turtle/turtle_fsm_controller/launch/` : launch `turtle_fsm.launch.py` (démarre `turtlesim` + le nœud FSM).
- `turtle/turtle_boundaries_interfaces/` : package d'interface qui définit l'action `DrawBoundaries`.
- `turtle/turtle_boundaries_interfaces/action/` : fichier `DrawBoundaries.action` (Goal/Result/Feedback du suivi de contour).
- `turtle/turtle_drawing_boundaries/` : package de suivi de contour et de contrôle manuel.
- `turtle/turtle_drawing_boundaries/src/` : implémentation de `turtle_boundaries_node` (serveur d'action) et `turtle_boundaries_keyboard` (contrôle clavier).
- `turtle/turtle_drawing_boundaries/launch/` : launches standard et clavier (`draw_boundaries.launch.py`, `draw_boundaries_with_keyboard.launch.py`).
- `turtle/turtle_drawing_boundaries/CR_Assignment2.txt` : note technique décrivant la logique d'interruption/reprise et la stratégie de réacquisition.

## Packages présents
- `turtle_square_interfaces` : définition de l'action `DrawSquare`.
- `turtle_square_controller` : nœud `turtle_square_server`.
- `turtle_fsm_controller` : nœud `turtle_fsm_node` + launch `turtle_fsm.launch.py`.
- `turtle_boundaries_interfaces` : définition de l'action `DrawBoundaries`.
- `turtle_drawing_boundaries` : nœuds `turtle_boundaries_node` et `turtle_boundaries_keyboard` + launches.

## Pré-requis
- ROS 2 Jazzy installé (`/opt/ros/jazzy`).
- `turtlesim` installé.
- Terminal Linux pour le mode clavier interactif.
- Optionnel : `gnome-terminal` (utilisé par le launch clavier par défaut).

## Build
Depuis la racine du dépôt :

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths turtle --packages-select \
  turtle_square_interfaces turtle_square_controller \
  turtle_fsm_controller \
  turtle_boundaries_interfaces turtle_drawing_boundaries
source install/setup.bash
```

## 1) DrawSquare (serveur d'action)
Lance 3 terminaux.

Terminal A :
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

Terminal B :
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtle_square_controller turtle_square_server
```

Terminal C (envoi d'un goal) :
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare \
  "{side_length: 2.0, speed: 1.0}" --feedback
```

## 2) FSM couleur/bords
Launch recommandé :

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py
```

Le launch démarre :
- `turtlesim_node`
- `turtle_fsm_node`

États implémentés :
- `WAITING_DATA`
- `FORWARD`
- `BOUNDARY_TURN`
- `BOUNDARY_ESCAPE`
- `RED_REVERSE`
- `RED_TURN`
- `RED_BOOST`

Exemple de réglage :
```bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py \
  red_boost_duration:=0.6 \
  red_event_cooldown:=1.0 \
  boundary_margin:=0.6 \
  boundary_heading_tolerance:=0.35
```

## 3) DrawBoundaries (suivi de contour)
Mode standard (sans clavier) :

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtle_drawing_boundaries draw_boundaries.launch.py \
  lost_wall_grace_s:=0.45 \
  resume_reacquire_timeout_s:=8.0 \
  resume_reacquire_radius:=0.22
```

Dans un autre terminal, envoi du goal :

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal /draw_boundaries turtle_boundaries_interfaces/action/DrawBoundaries \
  "{margin: 0.5, speed: 1.0, clockwise: true}" --feedback
```

## 4) Mode clavier (pause + reprise)
Launch avec nœud clavier :

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtle_drawing_boundaries draw_boundaries_with_keyboard.launch.py
```

Puis envoi du goal `DrawBoundaries` comme ci-dessus.

Touches clavier :
- `Espace` : pause/reprise du suivi automatique.
- `Flèches` ou `ZQSD/WASD` : déplacement manuel.
- `x` ou `0` : stop manuel.

Topics utilisés pour l'interaction :
- `/turtle_boundaries/pause_toggle` (`std_msgs/Empty`)
- `/turtle_boundaries/manual_cmd` (`geometry_msgs/Twist`)

## 5) Contrats d'actions
`DrawSquare` (`turtle_square_interfaces/action/DrawSquare`) :
- Goal : `side_length`, `speed`
- Result : `success`
- Feedback : `remaining_distance`

`DrawBoundaries` (`turtle_boundaries_interfaces/action/DrawBoundaries`) :
- Goal : `margin`, `speed`, `clockwise`
- Result : `success`, `perimeter`, `message`
- Feedback : `remaining_distance`, `side_index`
