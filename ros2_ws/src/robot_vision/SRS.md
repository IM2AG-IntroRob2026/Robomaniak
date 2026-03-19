# Projet Robot Patrouille / Suivi — iRobot Create 3 + ROS2

---

## Table des matières

1. [Introduction](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#1-introduction)
2. [Description générale](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#2-description-g%C3%A9n%C3%A9rale)
3. [Architecture système](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#3-architecture-syst%C3%A8me)
4. [Exigences fonctionnelles](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#4-exigences-fonctionnelles)
5. [Exigences non fonctionnelles](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#5-exigences-non-fonctionnelles)
6. [Composants logiciels et bibliothèques](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#6-composants-logiciels-et-biblioth%C3%A8ques)
7. [Interfaces](https://claude.ai/chat/9d0c2ffc-1be0-45a7-854e-210ba68cda9d#7-interfaces)

---

## 1. Introduction

### 1.1 Périmètre du projet

Le système doit permettre à un robot mobile de :

- Patrouiller de manière autonome dans un environnement intérieur.
- Détecter la présence d'un humain.
- Reconnaître, si possible, un individu spécifique fourni en référence.
- Suivre l'individu détecté.
- Être repris en contrôle manuel à tout moment, et repassé en mode automatique.

---

## 2. Description générale

### 2.1 Matériel

| Composant          | Description                                                   |
| ------------------ | ------------------------------------------------------------- |
| Robot              | iRobot Create 3                                               |
| Capteurs embarqués | Collision (bumper), vide (cliff), infrarouge                  |
| Support caméra     | Support physique posé sur le robot, accueillant un smartphone |
| Smartphone         | Caméra, stream MJPEG via réseau WiFi                          |

### 2.2 Logiciel de base

| Composant | Version / distribution |
| --------- | ---------------------- |
| ROS2      | Iron                   |
| OS        | (K)Ubuntu              |
| Langage   | C++ ou Python          |

### 2.3 Hypothèses et contraintes initiales

- Le robot évolue dans un environnement intérieur plat.
- La connexion entre le PC et le robot se fait via WiFi.

---

## 3. Architecture système

### 3.1 Modes de fonctionnement

```
         ┌───────────────────────────────────────────┐
         │                                           │
  ┌──────▼──────┐   détection     ┌──────────────┐   │
  │  PATROUILLE │───humain ──────▶│    SUIVI     │   │
  │  (autonome) │                 │  (autonome)  │   │
  └──────┬──────┘                 └──────┬───────┘   │
         │                               │           │
  override manuel                override manuel     │
         │                               │           │
         └──────────────┬────────────────┘           │
                        ▼                            │
                  ┌───────────┐                      │
                  │   TELEOP  │                      │
                  │  (manuel) │                      │
                  └─────┬─────┘                      │
                        │    commande "auto"         │
                        └────────────────────────────┘
```

---

## 4. Exigences fonctionnelles

### 4.1 Mode Téléopération (TELEOP)

| ID    | Exigence                                                                               | Priorité | Réalisé |
| ----- | -------------------------------------------------------------------------------------- | -------- | ------- |
| F-T01 | Le système doit permettre de contrôler le robot au clavier depuis le PC.               | Haute    |         |
| F-T02 | Le passage en mode TELEOP doit être possible à tout moment depuis n'importe quel mode. | Haute    |         |
| F-T03 | Le passage du mode TELEOP vers le mode automatique (PATROUILLE) doit être possible.    | Haute    |         |
| F-T04 | Le système doit permettre de contrôler le robot avec une manette gamepad.              | Basse    |         |
| F-T05 | La manette doit permettre de contrôler vitesse linéaire et angulaire du robot.         | Basse    |         |

### 4.2 Mode Patrouille (PATROL)

| ID    | Exigence                                                                                         | Priorité | Réalisé |
| ----- | ------------------------------------------------------------------------------------------------ | -------- | ------- |
| F-P01 | Le robot doit se déplacer de manière autonome dans la pièce selon un plan de patrouille. `[TBD]` | Haute    |         |
| F-P02 | Le robot doit éviter les chutes via ses capteurs de vide.                                        | Haute    |         |
| F-P03 | Le robot doit analyser le flux caméra en continu durant la patrouille pour détecter un humain.   | Haute    |         |
| F-P04 | Le robot doit éviter les collisions via ses capteurs infrarouges                                 | Moyenne  |         |
| F-P05 | Le robot doit être capable de cartographier la pièce (SLAM).                                     | Basse    |         |

### 4.3 Détection humaine

| ID    | Exigence                                                                           | Priorité | Réalisé |
|-------|------------------------------------------------------------------------------------|----------|---------|
| F-D01 | Le système doit détecter la présence d'au moins un humain dans le flux caméra.     | Haute    | X       |
| F-D02 | Le système doit utiliser le frame dropping pour minimiser la latence de détection. | Haute    | X       |
| F-D03 | L'inférence doit être accélérée par GPU.                                           | Moyenne  |         |

### 4.4 Reconnaissance de personne spécifique

| ID    | Exigence                                                                                   | Priorité | Réalisé |
| ----- | ------------------------------------------------------------------------------------------ | -------- | ------- |
| F-R01 | Le système doit pouvoir recevoir une ou plusieurs images de référence d'un individu cible. | Moyenne  |         |
| F-R02 | Le système doit calculer un embedding de visage pour l'individu de référence au démarrage. | Moyenne  |         |
| F-R03 | Le système doit comparer l'embedding des visages détectés à celui de la référence.         | Moyenne  |         |
| F-R04 | Si la similarité dépasse un seuil configurable, l'individu est considéré comme reconnu.    | Moyenne  |         |

### 4.5 Mode Suivi (FOLLOW)

| ID    | Exigence                                                                                                       | Priorité | Réalisé |
| ----- | -------------------------------------------------------------------------------------------------------------- | -------- | ------- |
| F-S01 | Lors de la détection d'un humain (ou de l'individu cible), le robot doit passer automatiquement en mode SUIVI. | Haute    |         |
| F-S02 | Le robot doit maintenir une distance raisonnable avec la cible tout en la suivant. `[distance TBD]`            | Haute    |         |
| F-S03 | Le robot doit jouer une mélodie une unique fois à l'entrée dans le mode SUIVI.                                 | Haute    |         |
| F-S04 | La lumière du robot doit adopter un état visuel spécifique pendant toute la durée du mode SUIVI.               | Haute    |         |
| F-S05 | Si la cible est perdue de vue, le robot repasse en mode PATROUILLE.                                            | Haute    |         |
| F-S06 | Le robot doit continuer à éviter les obstacles et les chutes en mode SUIVI.                                    | Moyenne  |         |

---

## 5. Exigences non fonctionnelles

| ID    | Exigence                                                                                             | Réalisé |
| ----- | ---------------------------------------------------------------------------------------------------- | ------- |
| NF-01 | Le système doit fonctionner sans intervention humaine une fois lancé (hors téléop).                  |         |
| NF-02 | La latence entre la capture caméra et la décision de navigation doit rester inférieure à `[TBD]` ms. |         |
| NF-03 | Le code doit être optimisé en termes de mémoire et de performances.                                  |         |
| NF-04 | Le code doit être structuré en nœuds ROS2 distincts et bien séparés.                                 |         |
| NF-05 | Un README décrivant le lancement du système doit être fourni.                                        |         |

---

## 6. Composants logiciels et bibliothèques

### 6.1 Robot et navigation

| Bibliothèque         | Rôle                                                    |
| -------------------- | ------------------------------------------------------- |
| `rclcpp`             | Client ROS2 C++                                         |
| **Nav2**             | Navigation autonome, planification de chemin, waypoints |
| **slam_toolbox**     | Cartographie SLAM _(optionnel)_                         |
| iRobot Create 3 pkgs | Topics et actions natifs du robot                       |

### 6.2 Vision et inférence

| Bibliothèque     | Rôle                                                        | Réalisé |
| ---------------- | ----------------------------------------------------------- | ------- |
| **OpenCV**       | Capture MJPEG, preprocessing, frame dropping                |         |
| **ONNX Runtime** | Inférence YOLO + modèle d'embedding (accélération GPU ROCm) |         |

### 6.3 Machine d'états

| Bibliothèque            | Rôle                                             | Réalisé |
| ----------------------- | ------------------------------------------------ | ------- |
| **BehaviorTree.CPP v4** | Orchestration des modes PATROL / FOLLOW / TELEOP |         |

### 6.4 Téléopération

| Bibliothèque / Outil             | Rôle                              | Réalisé |
| -------------------------------- | --------------------------------- | ------- |
| `teleop_twist_keyboard` (ROS2)   | Contrôle clavier (base existante) |         |
| **SDL2** ou **linux/joystick.h** | Lecture manette gamepad `[TBD]`   |         |

---

## 7. Interfaces

### 7.1 Flux vidéo smartphone

- L'application **IP Webcam** (Android) expose un flux MJPEG via HTTP sur le réseau local.
- Un nœud ROS2 (`video_node`) capture ce flux via `cv::VideoCapture` et publie sur le topic `/camera/image_raw` (`sensor_msgs/Image`).
- Le buffer est limité au minimum (`CAP_PROP_BUFFERSIZE = 1`) et plusieurs `grab()` sont appelés avant `retrieve()` pour garantir la frame la plus récente.

### 7.2 Capteurs du robot

|Capteur|Topic ROS2|Utilisation|
|---|---|---|
|Bumper|`/bumper_event`|Collision frontale|
|Cliff sensors|`/cliff_intensity`|Détection vide / chute|
|Infrarouge|`/ir_intensity`|Évitement d'obstacles proches|
|Odométrie|`/odom`|Navigation, suivi de trajectoire|

### 7.3 Commandes vers le robot

|Topic|Type|Utilisation|
|---|---|---|
|`/cmd_vel`|`geometry_msgs/Twist`|Consignes de vitesse|
|`/audio`|`irobot_create_msgs`|Lecture de mélodie|
|`/led_color`|`irobot_create_msgs`|Contrôle des LEDs|

### 7.4 Interfaces opérateur

- **Clavier** : contrôle direct via `teleop_twist_keyboard` ou nœud custom.
- **Manette gamepad** : nœud C++ dédié publiant sur `/cmd_vel`. `[TBD : lib et protocole]`
- **Commandes de mode** : topics ou services ROS2 dédiés pour basculer entre TELEOP / AUTO. `[TBD : design exact]`

---
