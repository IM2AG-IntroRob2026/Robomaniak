# Robomaniak - Turtle Square (ROS 2)

Ce projet contient (actuellement) un serveur d'action ROS 2 qui fait dessiner un carre a `turtlesim`.

## Arborescence utile

- `ros2_ws/src/turtle_square_interfaces`: definition de l'action `DrawSquare`
- `ros2_ws/src/turtle_square_controller`: noeud C++ `turtle_square_server`

## 1) Build du workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select turtle_square_interfaces turtle_square_controller
```

## 2) Lancer les noeuds

Ouvre **3 terminaux**.

### Terminal A - turtlesim

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

### Terminal B - serveur d'action

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 run turtle_square_controller turtle_square_server
```

### Terminal C - client d'action (envoi d'un goal)

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare "{side_length: 2.0, speed: 1.0}" --feedback
```

## 3) Verifier que tout tourne

Dans un terminal source:

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 action info /draw_square
```

## 4) Commandes utiles

Voir la pose courante:

```bash
ros2 topic echo /turtle1/pose
```

Voir les commandes de vitesse publiees:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Envoyer un autre goal:

```bash
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare "{side_length: 1.5, speed: 0.8}" --feedback
```
