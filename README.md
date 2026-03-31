# Robomaniak - Turtle Square et FSM (ROS 2)

Ce projet contient:

- un serveur d'action ROS 2 qui fait dessiner un carre a `turtlesim`
- un controleur FSM ROS 2 avec transitions basees sur la couleur rouge

## Arborescence utile

- `ros2_ws/src/turtle_square_interfaces`: definition de l'action `DrawSquare`
- `ros2_ws/src/turtle_square_controller`: noeud C++ `turtle_square_server`
- `ros2_ws/src/turtle_fsm_controller`: noeud C++ `turtle_fsm_node` + launch file

## 1) Build du workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select turtle_square_interfaces turtle_square_controller turtle_fsm_controller
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

## 5) Lancer le FSM (commande unique recommandee)

Cette commande demarre automatiquement `turtlesim_node` et `turtle_fsm_node`:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select turtle_fsm_controller
source install/setup.bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py \
  red_boost_duration:=0.35 \
  red_event_cooldown:=0.6 \
  red_reverse_duration:=0.5 \
  red_turn_duration:=1.1 \
  boundary_margin:=0.6 \
  boundary_heading_tolerance:=0.5
```

## 6) Etats FSM implementes

- `WAITING_DATA`: attend les premiers messages pose/couleur
- `FORWARD`: avance en ligne droite
- `BOUNDARY_TURN`: tourne si la tortue est proche du bord
- `BOUNDARY_ESCAPE`: se degage du bord en se reorientant vers le centre
- `RED_REVERSE`: recule quand du rouge est detecte
- `RED_TURN`: tourne apres le recul
- `RED_BOOST`: avance a vitesse double pendant une duree courte

## 7) Ajouter des murs rouges dans turtlesim

Lance d abord `turtlesim` (ou le launch FSM), puis dans un autre terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash

# Creer turtle2
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0.0, name: turtle2}"

# Stylo rouge epais
ros2 service call /turtle2/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 15, 'off': 0}"

# Tracer un rectangle (murs rouges)
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 10.5, y: 1.0, theta: 0.0}"
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 10.5, y: 10.5, theta: 1.57}"
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 1.0, y: 10.5, theta: 3.14}"
ros2 service call /turtle2/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 1.0, y: 1.0, theta: -1.57}"
```

Optionnel:

```bash
# Supprimer turtle2 mais garder les traits rouges
ros2 service call /kill turtlesim/srv/Kill "{name: turtle2}"

# Effacer le fond (supprime les murs rouges)
ros2 service call /clear std_srvs/srv/Empty "{}"
```

## 8) Reglages utiles FSM

Si la tortue tourne trop souvent pres des bords:

```bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py boundary_margin:=0.5
```

Si la reaction rouge est trop agressive:

```bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py red_boost_duration:=0.6 red_event_cooldown:=1.2
```

Si la tortue glisse le long du mur (petits triangles), augmente la tolerance d'orientation:

```bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py boundary_heading_tolerance:=0.5
```

Preset "mur rouge strict" (evite de traverser le trait rouge):

```bash
ros2 launch turtle_fsm_controller turtle_fsm.launch.py red_boost_duration:=0.4 red_event_cooldown:=0.5 red_reverse_duration:=1.3 red_turn_duration:=1.4 boundary_heading_tolerance:=0.5
```

## 9) Pour lancer le Assignement 2 (la tortue fait le contour de la fenêtre)
Dans le terminal 1 :
```bash
source /opt/ros/jazzy/setup.bash
cd /home/justine/Documents/M1_INFO/S2/Robotic/Robomaniak/ros2_ws
colcon build --packages-select turtle_boundaries_interfaces turtle_drawing_boundaries
source install/setup.bash
ros2 launch turtle_drawing_boundaries draw_boundaries.launch.py lost_wall_grace_s:=0.45
```

Dans le terminal 2 :
```
source /opt/ros/jazzy/setup.bash
cd /home/justine/Documents/M1_INFO/S2/Robotic/Robomaniak/ros2_ws
source install/setup.bash
ros2 action send_goal /draw_boundaries turtle_boundaries_interfaces/action/DrawBoundaries "{margin: 0.5, speed: 1.0, clockwise: true}" --feedback
```
