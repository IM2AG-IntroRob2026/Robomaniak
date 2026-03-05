Brancher dock

Poser robot dock

Attendre que le robot s'allume

Activer le wifi du robot et se connecter dessus

Aller sur http://192.168.10.1/ et connecter le robot sur le hotspot

Connecter le pc sur le même wifi que le robot

Pour modifier les paramètres du robot, trouver son adresse IP et se rendre sur http://<IP>/

---

```bash
docker run -it --net=host ros-iron-cyclon:latest bash
```

```bash
ros2 action list
```

```bash
ros2 action send_goal /RobotX/ACTION irobot_create_msgs/action/ACTION "{}"
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/Robot3/cmd_vel
```

To play Linkin Park - In the End:

https://musescore.com/user/189765/scores/5082403

http://www.ziggysono.com/pop/convert7.php

```bash
```