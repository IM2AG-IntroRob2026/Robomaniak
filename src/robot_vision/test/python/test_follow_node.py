from __future__ import annotations

import time
import pytest
from launch_ros.actions import Node as LaunchNode

from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector

from ros2_easy_test import ROS2TestEnvironment, with_single_node


def _make_detection(cx: float, cy: float, w: float, h: float) -> Detection2D:
    det = Detection2D()
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = cx
    det.bbox.center.position.y = cy
    det.bbox.size_x = w
    det.bbox.size_y = h
    return det


FOLLOW_NODE = LaunchNode(
    package="robot_vision",
    executable="follow_node",
    name="follow_node",
    parameters=[{
        "strategy": "most_centered",
        "angular_dead_zone": 0.2,
        "linear_dead_zone": 50.0,
        "target_bbox_height": 1000.0,
        "lock_lost_frames": 10,
        "allow_reverse": False,
    }],
)


@with_single_node(FOLLOW_NODE, watch_topics={"/follow/cmd_vel": Twist})
def test_cible_centree_pas_de_commande_angulaire(env: ROS2TestEnvironment) -> None:
    msg = Detection2DArray()
    msg.detections.append(_make_detection(cx=320.0, cy=240.0, w=100.0, h=500.0))

    env.publish("/detection/detections", msg)
    twist = env.assert_message_published("/follow/cmd_vel", timeout=2.0)
    assert abs(twist.angular.z) < 0.1, f"angular.z devrait être ~0, reçu {twist.angular.z}"


@with_single_node(FOLLOW_NODE, watch_topics={"/follow/cmd_vel": Twist})
def test_cible_a_droite_rotation_negative(env: ROS2TestEnvironment) -> None:
    msg = Detection2DArray()
    msg.detections.append(_make_detection(cx=580.0, cy=240.0, w=100.0, h=500.0))

    env.publish("/detection/detections", msg)
    twist = env.assert_message_published("/follow/cmd_vel", timeout=2.0)
    assert twist.angular.z < -0.1, f"angular.z devrait être négatif, reçu {twist.angular.z}"


@with_single_node(FOLLOW_NODE, watch_topics={"/follow/cmd_vel": Twist})
def test_bumper_declenche_recule(env: ROS2TestEnvironment) -> None:
    hazard = HazardDetectionVector()
    env.publish("/hazard_detection", hazard)

    dets = Detection2DArray()
    dets.detections.append(_make_detection(320.0, 240.0, 100.0, 500.0))
    env.publish("/detection/detections", dets)

    twist = env.assert_message_published("/follow/cmd_vel", timeout=2.0)
    assert twist.linear.x <= 0.0