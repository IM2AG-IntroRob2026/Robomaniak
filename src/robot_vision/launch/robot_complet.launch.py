import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('robot_vision')
    models_path = PathJoinSubstitution([pkg_share, 'models'])

    return LaunchDescription([
        # 1. Flux Vidéo
        Node(
            package='robot_vision',
            executable='video_node',
            name='video_node',
            parameters=[{'stream_url': "http://10.202.48.94:8080/video", 'publish_rate_hz': 30}]
        ),

        # 2. Détection (YOLO)
        Node(
            package='robot_vision',
            executable='detection_node',
            parameters=[{
                'model_path': PathJoinSubstitution([models_path, 'yolov8s.onnx']),
                'confidence_threshold': 0.5,
                'use_gpu': False
            }]
        ),

        # 3. Suivi de cible
        Node(
            package='robot_vision',
            executable='follow_node',
            parameters=[{'strategy': 'most_centered', 'allow_reverse': True}],
            remappings=[
                ('/ir_intensity', '/Robot3/ir_intensity'),
                ('/hazard_detection', '/Robot3/hazard_detection')
            ]
        ),

        # 4. BT Manager
        Node(
            package='robot_vision',
            executable='bt_manager_node',
            parameters=[{'bt_tick_hz': 50.0, 'dock_action': '/Robot3/dock', 'undock_action': '/Robot3/undock'}],
            remappings=[
                ('/cmd_vel', '/Robot3/cmd_vel'),
                ('/cmd_lightring', '/Robot3/cmd_lightring'),
                ('/odom', '/Robot3/odom')
            ]
        ),

        # 5. Listen Node
        Node(
            package='robot_vision',
            executable='listen_node',
            parameters=[{
                'encoder_path': PathJoinSubstitution([models_path, 'kws_encoder.onnx']),
                'decoder_path': PathJoinSubstitution([models_path, 'kws_decoder.onnx']),
                'joiner_path': PathJoinSubstitution([models_path, 'kws_joiner.onnx']),
                'tokens_path': PathJoinSubstitution([models_path, 'kws_tokens.txt']),
            }],
            output='screen'
        ),

        # 6. Teleop
        # Node(
        #     package='robot_vision',
        #     executable='teleop_node',
        #     condition=None
        # ),

        # 7. Dock Detector
        Node(
            package='robot_vision',
            executable='dock_detector_node',
            name='dock_detector_node',
            output='screen',
            parameters=[{
                'camera_info_path': '/root/ros2_ws/src/robot_vision/config/camera_intrinsics.yaml',
                'marker_id':      0,
                'marker_size_m':  0.08,
                'dictionary':     'DICT_4X4_50',
                'publish_debug':  False,
            }]
        )
    ])