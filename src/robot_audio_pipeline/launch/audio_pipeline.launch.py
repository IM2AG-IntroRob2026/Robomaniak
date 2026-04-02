from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory('robot_audio_pipeline'),
        'config',
        'audio_pipeline.yaml',
    )

    config = LaunchConfiguration('config')
    resource_path = LaunchConfiguration('resource_path')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Fichier YAML des parametres audio pipeline',
        ),
        DeclareLaunchArgument(
            'resource_path',
            default_value='',
            description='Chemin resource Snowboy (prioritaire sur le YAML)',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/audio/cmd_vel',
            description='Topic de commande vitesse pour voice_command_node',
        ),
        Node(
            package='robot_audio_pipeline',
            executable='audio_pipeline_node',
            name='audio_pipeline_node',
            output='screen',
            parameters=[
                config,
                {'resource_path': resource_path},
            ],
        ),
        Node(
            package='robot_audio_pipeline',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen',
            parameters=[config, {'cmd_vel_topic': cmd_vel_topic}],
        ),
    ])
