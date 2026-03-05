from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_margin = LaunchConfiguration('default_margin')
    default_speed = LaunchConfiguration('default_speed')
    trace_timeout_s = LaunchConfiguration('trace_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'default_margin',
            default_value='0.5',
            description='Marge par defaut appliquee aux bords de turtlesim',
        ),
        DeclareLaunchArgument(
            'default_speed',
            default_value='1.0',
            description='Vitesse lineaire par defaut de trace',
        ),
        DeclareLaunchArgument(
            'trace_timeout_s',
            default_value='240.0',
            description='Timeout maximal de suivi du contour',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='turtle_drawing_boundaries',
            executable='turtle_boundaries_node',
            name='turtle_boundaries',
            output='screen',
            parameters=[{
                'default_margin': default_margin,
                'default_speed': default_speed,
                'domain_min': 0.0,
                'domain_max': 11.0,
                'trace_timeout_s': trace_timeout_s,
            }],
        ),
    ])
