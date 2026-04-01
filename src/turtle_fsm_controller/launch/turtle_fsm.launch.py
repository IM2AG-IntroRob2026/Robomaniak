from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    red_boost_duration = LaunchConfiguration('red_boost_duration')
    red_event_cooldown = LaunchConfiguration('red_event_cooldown')
    red_reverse_duration = LaunchConfiguration('red_reverse_duration')
    red_turn_duration = LaunchConfiguration('red_turn_duration')
    boundary_margin = LaunchConfiguration('boundary_margin')
    boundary_heading_tolerance = LaunchConfiguration('boundary_heading_tolerance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'red_boost_duration',
            default_value='0.8',
            description='Duree du boost apres detection du rouge',
        ),
        DeclareLaunchArgument(
            'red_event_cooldown',
            default_value='0.8',
            description='Delai minimal entre deux evenements rouge',
        ),
        DeclareLaunchArgument(
            'red_reverse_duration',
            default_value='1.0',
            description='Duree du recul apres detection rouge',
        ),
        DeclareLaunchArgument(
            'red_turn_duration',
            default_value='1.2',
            description='Duree de rotation apres recul rouge',
        ),
        DeclareLaunchArgument(
            'boundary_margin',
            default_value='0.6',
            description='Marge de securite vis-a-vis du bord',
        ),
        DeclareLaunchArgument(
            'boundary_heading_tolerance',
            default_value='0.35',
            description='Tolerance d orientation vers le centre en degagement',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='turtle_fsm_controller',
            executable='turtle_fsm_node',
            name='turtle_fsm',
            output='screen',
            parameters=[{
                'red_boost_duration': red_boost_duration,
                'red_event_cooldown': red_event_cooldown,
                'red_reverse_duration': red_reverse_duration,
                'red_turn_duration': red_turn_duration,
                'boundary_margin': boundary_margin,
                'boundary_heading_tolerance': boundary_heading_tolerance,
            }],
        ),
    ])
