from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_margin = LaunchConfiguration('default_margin')
    default_speed = LaunchConfiguration('default_speed')
    trace_timeout_s = LaunchConfiguration('trace_timeout_s')
    lost_wall_grace_s = LaunchConfiguration('lost_wall_grace_s')
    manual_cmd_timeout_s = LaunchConfiguration('manual_cmd_timeout_s')
    resume_reacquire_timeout_s = LaunchConfiguration('resume_reacquire_timeout_s')
    resume_reacquire_radius = LaunchConfiguration('resume_reacquire_radius')
    start_keyboard = LaunchConfiguration('start_keyboard')
    keyboard_in_new_terminal = LaunchConfiguration('keyboard_in_new_terminal')
    keyboard_linear_speed = LaunchConfiguration('keyboard_linear_speed')
    keyboard_angular_speed = LaunchConfiguration('keyboard_angular_speed')

    start_keyboard_in_terminal = IfCondition(
        PythonExpression([
            "'",
            start_keyboard,
            "' == 'true' and '",
            keyboard_in_new_terminal,
            "' == 'true'",
        ])
    )
    start_keyboard_embedded = IfCondition(
        PythonExpression([
            "'",
            start_keyboard,
            "' == 'true' and '",
            keyboard_in_new_terminal,
            "' == 'false'",
        ])
    )

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
        DeclareLaunchArgument(
            'lost_wall_grace_s',
            default_value='0.45',
            description='Duree de grace avant de passer en recherche agressive du mur',
        ),
        DeclareLaunchArgument(
            'manual_cmd_timeout_s',
            default_value='0.25',
            description='Temps max de validite d une commande clavier manuelle',
        ),
        DeclareLaunchArgument(
            'resume_reacquire_timeout_s',
            default_value='8.0',
            description='Duree max de reacquisition apres reprise manuelle',
        ),
        DeclareLaunchArgument(
            'resume_reacquire_radius',
            default_value='0.22',
            description='Rayon de retour vers le point de decrochage manuel',
        ),
        DeclareLaunchArgument(
            'start_keyboard',
            default_value='true',
            description='Demarre le noeud clavier',
        ),
        DeclareLaunchArgument(
            'keyboard_in_new_terminal',
            default_value='true',
            description='Demarre le noeud clavier dans un gnome-terminal separe',
        ),
        DeclareLaunchArgument(
            'keyboard_linear_speed',
            default_value='1.2',
            description='Vitesse lineaire en mode manuel clavier',
        ),
        DeclareLaunchArgument(
            'keyboard_angular_speed',
            default_value='2.2',
            description='Vitesse angulaire en mode manuel clavier',
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
                'lost_wall_grace_s': lost_wall_grace_s,
                'manual_cmd_timeout_s': manual_cmd_timeout_s,
                'resume_reacquire_timeout_s': resume_reacquire_timeout_s,
                'resume_reacquire_radius': resume_reacquire_radius,
            }],
        ),
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--',
                'bash',
                '-lc',
                'ros2 run turtle_drawing_boundaries turtle_boundaries_keyboard; exec bash',
            ],
            output='screen',
            condition=start_keyboard_in_terminal,
        ),
        Node(
            package='turtle_drawing_boundaries',
            executable='turtle_boundaries_keyboard',
            name='turtle_boundaries_keyboard',
            output='screen',
            parameters=[{
                'manual_linear_speed': keyboard_linear_speed,
                'manual_angular_speed': keyboard_angular_speed,
            }],
            condition=start_keyboard_embedded,
        ),
    ])
