from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare('prevent'), 'worlds', 'minimap.world']),
        description='Path to world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    spawn_tb3 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'tb3',
             '-file', PathJoinSubstitution([
                 FindPackageShare('turtlebot3_gazebo'),
                 'models', 'turtlebot3_waffle_pi', 'model.sdf'
             ]),
             '-x', '0', '-y', '0', '-z', '0.03'],
        output='screen'
    )

    vision = Node(
        package='prevent',
        executable='vision_detector.py',
        name='vision_detector',
        output='screen'    )

    controller = Node(
        package='prevent',
        executable='bot_controller.py',
        name='bot_controller',
        output='screen',
        parameters=[{
            'forward_speed': 0.5,
            'control_rate': 10.0
        }]
    )

    return LaunchDescription([world_arg, gazebo, spawn_tb3, vision, controller])
