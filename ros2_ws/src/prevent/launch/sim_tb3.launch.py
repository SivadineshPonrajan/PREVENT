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
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # controller = Node(
    #     package='prevent',
    #     executable='bot_controller.py',
    #     name='bot_controller',
    #     output='screen',
    #     parameters=[{
    #     'use_sim_time': True,
    #     'max_speed': 0.35,
    #     'slow_speed': 0.22,
    #     'turn_speed': 0.25,
    #     'turn_linear': 0.0,
    #     'search_angle': 1.2,
    #     'turn_angle': 1.57,
    #     'safe_distance': 0.8,
    #     'door_duration': 12.0,
    #     'control_rate': 10.0,
	#     'cue_cooldown': 8.0,
    # }]
    # )

    controller = Node(
        package='prevent',
        executable='bot_controller.py',
        name='bot_controller',
        output='screen',
        parameters=[{
            'max_speed': 0.35,
            'slow_speed': 0.22,
            'turn_speed': 0.20,
            'turn_linear': 0.0,
            'turn_angle': 1.5708,
            'angle_tol': 0.08,
            'rotate_timeout': 4.0,
            'safe_distance': 0.8,
            'peek_sector_width': 1.0,
            'side_sector_width': 0.5,
            'junction_forward_thresh': 0.6,
            'junction_side_thresh': 4.0,
            'cue_cooldown': 7.0,
            'forward_clear_stop_m': 2.0,
            'forward_clear_hits': 2,
            'door_duration': 12.0,
            'control_rate': 10.0,
            'use_sim_time': True,  # optional if you want sim time
        }]
    )

    return LaunchDescription([world_arg, gazebo, spawn_tb3, vision, controller])
