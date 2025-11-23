import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('gazebo_hello')
    world_path = os.path.join(pkg_share, 'worlds', 'env.world')


    # Correct XACRO path
    xacro_path = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf.xacro')

    # Process XACRO so includes are expanded
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items()
    )
    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('gazebo_hello'), 'config', 'config_file.rviz')]
        )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'pan_tilt'],
        output='screen'
    )

    tracker = Node(
        package='gazebo_hello',
        executable='face_tracker',
        name='face_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    motion = Node(
        package='gazebo_hello',
        executable='pan_tilt_motion',
        name='pan_tilt_motion',
        output='screen',
        parameters=[{'use_sim_time': True}]
)

    detector = Node(
        package='gazebo_hello',
        executable='face_detector',
        name='face_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        tracker,
        detector,
        motion,
        rviz,
    ])
