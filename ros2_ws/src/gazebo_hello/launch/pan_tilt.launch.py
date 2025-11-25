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

    # URDF/XACRO path for pan-tilt
    xacro_path = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf.xacro')
    urdf_path = os.path.join(pkg_share, 'urdf', 'moving_face.urdf')

    # Load pan-tilt URDF
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # -----------------------------
    # Robot State Publishers
    # -----------------------------
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }],
        output='screen'
    )

    # -----------------------------
    # Gazebo + world
    # -----------------------------
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

    # -----------------------------
    # NOTE:
    # pan_tilt is ALREADY in env.world, so we DO NOT spawn it again.
    # This avoids "Entity [pan_tilt] already exists" errors.
    # -----------------------------

    # -----------------------------
    # Spawn moving_face model from SDF
    # -----------------------------


    spawn_moving_face = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-entity', 'moving_face',
            '-x', '3.0',
            '-y', '1.5',
            '-z', '0.2'
        ],
        output='screen'
    )

    # -----------------------------
    # Other nodes
    # -----------------------------
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

    viewer = Node(
        package='gazebo_hello',
        executable='image_viewer',
        name='image_viewer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_image = Node(
        package='gazebo_hello',
        executable='target_teleop',
        name='target_teleop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_moving_face,
        tracker,
        detector,
        motion,
        viewer,
        #move_image,
    ])
