# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('gazebo_hello')
#     urdf_file = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf')
#     controller_config = os.path.join(pkg_share, 'config', 'pan_tilt_controllers.yaml')
    
#     with open(urdf_file, 'r') as f:
#         robot_description = f.read()
    
#     # Gazebo launch
#     gazebo_launch = os.path.join(
#         get_package_share_directory('gazebo_ros'),
#         'launch',
#         'gazebo.launch.py'
#     )
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(gazebo_launch),
#         launch_arguments={'verbose': 'true'}.items()
#     )
    
#     # Robot State Publisher
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': True
#         }],
#     )
    
#     # Spawn Entity
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'pan_tilt',
#             '-topic', 'robot_description',
#             '-x', '0.0',
#             '-y', '0.0',
#             '-z', '0.1',
#         ],
#         output='screen'
#     )
    
#     # # Load and start Joint State Broadcaster
#     # joint_state_broadcaster_spawner = Node(
#     #     package='controller_manager',
#     #     executable='spawner',
#     #     arguments=[
#     #         'joint_state_broadcaster',
#     #         '--controller-manager', '/controller_manager',
#     #         '--controller-manager-timeout', '30',
#     #     ],
#     #     output='screen',
#     #     parameters=[{'use_sim_time': True}]
#     # )
    
#     # # Load and start Pan Tilt Controller
#     # pan_tilt_controller_spawner = Node(
#     #     package='controller_manager',
#     #     executable='spawner',
#     #     arguments=[
#     #         'pan_tilt_controller',
#     #         '--controller-manager', '/controller_manager',
#     #         '--controller-manager-timeout', '30',
#     #     ],
#     #     output='screen',
#     #     parameters=[{'use_sim_time': True}]
#     # )
    
#     # # Motion control node (delayed)
#     # pan_tilt_motion_node = Node(
#     #     package='gazebo_hello',
#     #     executable='pan_tilt_motion',
#     #     name='pan_tilt_motion',
#     #     output='screen',
#     #     parameters=[{'use_sim_time': True}]
#     # )
    
#     # # Delay motion node until controllers are loaded
#     # delayed_motion_node = TimerAction(
#     #     period=5.0,
#     #     actions=[pan_tilt_motion_node]
#     # )
    
#     return LaunchDescription([
#         gazebo,
#         rsp,
#         spawn_entity,
#         # RegisterEventHandler(
#         #     event_handler=OnProcessExit(
#         #         target_action=spawn_entity,
#         #         on_exit=[
#         #             joint_state_broadcaster_spawner,
#         #             pan_tilt_controller_spawner,
#         #         ]
#         #     )
#         # ),
#         # delayed_motion_node,
#     ])