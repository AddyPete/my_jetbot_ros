import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_name = 'my_jetbot_ros'
    package_dir = get_package_share_directory(package_name)
    use_sim_time = False
    
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_jetbot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'jetbot_world.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'jetbot'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )
    odom_estimator = Node(
        package=package_name,
        executable='odom_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0.0', '0.0', '0', '0', '0', 'base_link', 'laser'],
        )
    base_link_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0.0', '0.0', '0', '0', '0', 'base_link', 'imu'],
        )
    ekf_estimator = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[{
            'frequency': 10.0,
            'two_d_mode': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            # Use the imu node and only read the yaw, and the yaw velocity
            'imu0': '/imu',
            'odom0_config': [True,  True,  True,
                        False, False, False,
                        False, False, False,
                        False, False, True,
                        False, False, False],
            'imu0_queue_size': 40,
            # Use the estimated velocity node
            # listen for x, y, yaw positions, velocity in x and yaw directions
            'odom0': '/odom0',
            'imu0_config': [False, False, False,
                        True,  True,  True,
                        False, False, False,
                        False, False, False,
                        False, False, False],
            'odom0_queue_size': 40,
            'imu0_differential': False,
            'publish_tf': True,
            # we will not listen to the velocity message so the control_config
            # is useless
            'use_control': False,
            'control_config': [True, False, False,
                               False, False, True],
            'debug': False,
        }],
        remappings=[('/odometry/filtered', '/odom')]
    )

    return LaunchDescription([
        webots,
        base_link_to_laser,
        base_link_to_imu,
        my_robot_driver,
        ekf_estimator,
        odom_estimator,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])