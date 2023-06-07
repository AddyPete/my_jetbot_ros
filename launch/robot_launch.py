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
        name = 'ekf_estimator',
        output='screen',
        parameters=[os.path.join(package_dir, 'config', 'ekf_config.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(package_dir, 'config', 'rviz_config.rviz')]
    )

    return LaunchDescription([
        webots,
        base_link_to_laser,
        rviz,
        # base_link_to_imu,
        my_robot_driver,
        # ekf_estimator,
        # odom_estimator,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])