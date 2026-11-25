import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_name = LaunchConfiguration('world').perform(context)
    num_followers = int(LaunchConfiguration('num_followers').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))
    launch_controllers = (
        LaunchConfiguration('launch_controllers').perform(context).lower() == 'true'
    )

    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', world_name)

    actions = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_turtlebot3_gazebo, 'models')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v4 ', world],
                'on_exit_shutdown': 'true',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v4 '}.items()
        ),
    ]

    # Leader spawn & bridge
    leader_sdf = os.path.join(
        pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_leader', 'model.sdf'
    )
    leader_bridge_params = os.path.join(
        pkg_turtlebot3_gazebo, 'params', 'turtlebot3_burger_leader_bridge.yaml'
    )

    actions.append(
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'turtlebot3_burger_leader',
                '-file', leader_sdf,
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
            ],
        )
    )
    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='leader_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={leader_bridge_params}'],
        )
    )

    # Template files for followers
    follower_sdf_template_path = os.path.join(
        pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_follower', 'model.sdf'
    )
    follower_bridge_template_path = os.path.join(
        pkg_turtlebot3_gazebo, 'params', 'turtlebot3_burger_follower_bridge.yaml'
    )

    with open(follower_sdf_template_path, 'r', encoding='utf-8') as f:
        follower_sdf_template = f.read()
    with open(follower_bridge_template_path, 'r', encoding='utf-8') as f:
        follower_bridge_template = f.read()

    tmp_dir = tempfile.gettempdir()

    for k in range(1, num_followers + 1):
        ns = f'follower_{k}'
        spawn_x = -spacing * k

        sdf_path = os.path.join(tmp_dir, f'tb3_platoon_{ns}.sdf')
        with open(sdf_path, 'w', encoding='utf-8') as f:
            f.write(follower_sdf_template.replace('follower/', f'{ns}/'))

        bridge_path = os.path.join(tmp_dir, f'tb3_platoon_{ns}_bridge.yaml')
        with open(bridge_path, 'w', encoding='utf-8') as f:
            f.write(follower_bridge_template.replace('follower/', f'{ns}/'))

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', f'turtlebot3_burger_{ns}',
                    '-file', sdf_path,
                    '-x', f'{spawn_x:.3f}', '-y', '0.0', '-z', '0.01',
                ],
            )
        )
        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{ns}_bridge',
                output='screen',
                arguments=['--ros-args', '-p', f'config_file:={bridge_path}'],
            )
        )

        if launch_controllers:
            actions.append(
                Node(
                    package='turtlebot3_gazebo',
                    executable='twin_lidar_target.py',
                    name=f'{ns}_lidar_target',
                    output='screen',
                    parameters=[{
                        'scan_topic': f'/{ns}/scan',
                        'odom_topic': f'/{ns}/odom',
                        'target_topic': f'/{ns}/target',
                        'sector_deg': 45.0,
                        'track_mode': 'center',
                    }],
                )
            )
            actions.append(
                Node(
                    package='turtlebot3_gazebo',
                    executable='twin_follower_controller.py',
                    name=f'{ns}_controller',
                    output='screen',
                    parameters=[{
                        'robot_name': ns,
                        'target_topic': f'/{ns}/target',
                        'odom_topic': f'/{ns}/odom',
                        'cmd_topic': f'/{ns}/cmd_vel',
                        'target_distance': 0.30,
                        'kp_linear': 4.0,
                        'kd_linear': 0.4,
                        'use_breadcrumbs': True,
                        'lookahead_distance': 0.16,
                        'waypoint_spacing': 0.05,
                    }],
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_course.world',
            description='World file name inside turtlebot3_gazebo/worlds/',
        ),
        DeclareLaunchArgument(
            'num_followers',
            default_value='2',
            description='Number of following TurtleBots to spawn',
        ),
        DeclareLaunchArgument(
            'spacing',
            default_value='0.38',
            description='Initial center-to-center spacing (m) between robots',
        ),
        DeclareLaunchArgument(
            'launch_controllers',
            default_value='true',
            description='Auto-launch LiDAR target and Pure Pursuit controller for each follower',
        ),
        OpaqueFunction(function=launch_setup),
    ])
