import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'empty_world.world')

    # Gazebo Server & Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # SDF Paths
    leader_sdf = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_leader', 'model.sdf')
    follower_sdf = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_follower', 'model.sdf')

    # Spawn Nodes
    spawn_leader = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'turtlebot3_burger_leader', '-file', leader_sdf, '-x', '0.0', '-y', '0.0', '-z', '0.01']
    )

    spawn_follower = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'turtlebot3_burger_follower', '-file', follower_sdf, '-x', '-0.5', '-y', '0.0', '-z', '0.01']
    )

    # Bridge Parameters
    leader_bridge_params = os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_burger_leader_bridge.yaml')
    follower_bridge_params = os.path.join(pkg_turtlebot3_gazebo, 'params', 'turtlebot3_burger_follower_bridge.yaml')

    # Bridge Nodes
    leader_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['--ros-args', '-p', f'config_file:={leader_bridge_params}']
    )

    follower_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['--ros-args', '-p', f'config_file:={follower_bridge_params}']
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(spawn_leader)
    ld.add_action(spawn_follower)
    ld.add_action(leader_bridge)
    ld.add_action(follower_bridge)

    return ld
