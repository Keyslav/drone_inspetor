import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_drone_inspetor = get_package_share_directory("drone_inspetor")
    pkg_ros_gz_bridge = get_package_share_directory("ros_gz_bridge")

    # Gazebo launch
    """gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {os.path.join(pkg_drone_inspetor, 'models', 'oceano.sdf')}"
        }.items(),
    )"""

    # PX4 SITL
    """px4_sitl = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"cd {os.path.join(os.environ["HOME"], "PX4-Autopilot")} && PX4_SYS_AUTOSTART=4030 PX4_GZ_MODEL_PATH={os.path.join(pkg_drone_inspetor, 'models')} ./build/px4_sitl_default/bin/px4 -d",
        ],
        output="screen",
        shell=True,
    )"""

    # Spawn drone
    """spawn_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_spawn.launch.py")
        ),
        launch_arguments={
            "file": os.path.join(pkg_drone_inspetor, "models", "model.sdf"),
            "name": "x500_uerj",
            "allow_renaming": "true",
            "x": "0",
            "y": "0",
            "z": "1",
        }.items(),
    )"""

    # MicroXRCE-DDS Agent
    """microrc_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        shell=True,
    )"""

    # ROS-Gazebo Bridge
    # ROS-Gazebo Bridge Node
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(pkg_drone_inspetor, "config", "ros_gz_bridges.yaml"),
        }],
    )
    """ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_bridge, "launch", "ros_gz_bridge.launch.py")
        ),
        launch_arguments={
            "config_file": os.path.join(pkg_drone_inspetor, "config", "ros_gz_bridges.yaml"),
        }.items(),
    )"""

    return LaunchDescription([
        # gazebo,
        # px4_sitl,
        # px4_simulation_gazebo,
        # spawn_drone,
        # microrc_agent,
        ros_gz_bridge_node,
    ])


