import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_drone_inspetor = get_package_share_directory("drone_inspetor")
    
    # Caminho do arquivo de parâmetros ROS2
    params_file = os.path.join(pkg_drone_inspetor, "config", "param_ros.yaml")
    
    param_use_sim_time = True

    # Nó da Câmera
    camera_node = Node(
        package="drone_inspetor",
        executable="camera_node",
        name="camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    # Nó de Visão Computacional (CV)
    cv_node = Node(
        package="drone_inspetor",
        executable="cv_node",
        name="cv_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    # Nó da Câmera de Profundidade
    depth_node = Node(
        package="drone_inspetor",
        executable="depth_node",
        name="depth_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    # Nó do LiDAR
    lidar_node = Node(
        package="drone_inspetor",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    # Nó de Controle do Drone
    drone_node = Node(
        package="drone_inspetor",
        executable="drone_node",
        name="drone_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    # Nó da Máquina de Estados Finita (FSM)
    fsm_node = Node(
        package="drone_inspetor",
        executable="fsm_node",
        name="fsm_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
    )

    return LaunchDescription([
        camera_node,
        cv_node,
        depth_node,
        lidar_node,
        drone_node,
        fsm_node,
    ])
