import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    pkg_drone_inspetor = get_package_share_directory("drone_inspetor")
    
    # Caminho do arquivo de parâmetros ROS2
    params_file = os.path.join(pkg_drone_inspetor, "config", "param_ros.yaml")
    
    param_use_sim_time = True

    # ROS-Gazebo Bridge Node
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(pkg_drone_inspetor, "config", "ros_gz_bridges.yaml"),
            "use_sim_time": param_use_sim_time,
        }],
    )

    # ROS-Gazebo Image Bridge Node
    ros_gz_image_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        output="screen",
        parameters=[{'use_sim_time': param_use_sim_time, 'qos': 'sensor_data'}],
        arguments=[
            "/drone_inspetor/gz/gimbal/camera",
            "/drone_inspetor/gz/depth_camera",
        ],
        remappings=[
            ("/drone_inspetor/gz/gimbal/camera/compressed",      "/drone_inspetor/externo/camera/compressed"           ),
            ("/drone_inspetor/gz/gimbal/camera/compressedDepth", "/drone_inspetor/externo/camera/compressedDepth"      ),
            ("/drone_inspetor/gz/gimbal/camera/theora",          "/drone_inspetor/externo/camera/theora"               ),
            ("/drone_inspetor/gz/gimbal/camera/zstd",            "/drone_inspetor/externo/camera/zstd"                 ),
            ("/drone_inspetor/gz/depth_camera/compressed",       "/drone_inspetor/externo/depth_camera/compressed"     ),
            ("/drone_inspetor/gz/depth_camera/compressedDepth",  "/drone_inspetor/externo/depth_camera/compressedDepth"),
            ("/drone_inspetor/gz/depth_camera/theora",           "/drone_inspetor/externo/depth_camera/theora"         ),
            ("/drone_inspetor/gz/depth_camera/zstd",             "/drone_inspetor/externo/depth_camera/zstd"           ),
        ]
    )

    # Nó da Câmera
    camera_node = Node(
        package="drone_inspetor",
        executable="camera_node",
        name="camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó de Visão Computacional (CV)
    cv_node = Node(
        package="drone_inspetor",
        executable="cv_node",
        name="cv_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó da Câmera de Profundidade
    depth_node = Node(
        package="drone_inspetor",
        executable="depth_node",
        name="depth_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó do LiDAR
    lidar_node = Node(
        package="drone_inspetor",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó de Controle do Drone
    drone_node = Node(
        package="drone_inspetor",
        executable="drone_node",
        name="drone_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó da Máquina de Estados Finita (FSM)
    fsm_node = Node(
        package="drone_inspetor",
        executable="fsm_node",
        name="fsm_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    # Nó do Dashboard (lógica ROS2 e inicialização da GUI)
    # on_exit: Quando a GUI é fechada, todos os outros nodes são encerrados em cascata
    dashboard_node = Node(
        package="drone_inspetor",
        executable="dashboard_node",
        name="dashboard_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {'use_sim_time': param_use_sim_time}],
        on_exit=Shutdown(reason="Dashboard GUI fechado - encerrando todos os nodes"),
        #arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    return LaunchDescription([
        ros_gz_bridge_node,
        ros_gz_image_node,
        camera_node,
        cv_node,
        depth_node,
        lidar_node,
        drone_node,
        fsm_node,
        dashboard_node,
    ])
