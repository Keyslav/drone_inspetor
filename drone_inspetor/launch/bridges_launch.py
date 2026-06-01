import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_drone_inspetor = get_package_share_directory("drone_inspetor")
    
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

    return LaunchDescription([
        ros_gz_bridge_node,
        ros_gz_image_node,
    ])
