"""
Specs dos tópicos externos vindos do simulador/drone real (/drone_inspetor/externo/*)
e dos tópicos brutos do Gazebo (/drone_inspetor/gz/*).
"""

from sensor_msgs.msg import CompressedImage, Image, LaserScan

from drone_inspetor.ros_interfaces.qos import QoSProfiles
from drone_inspetor.ros_interfaces.specs import TopicSpec


class ExternalTopics:
    """Tópicos externos. Acessar via Topics.Externo.<NOME>."""

    # --- Câmera ---
    CAMERA_COMPRESSED = TopicSpec(
        "/drone_inspetor/externo/camera/compressed",
        CompressedImage,
        QoSProfiles.sensor_data(depth=1),
    )
    CAMERA_IMAGE_RAW = TopicSpec(
        "/drone_inspetor/externo/camera/image_raw",
        Image,
        QoSProfiles.sensor_data(),
    )
    CAMERA_COMPRESSED_DEPTH = TopicSpec(
        "/drone_inspetor/externo/camera/compressedDepth",
        CompressedImage,
        QoSProfiles.sensor_data(),
    )
    CAMERA_THEORA = TopicSpec(
        "/drone_inspetor/externo/camera/theora",
        Image,
        QoSProfiles.sensor_data(),
    )
    CAMERA_ZSTD = TopicSpec(
        "/drone_inspetor/externo/camera/zstd",
        Image,
        QoSProfiles.sensor_data(),
    )

    # --- Depth Camera ---
    DEPTH_CAMERA_IMAGE_RAW = TopicSpec(
        "/drone_inspetor/externo/depth_camera/image_raw",
        Image,
        QoSProfiles.sensor_data(),
    )
    DEPTH_CAMERA_COMPRESSED = TopicSpec(
        "/drone_inspetor/externo/depth_camera/compressed",
        CompressedImage,
        QoSProfiles.sensor_data(),
    )
    DEPTH_CAMERA_COMPRESSED_DEPTH = TopicSpec(
        "/drone_inspetor/externo/depth_camera/compressedDepth",
        CompressedImage,
        QoSProfiles.sensor_data(),
    )
    DEPTH_CAMERA_THEORA = TopicSpec(
        "/drone_inspetor/externo/depth_camera/theora",
        Image,
        QoSProfiles.sensor_data(),
    )
    DEPTH_CAMERA_ZSTD = TopicSpec(
        "/drone_inspetor/externo/depth_camera/zstd",
        Image,
        QoSProfiles.sensor_data(),
    )

    # --- LiDAR ---
    LIDAR_SCAN = TopicSpec(
        "/drone_inspetor/externo/lidar/scan",
        LaserScan,
        QoSProfiles.sensor_data(),
    )
    LIDAR_DOWN_SCAN = TopicSpec(
        "/drone_inspetor/externo/lidar_down/scan",
        LaserScan,
        QoSProfiles.sensor_data(),
    )

    # --- Gazebo (pré-bridge) ---
    GZ_GIMBAL_CAMERA = TopicSpec(
        "/drone_inspetor/gz/gimbal/camera",
        Image,
        QoSProfiles.sensor_data(),
    )
    GZ_DEPTH_CAMERA = TopicSpec(
        "/drone_inspetor/gz/depth_camera",
        Image,
        QoSProfiles.sensor_data(),
    )
