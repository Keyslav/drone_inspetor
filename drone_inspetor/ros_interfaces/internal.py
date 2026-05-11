"""
Specs dos tópicos internos entre nós (/drone_inspetor/interno/*).

Estes tópicos têm produtor e consumidor dentro do próprio sistema, então o
contrato (tipo + QoS) é definido aqui de forma única e inalterável.
"""

from px4_msgs.msg import BatteryStatus
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from drone_inspetor_msgs.msg import (
    CVDetectionMSG,
    DroneStateMSG,
    LidarMSG,
    MissionStateMSG,
    ObstaclesMSG,
)

from drone_inspetor.ros_interfaces.qos import QoSProfiles
from drone_inspetor.ros_interfaces.specs import TopicSpec


class InternalTopics:
    """Tópicos internos entre nós. Acessar via Topics.Interno.<NOME>."""

    # --- drone_node ---
    DRONE_STATE = TopicSpec(
        "/drone_inspetor/interno/drone_node/drone_state",
        DroneStateMSG,
        QoSProfiles.status(),
    )
    DRONE_BATTERY_STATUS = TopicSpec(
        "/drone_inspetor/interno/drone_node/battery_status",
        BatteryStatus,
        QoSProfiles.status(),
    )

    # --- mission_node ---
    MISSION_STATE = TopicSpec(
        "/drone_inspetor/interno/mission_node/mission_state",
        MissionStateMSG,
        QoSProfiles.status(),
    )

    # --- camera_node ---
    CAMERA_COMPRESSED = TopicSpec(
        "/drone_inspetor/interno/camera_node/compressed",
        CompressedImage,
        QoSProfiles.sensor_data(depth=1),
    )
    CAMERA_RECORDING = TopicSpec(
        "/drone_inspetor/interno/camera_node/recording",
        Bool,
        QoSProfiles.commands_volatile(depth=1),
    )

    # --- cv_node ---
    CV_COMPRESSED = TopicSpec(
        "/drone_inspetor/interno/cv_node/compressed",
        CompressedImage,
        QoSProfiles.sensor_data(),
    )
    # Padronizado: pub e sub usam a mesma profundidade (antes pub=1, sub=10)
    CV_OBJECT_DETECTIONS = TopicSpec(
        "/drone_inspetor/interno/cv_node/object_detections",
        CVDetectionMSG,
        QoSProfiles.sensor_data(depth=10),
    )
    CV_ANALYSIS_REPORT = TopicSpec(
        "/drone_inspetor/interno/cv_node/analysis_report",
        String,
        QoSProfiles.sensor_data(depth=1),
    )

    # --- depth_node ---
    DEPTH_IMAGE_PROCESSED = TopicSpec(
        "/drone_inspetor/interno/depth_node/image_processed",
        Image,
        QoSProfiles.sensor_data(),
    )
    DEPTH_STATISTICS = TopicSpec(
        "/drone_inspetor/interno/depth_node/statistics",
        String,
        QoSProfiles.sensor_data(),
    )
    DEPTH_PROXIMITY_ALERTS = TopicSpec(
        "/drone_inspetor/interno/depth_node/proximity_alerts",
        String,
        QoSProfiles.sensor_data(),
    )
    # Detecções de obstáculos da câmera depth (cobertura frontal)
    DEPTH_OBSTACLE_DETECTIONS = TopicSpec(
        "/drone_inspetor/interno/depth_node/obstacle_detections",
        ObstaclesMSG,
        QoSProfiles.commands_volatile(depth=10),
    )

    # --- lidar_node ---
    LIDAR_DATA = TopicSpec(
        "/drone_inspetor/lidar_node/lidar_data",
        LidarMSG,
        QoSProfiles.sensor_data(),
    )
    LIDAR_STATISTICS = TopicSpec(
        "/drone_inspetor/lidar_node/statistics",
        String,
        QoSProfiles.sensor_data(),
    )
    # Detecções de obstáculos do LiDAR (cobertura 360° + abaixo)
    LIDAR_OBSTACLE_DETECTIONS = TopicSpec(
        "/drone_inspetor/interno/lidar_node/obstacle_detections",
        ObstaclesMSG,
        QoSProfiles.commands_volatile(depth=10),
    )
