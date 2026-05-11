"""
Specs dos serviços ROS2 (request/response) do drone_inspetor.

Todos os serviços expostos são do cv_node.
"""

from drone_inspetor_msgs.srv import (
    CVDetectionSRV,
    CVModelsSRV,
    EnableAnomalyDetectionSRV,
    RecordDetectionsSRV,
)

from drone_inspetor.ros_interfaces.specs import ServiceSpec


class Services:
    """Serviços ROS2. Acessar via Topics.Service.<NOME>."""

    CV_DETECTION = ServiceSpec(
        "/drone_inspetor/interno/cv_node/srv/detection",
        CVDetectionSRV,
    )
    CV_RECORD_DETECTIONS = ServiceSpec(
        "/drone_inspetor/interno/cv_node/srv/record_detections",
        RecordDetectionsSRV,
    )
    CV_ENABLE_ANOMALY = ServiceSpec(
        "/drone_inspetor/interno/cv_node/srv/enable_anomaly_detection",
        EnableAnomalyDetectionSRV,
    )
    CV_LIST_MODELS = ServiceSpec(
        "/drone_inspetor/interno/cv_node/srv/list_models",
        CVModelsSRV,
    )
