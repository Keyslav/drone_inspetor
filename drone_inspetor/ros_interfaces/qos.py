"""
Esses perfis de QoS isolam as nuances da comunicação ROS2 
e definem o comportamento esperado para cada cenário de envio/recebimento de dados.
Estes perfis são consumidos pelas TopicSpecs em ros_interfaces/*.py — não devem ser
usados diretamente em call-sites de create_subscription/create_publisher.
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class QoSProfiles:
    """Perfis QoS padronizados para comunicação ROS2 no drone_inspetor."""

    @staticmethod
    def sensor_data(depth: int = 10) -> QoSProfile:
        """BEST_EFFORT + VOLATILE. Para dados contínuos de sensores (câmera, LiDAR, depth, CV)."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

    @staticmethod
    def status() -> QoSProfile:
        """BEST_EFFORT + TRANSIENT_LOCAL, depth=1. Para estados/status (drone_state, mission_state)."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    @staticmethod
    def commands() -> QoSProfile:
        """RELIABLE + TRANSIENT_LOCAL, depth=5. Para comandos críticos (mission commands)."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    @staticmethod
    def commands_volatile(depth: int = 10) -> QoSProfile:
        """RELIABLE + VOLATILE. Para comandos do dashboard (câmera, CV, depth, LiDAR, mapa, drone)."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

    @staticmethod
    def px4() -> QoSProfile:
        """BEST_EFFORT + TRANSIENT_LOCAL, depth=1. Para telemetria PX4 (/fmu/out/*, /fmu/in/*)."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
