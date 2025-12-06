from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from drone_inspetor.signals.dashboard_signals import CameraSignals

from cv_bridge import CvBridge

class DashboardCameraSubscriber:
    """
    Gerencia a assinatura de tópicos de câmera e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: CameraSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals
        self.bridge = CvBridge() # Para conversão de imagens ROS para OpenCV/PyQt

        # QoS para dados de sensores (imagens): VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.camera_image_sub = self.DashboardNode.create_subscription(
            Image,
            "/drone_inspetor/interno/camera_node/image_raw",
            self.camera_image_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.camera_image_sub.topic_name}")

    def camera_image_callback(self, msg):
        """
        Callback para mensagens de imagem da câmera principal.
        Recebe a mensagem ROS Image e a converte para OpenCV antes de passar para a tela.
        Emite o sinal image_received da subclasse Camera.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.signals.image_received.emit(cv_image)


