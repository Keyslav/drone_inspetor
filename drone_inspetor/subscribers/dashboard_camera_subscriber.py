from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from drone_inspetor.signals.dashboard_signals import CameraSignals

from cv_bridge import CvBridge

class DashboardCameraSubscriber:
    """
    Gerencia a assinatura de tópicos de câmera e emite sinais PyQt.
    Assina tanto o tópico de imagem quanto o tópico de status de gravação.
    """
    def __init__(self, DashboardNode: Node, signals: CameraSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals
        self.bridge = CvBridge()

        # ==================== CONFIGURAÇÃO DE QoS ========================
        # QoS para dados de sensores (imagens)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS para status (RELIABLE para garantir entrega)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber para imagens da câmera
        self.camera_image_sub = self.DashboardNode.create_subscription(
            CompressedImage,
            "/drone_inspetor/interno/camera_node/compressed",
            self.camera_image_callback,
            qos_sensor_data
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.camera_image_sub.topic_name}")
        
        # Subscriber para status de gravação
        self.recording_status_sub = self.DashboardNode.create_subscription(
            Bool,
            "/drone_inspetor/interno/camera_node/recording",
            self.recording_status_callback,
            qos_status
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.recording_status_sub.topic_name}")

    def camera_image_callback(self, msg):
        """
        Callback para mensagens de imagem da câmera principal.
        Converte a mensagem para OpenCV e emite o sinal.
        """
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.signals.image_received.emit(cv_image)
    
    def recording_status_callback(self, msg: Bool):
        """
        Callback para status de gravação.
        Emite o sinal recording_status_received com o status atual.
        """
        self.signals.recording_status_received.emit(msg.data)
