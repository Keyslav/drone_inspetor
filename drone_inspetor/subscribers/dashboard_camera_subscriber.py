from rclpy.node import Node
from std_msgs.msg import Bool
from drone_inspetor.signals.dashboard_signals import CameraSignals
from drone_inspetor.ros_interfaces import Topics, create_subscription_from

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

        # Subscriber para imagens da câmera
        self.camera_image_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.CAMERA_COMPRESSED, self.camera_image_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.camera_image_sub.topic_name}")

        # Subscriber para status de gravação
        self.recording_status_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.CAMERA_RECORDING, self.recording_status_callback,
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
