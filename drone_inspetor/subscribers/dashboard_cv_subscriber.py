from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from drone_inspetor_msgs.msg import CVDetectionMSG
from drone_inspetor_msgs.srv import CVModelsSRV
import json
from drone_inspetor.signals.dashboard_signals import CVSignals
from drone_inspetor.ros_interfaces import Topics, create_client_from, create_subscription_from
from cv_bridge import CvBridge
from drone_inspetor.gui.presentation.detections import DetectionFrame

class DashboardCVSubscriber:
    """
    Gerencia a assinatura de tópicos de Visão Computacional e emite sinais PyQt.
    """
    def __init__(self, DashboardNode: Node, signals: CVSignals):
        self.DashboardNode = DashboardNode
        self.signals = signals
        self.bridge = CvBridge()
        self._retry_timer = None
        self._models_future = None

        # Subscriber para imagem processada por CV (comprimida)
        self.cv_image_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.CV_COMPRESSED, self.cv_image_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_image_sub.topic_name}")

        # Subscriber para detecções de objetos
        self.cv_detections_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.CV_OBJECT_DETECTIONS, self.cv_detections_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_detections_sub.topic_name}")

        # Subscriber para dados de análise
        self.cv_analysis_sub = create_subscription_from(
            self.DashboardNode, Topics.Interno.CV_ANALYSIS_REPORT, self.cv_analysis_callback,
        )
        self.DashboardNode.get_logger().info(f"Inscrito no tópico: {self.cv_analysis_sub.topic_name}")

        # Service Client para listar modelos
        self.cv_models_client = create_client_from(self.DashboardNode, Topics.Service.CV_LIST_MODELS)

        # Conecta sinal de requisição
        self.signals.models_requested.connect(self.request_models)

    def cv_image_callback(self, msg: CompressedImage):
        """
        Callback para mensagens de imagem processada por Visão Computacional.
        Converte CompressedImage para OpenCV e emite o sinal image_received.
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.signals.image_received.emit(cv_image)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao converter imagem CV comprimida: {e}")

    def cv_detections_callback(self, msg: CVDetectionMSG):
        """Entrega dados copiados e imutáveis à thread Qt."""
        self.signals.detections_received.emit(DetectionFrame.from_message(msg))

    def cv_analysis_callback(self, msg):
        """
        Callback para mensagens de dados de análise de Visão Computacional.
        Decodifica o JSON e emite o sinal analysis_data_received da subclasse CV.
        """
        try:
            analysis_data = json.loads(msg.data)
            self.signals.analysis_data_received.emit(analysis_data)
        except json.JSONDecodeError as e:
            self.DashboardNode.get_logger().error(f"Erro ao decodificar JSON de análise CV: {e}")

    def request_models(self):
        """
        Solicita a lista de modelos disponíveis e atuais ao cv_node.
        Se o serviço não estiver disponível, tenta novamente após 1 segundo.
        """
        if self._models_future is not None and not self._models_future.done():
            return
        if not self.cv_models_client.service_is_ready():
            self.DashboardNode.get_logger().warn("Serviço list_models não disponível, tentando novamente em 1s...")
            # Cria um timer para tentar novamente em 1 segundo (one-shot)
            if self._retry_timer is None:
                self._retry_timer = self.DashboardNode.create_timer(1.0, self._retry_request_models)
            return

        request = CVModelsSRV.Request()
        self._models_future = self.cv_models_client.call_async(request)
        self._models_future.add_done_callback(self._models_response_callback)
    
    def _retry_request_models(self):
        """
        Callback do timer para tentar solicitar modelos novamente.
        """
        # Cancela o timer para que ele não execute novamente
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.DashboardNode.destroy_timer(self._retry_timer)
            self._retry_timer = None
            
        # Tenta solicitar novamente
        self.request_models()

    def _models_response_callback(self, future):
        """
        Callback para resposta do serviço de modelos.
        """
        try:
            response = future.result()
            
            # Repassa o JSON completo e os modelos atuais para o sinal
            models_data = {
                'models_data_json': response.models_data_json,
                'current_object_model': response.current_object_model,
                'current_anomaly_model': response.current_anomaly_model
            }
            self.signals.models_received.emit(models_data)
        except Exception as e:
            self.DashboardNode.get_logger().error(f"Erro ao obter modelos do CV: {e}")


