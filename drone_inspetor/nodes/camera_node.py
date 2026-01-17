"""
camera_node.py
=================================================================================================
Nó ROS2 responsável por gerenciar a comunicação com a câmera principal.

Este nó atua como intermediário entre o tópico externo da câmera (proveniente do simulador
ou driver de hardware) e o tópico interno padronizado do sistema. Sua função principal
é republicar as imagens recebidas em um formato padronizado para consumo pelo dashboard
e outros componentes do sistema.

ARQUITETURA:
- Assina tópico externo: /drone_inspetor/externo/camera/image_raw
- Publica tópico interno: /drone_inspetor/interno/camera_node/image_raw
- Utiliza cv_bridge para conversão de formatos de imagem quando necessário
=================================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class CameraNode(Node):
    """
    Nó ROS2 para gerenciamento da câmera principal.
    
    Este nó implementa um padrão de republishing, onde recebe imagens de uma fonte externa
    e as republica em um tópico interno padronizado. Isso permite desacoplamento entre
    a fonte de dados (simulador/hardware) e os consumidores internos do sistema.
    
    Responsabilidades:
    - Assinar tópicos da câmera raw externa (simulador ou driver de hardware)
    - Republicar imagens em tópico interno padronizado para o dashboard
    - Manter compatibilidade entre diferentes fontes de dados de câmera
    """
    
    def __init__(self):
        """
        Inicializa o nó da câmera principal.
        
        Configura subscribers e publishers necessários para o republishing de imagens.
        """
        super().__init__("camera_node")
        self.get_logger().info("Nó CameraNode iniciado.")
        
        # ==================== INICIALIZAÇÃO ============================================================
        # Cria instância do CvBridge para conversão entre formatos ROS e OpenCV quando necessário
        self.bridge = CvBridge()
        
        # ==================== CONFIGURAÇÃO DE QoS ========================
        # QoS para dados de sensores (imagens) - equivalente ao "sensor_data":
        # - BEST_EFFORT: menor latência, evita retransmissões; adequado para vídeo/imagem
        # - VOLATILE: não mantém amostras antigas
        # - KEEP_LAST: mantém somente as últimas N amostras
        # - DEPTH=1: evita fila e reduz lag no dashboard/YOLO (processa sempre o frame mais recente)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ==================== SUBSCRIBERS EXTERNOS (ENTRADA DE DADOS DO DRONE) =========================
        # Assina o tópico da câmera raw externa
        # Este tópico pode vir de diferentes fontes: simulador Gazebo, driver de câmera física, etc.
        self.camera_raw_sub = self.create_subscription(
            CompressedImage,
            "/drone_inspetor/externo/camera/compressed",
            self.camera_raw_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado tópico externo: {self.camera_raw_sub.topic_name}")

        # ==================== PUBLISHERS INTERNOS (SAÍDA DE DADOS PARA O DASHBOARD) ====================
        # Publica a imagem raw no tópico padronizado interno do sistema
        # Este tópico é consumido pelo dashboard e pelo nó de visão computacional
        self.dashboard_camera_image_pub = self.create_publisher(
            CompressedImage,
            "/drone_inspetor/interno/camera_node/compressed",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.dashboard_camera_image_pub.topic_name}")

    # ==================== CALLBACKS EXTERNOS (PROCESSAMENTO DE DADOS DO DRONE) ====================

    def camera_raw_callback(self, msg):
        """
        Callback para mensagens de imagem da câmera principal.
        
        Este método é chamado sempre que uma nova imagem é recebida do tópico externo.
        A imagem é simplesmente republicada no tópico interno padronizado, mantendo
        todos os metadados originais (header, encoding, etc.).
        
        Args:
            msg (sensor_msgs.msg.Image): Mensagem de imagem recebida do tópico externo.
        """
        self.get_logger().debug("Imagem da câmera principal recebida e re-publicada para o dashboard.")
        # Republica a mensagem diretamente no tópico interno
        # Não há necessidade de processamento, apenas republishing
        self.dashboard_camera_image_pub.publish(msg)

def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
