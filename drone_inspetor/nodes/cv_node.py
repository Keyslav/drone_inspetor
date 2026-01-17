"""
cv_node.py
=================================================================================================
Nó ROS2 responsável pelo processamento de visão computacional.

Este nó implementa algoritmos de detecção de objetos usando YOLO (You Only Look Once),
uma rede neural convolucional para detecção de objetos em tempo real. O nó processa
imagens recebidas da câmera principal, detecta objetos de interesse (como flares, anomalias),
e publica tanto imagens anotadas quanto dados estruturados de detecção.

ARQUITETURA:
- Assina: /drone_inspetor/interno/camera_node/image_raw (imagens da câmera)
- Publica: /drone_inspetor/interno/cv_node/image_processed (imagens anotadas)
- Publica: /drone_inspetor/interno/cv_node/object_detections (dados de detecção em JSON)

ALGORITMO:
- Utiliza modelo YOLO customizado (best.pt) treinado para detecção de objetos específicos
- Aplica threshold de confiança mínimo de 0.5 para filtrar detecções
- Desenha bounding boxes e labels nas imagens processadas
=================================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
from datetime import datetime
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

# Importação das mensagens ROS customizadas
from drone_inspetor_msgs.msg import CVDetectionMSG, CVDetectionItemMSG

class CVNode(Node):
    """
    Nó ROS2 para processamento de visão computacional.
    
    Este nó utiliza YOLO para detectar objetos em imagens recebidas da câmera principal.
    As detecções são processadas e publicadas em dois formatos:
    1. Imagens anotadas com bounding boxes e labels
    2. Dados estruturados em JSON contendo informações sobre cada detecção
    
    Responsabilidades:
    - Assinar tópicos de imagem raw da câmera
    - Aplicar algoritmos de detecção de objetos usando YOLO
    - Publicar imagem processada com bounding boxes e labels
    - Publicar dados estruturados de detecção de objetos em formato JSON
    """
    
    def __init__(self):
        """
        Inicializa o nó de visão computacional.
        
        Carrega o modelo YOLO e configura subscribers/publishers necessários.
        """
        super().__init__("cv_node")
        self.get_logger().info("Nó CVNode iniciado.")
        
        # ==================== INICIALIZAÇÃO ============================================================
        # Cria instância do CvBridge para conversão entre formatos ROS e OpenCV
        self.bridge = CvBridge()
        
        # ==================== CARREGAMENTO DO MODELO YOLO ===============================================
        # Obtém o caminho do diretório de instalação do pacote
        # O modelo YOLO customizado deve estar localizado em: <package_share>/best.pt
        pkg_share_dir = get_package_share_directory('drone_inspetor')
        model_path = os.path.join(pkg_share_dir, 'best.pt')
        
        try:
            # Carrega o modelo YOLO customizado
            # Este modelo foi treinado especificamente para detectar objetos de interesse
            # (flares, anomalias, etc.) no contexto da inspeção de drones
            self.yolo_model = YOLO(model_path)
            self.get_logger().info(f"Modelo YOLO carregado com sucesso de: {model_path}")

            import torch
            self.get_logger().info(f"torch.cuda.is_available()={torch.cuda.is_available()}")
            if torch.cuda.is_available():
                self.get_logger().info(f"GPU: {torch.cuda.get_device_name(0)}")

        except Exception as e:
            # Se o modelo não puder ser carregado, o nó continua funcionando
            # mas não realizará detecções (retornará imagens sem anotações)
            self.get_logger().error(f"Erro ao carregar modelo YOLO: {e}")
            self.get_logger().error(f"Tentando carregar de: {model_path}")
            self.yolo_model = None
        
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
        
        # ==================== SUBSCRIBERS EXTERNOS (ENTRADA DE DADOS DO SIMULADOR/DRONE) ========================
        # Assina o tópico externo de imagens comprimidas da câmera do Gazebo/drone
        self.compressed_image_subscription = self.create_subscription(
            CompressedImage,
            "/drone_inspetor/externo/camera/compressed",
            self.image_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado tópico externo: {self.compressed_image_subscription.topic_name}")
        
        # ==================== PUBLISHERS INTERNOS (SAÍDA DE DADOS PARA O DASHBOARD) ====================
        # Publica imagens processadas com anotações (bounding boxes e labels)
        # Estas imagens são consumidas pelo dashboard para visualização
        self.processed_image_publisher = self.create_publisher(
            CompressedImage,
            "/drone_inspetor/interno/cv_node/compressed",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.processed_image_publisher.topic_name}")
        
        # Publica dados estruturados de detecção de objetos como mensagem ROS
        # Estes dados são consumidos pela FSM para tomar decisões baseadas em detecções
        self.detection_publisher = self.create_publisher(
            CVDetectionMSG,
            "/drone_inspetor/interno/cv_node/object_detections",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando detecções no tópico: {self.detection_publisher.topic_name}")

        self.get_logger().info("CVNode inicializado com sucesso.")

    # ==================== CALLBACKS INTERNOS (PROCESSAMENTO DE IMAGENS DA CÂMERA) ====================

    def image_callback(self, msg: CompressedImage):
        """
        Callback para processar imagens comprimidas recebidas da câmera.
        
        Este método é chamado sempre que uma nova imagem comprimida é recebida. Ele:
        1. Converte a mensagem CompressedImage ROS para formato OpenCV
        2. Aplica detecção de objetos usando YOLO
        3. Publica a imagem anotada como CompressedImage
        4. Publica dados estruturados de detecção como mensagem ROS
        
        Args:
            msg (sensor_msgs.msg.CompressedImage): Mensagem de imagem comprimida recebida.
        """
        try:
            # Converte mensagem CompressedImage ROS para imagem OpenCV no formato BGR8
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Aplica detecção de objetos usando YOLO
            # Retorna imagem anotada e lista de detecções
            annotated_image, detections = self.detect_objects(cv_image)
            
            # Publica imagem processada com anotações como CompressedImage
            try:
                # Converte imagem OpenCV para mensagem CompressedImage ROS
                processed_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format="jpeg")
                # Preserva o header original (timestamp, frame_id, etc.)
                processed_msg.header = msg.header
                self.processed_image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().error(f"Erro ao publicar imagem processada: {e}")
            
            # Publica dados de detecção como mensagem ROS
            # Apenas publica se houver detecções
            if detections:
                # Cria mensagem CVDetectionMSG
                detection_msg = CVDetectionMSG()
                detection_msg.timestamp = datetime.now().isoformat()
                detection_msg.count = len(detections)
                
                # Converte cada detecção para CVDetectionItemMSG
                detection_items = []
                for det in detections:
                    item = CVDetectionItemMSG()
                    item.object_type = det.get("object_type", "")
                    item.class_name = det.get("class", "")
                    item.confidence = det.get("confidence", 0.0)
                    item.bbox = det.get("bbox", [])
                    item.bbox_center = det.get("bbox_center", [])
                    detection_items.append(item)
                
                detection_msg.detections = detection_items
                self.detection_publisher.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento da imagem: {e}")

    # ==================== MÉTODOS DE PROCESSAMENTO (ALGORITMOS DE VISÃO COMPUTACIONAL) ================

    def detect_objects(self, image):
        """
        Detecta objetos na imagem usando YOLO e retorna a imagem com bounding boxes.
        
        Versão otimizada que faz conversão GPU->CPU uma única vez por frame,
        reduzindo transferências de memória e melhorando performance.
        
        Args:
            image (numpy.ndarray): Imagem OpenCV no formato BGR (numpy array)
            
        Returns:
            tuple: (imagem_anotada, lista_deteccoes)
                - imagem_anotada: Imagem com bounding boxes e labels desenhados
                - lista_deteccoes: Lista de dicionários com informações das detecções
        """
        if self.yolo_model is None:
            return image, []

        try:
            # Inferência (forçando GPU)
            results = self.yolo_model.predict(image, verbose=False, device=0)
            if not results:
                return image, []

            result = results[0]
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                return image, []

            annotated_image = image.copy()

            # Converte UMA vez por frame (GPU -> CPU) - muito mais eficiente
            xyxy = boxes.xyxy.cpu().numpy()              # (N, 4)
            conf = boxes.conf.cpu().numpy()              # (N,)
            cls  = boxes.cls.cpu().numpy().astype(int)   # (N,)

            detections = []
            for (x1, y1, x2, y2), confidence, class_id in zip(xyxy, conf, cls):
                # Filtra detecções com confiança mínima de 0.5 (50%)
                if confidence <= 0.5:
                    continue

                class_name = self.yolo_model.names[class_id]

                # Desenho (CPU/OpenCV)
                x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(annotated_image, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)

                label = f"{class_name}: {float(confidence):.2f}"
                cv2.putText(annotated_image, label, (x1i, y1i - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calcula centro do bounding box
                bbox_center_x = (x1 + x2) / 2.0
                bbox_center_y = (y1 + y2) / 2.0

                # Determina tipo de objeto baseado no nome da classe
                object_type = (
                    "flare" if "flare" in class_name.lower()
                    else "anomalia" if "anomalia" in class_name.lower()
                    else class_name.lower()
                )

                detections.append({
                    "object_type": object_type,
                    "class": class_name,
                    "confidence": float(confidence),
                    "bbox": [x1i, y1i, x2i, y2i],
                    "bbox_center": [float(bbox_center_x), float(bbox_center_y)]
                })

            return annotated_image, detections

        except Exception as e:
            self.get_logger().error(f"Erro na detecção de objetos: {e}")
            return image, []

def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    cv_node = CVNode()
    
    try:
        rclpy.spin(cv_node)
    except KeyboardInterrupt:
        pass
    finally:
        cv_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

