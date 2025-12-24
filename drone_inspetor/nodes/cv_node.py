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
from sensor_msgs.msg import Image
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
        except Exception as e:
            # Se o modelo não puder ser carregado, o nó continua funcionando
            # mas não realizará detecções (retornará imagens sem anotações)
            self.get_logger().error(f"Erro ao carregar modelo YOLO: {e}")
            self.get_logger().error(f"Tentando carregar de: {model_path}")
            self.yolo_model = None
        
        # ==================== CONFIGURAÇÃO DE QoS ========================
        # QoS para dados de sensores (imagens): VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==================== SUBSCRIBERS INTERNOS (ENTRADA DE DADOS DA CÂMERA) ========================
        # Assina o tópico interno de imagens da câmera
        # As imagens já foram processadas pelo camera_node e estão no formato padronizado
        self.raw_image_subscription = self.create_subscription(
            Image,
            "/drone_inspetor/interno/camera_node/image_raw",
            self.image_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado tópico interno: {self.raw_image_subscription.topic_name}")
        
        # ==================== PUBLISHERS INTERNOS (SAÍDA DE DADOS PARA O DASHBOARD) ====================
        # Publica imagens processadas com anotações (bounding boxes e labels)
        # Estas imagens são consumidas pelo dashboard para visualização
        self.processed_image_publisher = self.create_publisher(
            Image,
            "/drone_inspetor/interno/cv_node/image_processed",
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

    def image_callback(self, msg):
        """
        Callback para processar imagens recebidas da câmera.
        
        Este método é chamado sempre que uma nova imagem é recebida. Ele:
        1. Converte a mensagem ROS para formato OpenCV
        2. Aplica detecção de objetos usando YOLO
        3. Publica a imagem anotada
        4. Publica dados estruturados de detecção como mensagem ROS
        
        Args:
            msg (sensor_msgs.msg.Image): Mensagem de imagem recebida da câmera.
        """
        try:
            # Converte mensagem ROS para imagem OpenCV no formato BGR8
            # BGR8 é o formato padrão do OpenCV (Blue-Green-Red, 8 bits por canal)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Aplica detecção de objetos usando YOLO
            # Retorna imagem anotada e lista de detecções
            annotated_image, detections = self.detect_objects(cv_image)
            
            # Publica imagem processada com anotações
            try:
                # Converte imagem OpenCV de volta para mensagem ROS
                processed_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
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
        
        Este método aplica o modelo YOLO à imagem fornecida e processa os resultados:
        - Filtra detecções com confiança abaixo do threshold (0.5)
        - Desenha bounding boxes verdes ao redor dos objetos detectados
        - Adiciona labels com nome da classe e nível de confiança
        - Retorna lista estruturada de detecções
        
        Args:
            image (numpy.ndarray): Imagem OpenCV no formato BGR (numpy array)
            
        Returns:
            tuple: (imagem_anotada, lista_deteccoes)
                - imagem_anotada: Imagem com bounding boxes e labels desenhados
                - lista_deteccoes: Lista de dicionários, cada um contendo:
                    - "class": Nome da classe detectada
                    - "confidence": Nível de confiança (0.0 a 1.0)
                    - "bbox": [x1, y1, x2, y2] coordenadas do bounding box
        """
        # Se o modelo YOLO não foi carregado, retorna imagem original sem detecções
        if self.yolo_model is None:
            return image, []
        
        try:
            # Executa detecção com YOLO
            # O modelo processa a imagem e retorna resultados contendo:
            # - Coordenadas dos bounding boxes
            # - Classes detectadas
            # - Níveis de confiança
            # verbose=False suprime as mensagens de debug do YOLO no terminal
            results = self.yolo_model(image, verbose=False)
            
            # Processa resultados da detecção
            detections = []
            annotated_image = image.copy()  # Cria cópia para não modificar a imagem original
            
            # Itera sobre os resultados (geralmente há apenas um resultado por imagem)
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    # Itera sobre cada bounding box detectado
                    for box in boxes:
                        # Extrai coordenadas do bounding box (canto superior esquerdo e inferior direito)
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        # Extrai nível de confiança da detecção
                        confidence = box.conf[0].cpu().numpy()
                        # Extrai ID da classe detectada
                        class_id = int(box.cls[0].cpu().numpy())
                        # Obtém nome da classe a partir do ID
                        class_name = self.yolo_model.names[class_id]
                        
                        # Filtra detecções com confiança mínima de 0.5 (50%)
                        # Isso elimina falsos positivos e detecções incertas
                        if confidence > 0.5:
                            # Desenha bounding box verde (BGR: 0, 255, 0) com espessura 2
                            cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            
                            # Cria label com nome da classe e confiança formatada
                            label = f"{class_name}: {confidence:.2f}"
                            # Desenha label acima do bounding box
                            cv2.putText(annotated_image, label, (int(x1), int(y1) - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            # Calcula o centro do bounding box
                            bbox_center_x = (x1 + x2) / 2.0
                            bbox_center_y = (y1 + y2) / 2.0
                            
                            # Determina o tipo de objeto baseado no nome da classe
                            # Mapeia classes para tipos de objeto (flare, anomalia, etc.)
                            object_type = "flare" if "flare" in class_name.lower() else "anomalia" if "anomalia" in class_name.lower() else class_name.lower()
                            
                            # Armazena detecção em formato estruturado
                            detection = {
                                "object_type": object_type,
                                "class": class_name,
                                "confidence": float(confidence),
                                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                                "bbox_center": [float(bbox_center_x), float(bbox_center_y)]
                            }
                            detections.append(detection)
            
            return annotated_image, detections
            
        except Exception as e:
            self.get_logger().error(f"Erro na detecção de objetos: {e}")
            # Em caso de erro, retorna imagem original sem detecções
            return image, []

def main(args=None):
    """
    Função principal para iniciar o nó de visão computacional.
    
    Args:
        args: Argumentos de linha de comando (opcional)
    """
    # Inicializa o ROS2
    rclpy.init(args=args)
    
    # Cria e executa o nó de visão computacional
    cv_node = CVNode()
    rclpy.spin(cv_node)
    
    # Limpeza ao encerrar
    cv_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
