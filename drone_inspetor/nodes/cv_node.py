"""
cv_node.py
=================================================================================================
Nó ROS2 responsável pelo processamento de visão computacional.

Este nó implementa algoritmos de detecção de objetos usando YOLO (You Only Look Once),
uma rede neural convolucional para detecção de objetos em tempo real. O nó processa
imagens recebidas da câmera principal, detecta objetos de interesse (como flares, anomalias),
e publica tanto imagens anotadas quanto dados estruturados de detecção.

ARQUITETURA:
- Assina: /drone_inspetor/externo/camera/compressed (imagens comprimidas da câmera)
- Publica: /drone_inspetor/interno/cv_node/compressed (imagens anotadas comprimidas)
- Publica: /drone_inspetor/interno/cv_node/object_detections (dados de detecção em JSON)

PARÂMETROS ROS2 (definidos em param_ros.yaml):
- anomaly_photo_interval_seconds: Intervalo entre capturas de fotos de anomalias (segundos)
- object_detection_min_confidence: Confiança mínima para detecção de objetos (0.0 a 1.0)
- anomaly_detection_min_confidence: Confiança mínima para detecção de anomalias (0.0 a 1.0)
- photo_format: Formato das imagens (jpg, png)
- photo_quality: Qualidade de compressão JPEG (1-100)
- video_fps: FPS do vídeo de detecções
- video_codec: Codec de vídeo (MJPG recomendado, XVID alternativa)

ALGORITMO:
- Utiliza modelo YOLO customizado para detecção de objetos específicos
- Aplica threshold de confiança mínimo configurável para filtrar detecções
- Desenha bounding boxes e labels nas imagens processadas
=================================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import threading
from datetime import datetime
from ultralytics import YOLO
import os
import time
from ament_index_python.packages import get_package_share_directory

# Importação das mensagens ROS customizadas
from drone_inspetor_msgs.msg import CVDetectionMSG, CVDetectionItemMSG, FSMStateMSG, CVControlMSG
from drone_inspetor_msgs.srv import CVDetectionSRV, RecordDetectionsSRV, EnableAnomalyDetectionSRV, CVModelsSRV
import json

# Gerador do relatório da missão (relatorio_da_missao.md)
from drone_inspetor.reports.mission_report import MissionReport


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
        
        # Flag para controle de shutdown limpo
        self._is_shutting_down = False
        
        # ==================== GRUPOS DE CALLBACK (CONCORRÊNCIA) ====================
        # Grupos separados para evitar que serviços longos (com loops de espera)
        # bloqueiem o processamento de imagens (deadlock).
        self.camera_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # ==================== INICIALIZAÇÃO ============================================================
        # Cria instância do CvBridge para conversão entre formatos ROS e OpenCV
        self.bridge = CvBridge()
        
        # Estado da FSM atual
        self._current_fsm_state = ""
        self._on_mission = False
        self._last_annotated_image = None  # Última imagem com anotações

        # === Relatório da missão (relatorio_da_missao.md) ===
        self._report = None            # Instância de MissionReport (criada no início da missão)
        self._mission_name = ""        # Nome da missão atual (para o relatório)
        self._last_milestone_label = ""  # Último rótulo de marco (waypoint) registrado
        
        # Controle de fotos
        self._photos_folder = ""
        self._photo_counter = 0
        self._ponto_indice_atual = 0
        self._objeto_alvo = ""
        
        # === Variáveis para service de detecção ===
        self._last_detections = []  # Últimas detecções encontradas
        
        # === Variáveis para gravação de vídeo ===
        # Lock protege o _video_writer: image_callback (camera_cb_group) e
        # record_service_callback (service_cb_group) rodam em threads paralelas no
        # MultiThreadedExecutor; escrever um frame durante o release() corrompe o
        # muxer (warnings "Invalid pts ... <= last ...") e o vídeo.
        self._video_lock = threading.Lock()
        self._is_recording = False
        self._video_writer = None
        self._video_path = ""
        self._videos_folder = ""  # Pasta para salvar vídeos CV
        self._video_frame_size = (1280, 720)  # Tamanho padrão do vídeo
        
        # === Flag para controle de detecção de anomalias ===
        self._anomaly_detection_enabled = False  # Só roda rede de anomalias quando True
        
        # === Controle de tempo para fotos de anomalias ===
        self._last_anomaly_photo_time = 0.0  # Timestamp da última foto de anomalia
        
        # Declara parâmetro para intervalo de fotos de anomalias (default: 4 segundos)
        self.declare_parameter('anomaly_photo_interval_seconds', 4.0)
        self._anomaly_photo_interval = self.get_parameter('anomaly_photo_interval_seconds').value
        self.get_logger().info(f"Intervalo fotos anomalias: {self._anomaly_photo_interval}s")
        
        # Declara parâmetros de confiança mínima
        self.declare_parameter('object_detection_min_confidence', 0.5)
        self._object_min_confidence = self.get_parameter('object_detection_min_confidence').value
        
        self.declare_parameter('anomaly_detection_min_confidence', 0.5)
        self._anomaly_min_confidence = self.get_parameter('anomaly_detection_min_confidence').value
        
        self.get_logger().info(f"Confiança mínima: objetos={self._object_min_confidence}, anomalias={self._anomaly_min_confidence}")
        
        # ==================== PARÂMETROS DE FOTO E VÍDEO =============================================
        # Parâmetros de foto (similar ao camera_node)
        self.declare_parameter("photo_format", "jpg")
        self.declare_parameter("photo_quality", 95)
        self._photo_format = self.get_parameter("photo_format").get_parameter_value().string_value
        self._photo_quality = self.get_parameter("photo_quality").get_parameter_value().integer_value
        
        # Parâmetros de vídeo
        self.declare_parameter("video_fps", 15)
        self.declare_parameter("video_codec", "mp4v")
        self._video_fps = self.get_parameter("video_fps").get_parameter_value().integer_value
        self._video_codec = self.get_parameter("video_codec").get_parameter_value().string_value
        
        self.get_logger().info(f"CV Foto: formato={self._photo_format}, qualidade={self._photo_quality}")
        self.get_logger().info(f"CV Vídeo: fps={self._video_fps}, codec={self._video_codec}")

        # ==================== PARÂMETRO DE IDENTIFICAÇÃO DO DISPOSITIVO ===========================
        # Rótulo do dispositivo usado no relatório da missão para diferenciar as plataformas
        # na comparação (ex.: "RPI5", "Jetson Orin Nano", "PC"). "auto" deduz de
        # /proc/device-tree/model (Raspberry Pi / Jetson) ou do hostname.
        self.declare_parameter("device_label", "auto")
        self._device_label = self.get_parameter("device_label").get_parameter_value().string_value
        self.get_logger().info(f"Relatório de missão: dispositivo='{self._device_label}'")
        
        # ==================== CARREGAMENTO DOS MODELOS YOLO ===============================================
        # Obtém o caminho do diretório de instalação do pacote
        self._pkg_share_dir = get_package_share_directory('drone_inspetor')
        
        # Carrega configuração de modelos disponíveis do models.json
        self._available_models = self._load_models_json()
        
        # Nomes dos arquivos dos modelos atualmente carregados
        self._current_object_model_file = ""
        self._current_anomaly_model_file = ""
        
        # Define modelos iniciais baseados no primeiro disponível de cada tipo
        for model in self._available_models:
            filename = model.get('file_name', '')
            obj_type = model.get('object_type', '')
            
            if not self._current_object_model_file and obj_type == 'equipment':
                self._current_object_model_file = filename
            
            if not self._current_anomaly_model_file and obj_type == 'anomaly':
                self._current_anomaly_model_file = filename
            
            # Se já encontrou ambos, para
            if self._current_object_model_file and self._current_anomaly_model_file:
                break
        
        # Modelo 1: Detecção de objetos da plataforma (Flare, roldanas, etc.)
        objects_model_path = os.path.join(self._pkg_share_dir, 'redes_treinadas', self._current_object_model_file)
        
        # Modelo 2: Detecção de anomalias (corrosão) - usado em crops dos objetos detectados
        anomalies_model_path = os.path.join(self._pkg_share_dir, 'redes_treinadas', self._current_anomaly_model_file)
        
        import torch
        if torch.cuda.is_available():
            self.get_logger().info(f"GPU disponível: {torch.cuda.get_device_name(0)}")
        else:
            self.get_logger().warn("------------------------------------------------")
            self.get_logger().warn("GPU indisponível! - Redes YOLO executadas na CPU")
        
        # Carrega modelo de objetos da plataforma
        try:
            self.yolo_model_objects = YOLO(objects_model_path)
            self.get_logger().info(f"✅ Modelo de objetos carregado: {os.path.basename(objects_model_path)}")
            self.get_logger().info(f"   Classes disponíveis: {self.yolo_model_objects.names}")
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar modelo de objetos: {e}")
            self.yolo_model_objects = None
        
        # Carrega modelo de detecção de anomalias (corrosão)
        try:
            self.yolo_model_anomalies = YOLO(anomalies_model_path)
            self.get_logger().info(f"✅ Modelo de anomalias carregado: {os.path.basename(anomalies_model_path)}")
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar modelo de anomalias: {e}")
            self.yolo_model_anomalies = None
        
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
            qos_sensor_data,
            callback_group=self.camera_cb_group
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
        
        # ==================== SUBSCRIBER FSM STATE =========================
        # ==================== SUBSCRIBER FSM STATE =========================
        # QoS para mensagens de estado (BEST_EFFORT + TRANSIENT_LOCAL para match com publisher)
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Assina estado da FSM para detectar quando tirar fotos
        self.fsm_state_sub = self.create_subscription(
            FSMStateMSG,
            "/drone_inspetor/interno/fsm_node/fsm_state",
            self.fsm_state_callback,
            qos_state
        )
        self.get_logger().info("Assinado tópico FSM State para monitoramento de missão.")
        
        # ==================== SERVICES ====================
        # Service para solicitar detecção de objeto
        self.detection_service = self.create_service(
            CVDetectionSRV,
            '/drone_inspetor/interno/cv_node/srv/detection',
            self.detection_service_callback,
            callback_group=self.service_cb_group
        )
        self.get_logger().info("Service de detecção criado: /drone_inspetor/interno/cv_node/srv/detection")
        
        # Service para controlar gravação de vídeo
        self.record_service = self.create_service(
            RecordDetectionsSRV,
            '/drone_inspetor/interno/cv_node/srv/record_detections',
            self.record_service_callback,
            callback_group=self.service_cb_group
        )
        self.get_logger().info("Service de gravação criado: /drone_inspetor/interno/cv_node/srv/record_detections")
        
        # Service para habilitar/desabilitar detecção de anomalias
        self.anomaly_detection_service = self.create_service(
            EnableAnomalyDetectionSRV,
            '/drone_inspetor/interno/cv_node/srv/enable_anomaly_detection',
            self.enable_anomaly_detection_callback,
            callback_group=self.service_cb_group
        )
        self.get_logger().info("Service de anomalias criado: /drone_inspetor/interno/cv_node/srv/enable_anomaly_detection")
        
        # Service para listar modelos disponíveis
        self.cv_models_service = self.create_service(
            CVModelsSRV,
            '/drone_inspetor/interno/cv_node/srv/list_models',
            self.cv_models_service_callback,
            callback_group=self.service_cb_group
        )
        self.get_logger().info("Service de modelos criado: /drone_inspetor/interno/cv_node/srv/list_models")

        # ==================== SUBSCRIBER PARA CONTROLE DE MODELOS ====================
        # QoS para comandos (RELIABLE para garantir entrega)
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber para receber seleção de modelos do dashboard
        self.cv_control_sub = self.create_subscription(
            CVControlMSG,
            "/drone_inspetor/interno/dashboard_node/cv_node/cv_control",
            self._cv_control_callback,
            qos_commands
        )
        self.get_logger().info(f"Subscriber CVControlMSG criado: {self.cv_control_sub.topic_name}")

        self.get_logger().info("CVNode inicializado com sucesso.")

    def destroy_node(self):
        """
        Override do método destroy_node para garantir shutdown limpo.
        """
        self._is_shutting_down = True
        self.get_logger().info("Encerrando cv_node... (Flag _is_shutting_down=True)")

        # Finaliza o relatório da missão se ainda estiver ativo (encerramento no meio da missão)
        if self._report is not None:
            try:
                self._report.finish("INTERROMPIDA (cv_node encerrado)")
            except Exception as e:
                self.get_logger().error(f"Erro ao finalizar relatório no shutdown: {e}")
            self._report = None

        # Libera recursos de vídeo se estiver gravando
        with self._video_lock:
            self._is_recording = False
            if self._video_writer:
                self._video_writer.release()
                self._video_writer = None

        super().destroy_node()

    
    # ==================== MÉTODOS DE GERENCIAMENTO DE MODELOS ====================
    
    def _load_models_json(self):
        """
        Carrega a lista de modelos disponíveis do arquivo models.json.
        Adapta a nova estrutura hierárquica (equipment/anomaly) para uma lista plana
        com o campo 'object_type' injetado, mantendo compatibilidade com o restante do sistema.
        
        Returns:
            list: Lista de dicionários com informações dos modelos.
        """
        try:
            models_json_path = os.path.join(self._pkg_share_dir, 'redes_treinadas', 'models.json')
            
            with open(models_json_path, 'r') as f:
                data = json.load(f)
            
            # Suporta tanto a estrutura antiga (lista) quanto a nova (dict)
            raw_models = data.get('models', {})
            
            flat_models = []
            
            if isinstance(raw_models, list):
                # Estrutura antiga: já é uma lista plana
                flat_models = raw_models
            elif isinstance(raw_models, dict):
                # Nova estrutura: chaves 'equipment' e 'anomaly'
                equipment_list = raw_models.get('equipment', [])
                anomaly_list = raw_models.get('anomaly', [])
                
                # Injeta object_type e adiciona à lista plana
                for m in equipment_list:
                    m['object_type'] = 'equipment'
                    flat_models.append(m)
                
                for m in anomaly_list:
                    m['object_type'] = 'anomaly'
                    flat_models.append(m)
            
            self.get_logger().info(f"Models.json carregado: {len(flat_models)} modelos disponíveis")
            return flat_models
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar models.json: {e}")
            return []
    
    def _cv_control_callback(self, msg: CVControlMSG):
        """
        Callback para receber comandos de controle de modelos do dashboard.
        
        Args:
            msg (CVControlMSG): Mensagem com modelos selecionados
        """
        self.get_logger().info(f"CVControlMSG recebido: obj={msg.object_detection_model}, anom={msg.anomaly_detection_model}")
        
        # Recarrega modelo de objetos se diferente do atual
        if msg.object_detection_model and msg.object_detection_model != self._current_object_model_file:
            self._load_object_model(msg.object_detection_model)
        
        # Recarrega modelo de anomalias se diferente do atual
        if msg.anomaly_detection_model and msg.anomaly_detection_model != self._current_anomaly_model_file:
            self._load_anomaly_model(msg.anomaly_detection_model)
    
    def _load_object_model(self, model_filename: str):
        """
        Carrega um novo modelo de detecção de objetos.
        
        Args:
            model_filename (str): Nome do arquivo do modelo (ex: 'plataform_objects_yolo8n_detection.pt')
        """
        model_path = os.path.join(self._pkg_share_dir, 'redes_treinadas', model_filename)
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo de objetos não encontrado: {model_path}")
            return
        
        old_model_file = self._current_object_model_file
        try:
            self.get_logger().info(f"🔄 Carregando modelo de objetos: {model_filename}...")
            self.yolo_model_objects = YOLO(model_path)
            self._current_object_model_file = model_filename
            self.get_logger().info(f"✅ Modelo de objetos atualizado: {model_filename}")
            self.get_logger().info(f"   Classes disponíveis: {self.yolo_model_objects.names}")
            # Registra a troca de modelo no relatório da missão (se houver missão ativa)
            if self._report is not None:
                self._report.note_model_change("objetos", old_model_file, model_filename)
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar modelo de objetos {model_filename}: {e}")
    
    def _load_anomaly_model(self, model_filename: str):
        """
        Carrega um novo modelo de detecção de anomalias.
        
        Args:
            model_filename (str): Nome do arquivo do modelo (ex: 'corrosion_yolo8x_detection.pt')
        """
        model_path = os.path.join(self._pkg_share_dir, 'redes_treinadas', model_filename)
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Modelo de anomalias não encontrado: {model_path}")
            return
        
        old_model_file = self._current_anomaly_model_file
        try:
            self.get_logger().info(f"🔄 Carregando modelo de anomalias: {model_filename}...")
            self.yolo_model_anomalies = YOLO(model_path)
            self._current_anomaly_model_file = model_filename
            self.get_logger().info(f"✅ Modelo de anomalias atualizado: {model_filename}")
            # Registra a troca de modelo no relatório da missão (se houver missão ativa)
            if self._report is not None:
                self._report.note_model_change("anomalias", old_model_file, model_filename)
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar modelo de anomalias {model_filename}: {e}")

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
            # Verifica se o nó está encerrando antes de processar
            if self._is_shutting_down:
                return

            # Marca o frame que CHEGOU ao cv_node (FR de imagens recebidas no relatório)
            if self._report is not None:
                self._report.mark_frame_received()

            # Converte mensagem CompressedImage ROS para imagem OpenCV no formato BGR8
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Aplica detecção de objetos usando YOLO. As latências de inferência (objetos e
            # anomalias) são medidas DENTRO de detect_objects em TEMPO DE PAREDE real
            # (perf_counter), independente de use_sim_time — essencial para comparar o
            # desempenho entre RPI5, Jetson e PC.
            annotated_image, detections, timing = self.detect_objects(cv_image)

            # Registra latências e nº de bounding boxes do frame no relatório da missão
            if self._report is not None:
                n_obj_bboxes = len(detections)
                n_anom_bboxes = sum(len(d.get("anomalies", [])) for d in detections)
                self._report.record_frame(
                    obj_ms=timing.get("obj_ms"),
                    anom_ms=timing.get("anom_ms"),
                    n_obj_bboxes=n_obj_bboxes,
                    n_anom_bboxes=n_anom_bboxes,
                    anomaly_active=timing.get("anomaly_active", False),
                )
            
            # Publica imagem processada com anotações como CompressedImage
            try:
                # Converte imagem OpenCV para mensagem CompressedImage ROS
                processed_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format="jpeg")
                # Preserva o header original (timestamp, frame_id, etc.)
                processed_msg.header = msg.header
                
                # Double-check antes de publicar
                if not self._is_shutting_down:
                    self.processed_image_publisher.publish(processed_msg)
            except Exception as e:
                self.get_logger().error(f"Erro ao publicar imagem processada: {e}")
            
            # Publica dados de detecção como mensagem ROS
            # Apenas publica se houver detecções
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
            detection_msg.detections = detection_items
            
            # Double-check antes de publicar
            if not self._is_shutting_down:
                self.detection_publisher.publish(detection_msg)

            
            # Armazena as últimas detecções para o service
            self._last_detections = detections
            
            # Armazena a última imagem anotada para captura de foto
            self._last_annotated_image = annotated_image
            
            # Grava frame se gravação estiver ativa.
            # O lock garante que o writer não seja liberado por outra thread
            # (record_service_callback) entre o teste e o write().
            with self._video_lock:
                if self._is_recording and self._video_writer is not None:
                    try:
                        # Redimensiona para tamanho padrão do vídeo
                        frame_resized = cv2.resize(annotated_image, self._video_frame_size)
                        self._video_writer.write(frame_resized)
                    except Exception as e:
                        self.get_logger().error(f"Erro ao gravar frame: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento da imagem: {e}")
    
    def fsm_state_callback(self, msg: FSMStateMSG):
        """
        Callback para mensagens de estado da FSM.
        Monitora início/fim de missões e estados que disparam captura de foto.
        """
        new_state = msg.state_name
        new_on_mission = msg.on_mission
        new_mission_folder = msg.mission_folder_path
        
        # ========== INÍCIO DE MISSÃO (configura pastas) ==========
        if new_on_mission and not self._on_mission:
            self._photos_folder = os.path.join(new_mission_folder, "fotos_cv") if new_mission_folder else ""
            self._videos_folder = os.path.join(new_mission_folder, "videos_cv") if new_mission_folder else ""
            self._photo_counter = 0
            
            # Cria pasta de fotos CV se não existir
            if self._photos_folder and not os.path.exists(self._photos_folder):
                try:
                    os.makedirs(self._photos_folder, exist_ok=True)
                    self.get_logger().info(f"📷 Pasta de fotos CV criada: {self._photos_folder}")
                except Exception as e:
                    self.get_logger().error(f"Erro ao criar pasta de fotos CV: {e}")

            # === Cria o relatório da missão (relatorio_da_missao.md na raiz da pasta) ===
            self._mission_name = msg.mission_name
            self._start_mission_report(new_mission_folder, msg)

        # ========== ATUALIZAÇÃO DE INFORMAÇÕES DO PONTO ==========
        if new_on_mission:
            self._ponto_indice_atual = msg.ponto_de_inspecao_indice_atual
            self._objeto_alvo = msg.objeto_alvo
        
        # ========== FIM DE MISSÃO (limpa variáveis) ==========
        if not new_on_mission and self._on_mission:
            self.get_logger().info(f"Missão finalizada. Fotos CV: {self._photo_counter}")
            self._photos_folder = ""
            self._videos_folder = ""
            # Finaliza e grava a versão final do relatório da missão
            self._finish_mission_report("CONCLUÍDA")

        # ========== CAPTURA DE FOTO ==========
        # Tira foto sempre que FSM transicionar de INSPECIONANDO para DETECTANDO
        # IMPORTANTE: verificar ANTES de atualizar _current_fsm_state
        if new_on_mission and new_state == "EXECUTANDO_INSPECIONANDO_DETECTANDO":
            if self._current_fsm_state == "EXECUTANDO_INSPECIONANDO":
                self._capture_photo(new_state)

        # ========== REGISTRO DE EVENTOS E MARCOS DA MISSÃO NO RELATÓRIO ==========
        if self._report is not None and new_on_mission:
            # Marco (trecho): muda em Decolagem → Ponto Pk → RTL e quando o índice
            # do ponto avança (mesmo sem mudar o estado da FSM).
            self._update_mission_milestone(new_state, msg.ponto_de_inspecao_indice_atual)

            # Transição de estado da FSM (linha do tempo de eventos)
            if new_state != self._current_fsm_state:
                self._report.note_state_transition(
                    self._current_fsm_state or "(início)", new_state,
                    ponto_idx=msg.ponto_de_inspecao_indice_atual,
                    objeto_alvo=msg.objeto_alvo,
                )
                # A transição para ESCANEANDO indica que o objeto-alvo foi confirmado/detectado
                if new_state == "EXECUTANDO_INSPECIONANDO_ESCANEANDO":
                    self._report.note_target_detected(
                        msg.objeto_alvo, ponto_idx=msg.ponto_de_inspecao_indice_atual
                    )
            self._report.set_mission_info(points_total=msg.total_pontos_de_inspecao)
            self._report.note_point_reached(msg.ponto_de_inspecao_indice_atual)

        # Atualiza estado (APÓS verificação de captura de foto)
        self._on_mission = new_on_mission
        self._current_fsm_state = new_state

    def _start_mission_report(self, mission_folder: str, msg: FSMStateMSG):
        """
        Cria e inicia o relatório da missão (relatorio_da_missao.md).

        O relatório é criado na RAIZ da pasta da missão (ao lado de fotos/, videos/,
        fotos_cv/ e videos_cv/) e é preenchido continuamente durante a execução.
        """
        # Fecha qualquer relatório anterior que tenha ficado aberto
        if self._report is not None:
            try:
                self._report.finish("INTERROMPIDA (nova missão iniciada)")
            except Exception:
                pass
            self._report = None

        if not mission_folder:
            self.get_logger().warn("Pasta da missão vazia: relatório da missão não será gerado.")
            return

        try:
            self._report = MissionReport(
                mission_folder=mission_folder,
                mission_name=msg.mission_name,
                device_label=self._device_label,
                available_models=self._available_models,
                logger=self.get_logger(),
            )
            # Modelos de CV ativos no início da missão
            self._report.set_initial_models(
                self._current_object_model_file, self._current_anomaly_model_file
            )
            self._report.set_mission_info(points_total=msg.total_pontos_de_inspecao)
            self._report.start()
            # Abre o primeiro trecho já com o rótulo do marco correspondente ao estado atual
            self._last_milestone_label = ""
            self._update_mission_milestone(msg.state_name, msg.ponto_de_inspecao_indice_atual)
            self.get_logger().info(f"📝 Relatório da missão iniciado: {self._report.report_path}")
        except Exception as e:
            self.get_logger().error(f"Erro ao iniciar relatório da missão: {e}")
            self._report = None

    def _finish_mission_report(self, status: str):
        """Finaliza e grava a versão final do relatório da missão atual, se houver."""
        if self._report is None:
            return
        try:
            self._report.finish(status)
            self.get_logger().info(f"📝 Relatório da missão finalizado: {self._report.report_path}")
        except Exception as e:
            self.get_logger().error(f"Erro ao finalizar relatório da missão: {e}")
        finally:
            self._report = None
            self._last_milestone_label = ""

    def _milestone_label(self, state_name: str, ponto_idx: int):
        """
        Deriva o rótulo do MARCO (trecho da missão) a partir do estado da FSM.

        Marcos: 'Decolagem' (armando/decolando), 'Ponto Pkk' (navegação + inspeção de
        cada ponto de inspeção) e 'RTL (retorno)'. Retorna None para estados que não
        delimitam um trecho (PRONTO/DESATIVADO), mantendo o trecho anterior aberto.
        """
        if not state_name:
            return None
        if state_name in ("EXECUTANDO_ARMANDO", "EXECUTANDO_DECOLANDO"):
            return "Decolagem"
        if state_name.startswith("EXECUTANDO_INSPECIONANDO"):
            return f"Ponto P{ponto_idx + 1:02d}"
        if state_name == "RETORNANDO":
            return "RTL (retorno)"
        return None

    def _update_mission_milestone(self, state_name: str, ponto_idx: int):
        """
        Fecha o trecho atual e abre um novo no relatório quando o marco muda.

        O marco muda na transição Decolagem → Ponto Pk → RTL e também quando o índice
        do ponto de inspeção avança (P01 → P02), mesmo sem mudança de estado da FSM.
        """
        if self._report is None:
            return
        label = self._milestone_label(state_name, ponto_idx)
        if label and label != self._last_milestone_label:
            self._last_milestone_label = label
            self._report.mark_waypoint(label)

    def _capture_photo(self, state_name: str):
        """
        Captura e salva uma foto anotada (com bounding boxes) na pasta fotos_cv da missão.
        """
        if self._last_annotated_image is None:
            self.get_logger().warn("Nenhuma imagem anotada disponível para captura.")
            return
        
        if not self._photos_folder or not os.path.exists(self._photos_folder):
            self.get_logger().warn("Pasta de fotos CV não existe. Foto não salva.")
            return
        
        try:
            self._photo_counter += 1
            timestamp = datetime.now().strftime("%H%M%S")
            
            # Formato: 001_P01_detectando_flare_093012.jpg
            prefix = state_name.lower().replace("executando_inspecionando_", "")
            ponto_str = f"P{self._ponto_indice_atual + 1:02d}"  # P01, P02, etc.
            objeto_str = f"_{self._objeto_alvo}" if self._objeto_alvo else ""
            filename = f"{self._photo_counter:03d}_{ponto_str}_{prefix}{objeto_str}_{timestamp}.{self._photo_format}"
            
            photo_path = os.path.join(self._photos_folder, filename)
            
            # Configura parâmetros de compressão
            if self._photo_format.lower() == "jpg":
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._photo_quality]
            elif self._photo_format.lower() == "png":
                png_compression = max(0, min(9, 9 - int(self._photo_quality / 11)))
                encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression]
            else:
                encode_params = []
            
            # Salva a imagem anotada
            cv2.imwrite(photo_path, self._last_annotated_image, encode_params)
            self.get_logger().info(f"🔍 Foto CV capturada: {os.path.basename(photo_path)}")

            # Registra a foto no relatório da missão
            if self._report is not None:
                self._report.note_photo("cv")

        except Exception as e:
            self.get_logger().error(f"Erro ao capturar foto CV: {e}")

    def _capture_anomaly_photos(self, img_original: np.ndarray, img_objeto: np.ndarray,
                                  img_anomalias: np.ndarray, crop_original: np.ndarray,
                                  crop_anomalias: np.ndarray, object_name: str, n_anomalias: int = 0):
        """
        Salva 5 fotos do MESMO momento quando anomalias são detectadas.
        Só salva se passou pelo menos 1 segundo desde a última captura.
        
        Todas as fotos têm a mesma resolução da imagem original.
        Sequencial: <seq_momento>_<seq_foto> (ex: 1_1, 1_2, etc.)
        
        As 5 fotos são salvas instantaneamente (mesmo timestamp):
        1. original: imagem completa sem anotações
        2. objeto: imagem completa com BB do objeto (verde)
        3. anomalias: imagem completa com BB objeto + anomalias (verde + vermelho)
        4. crop_original: crop do objeto redimensionado para resolução original
        5. crop_anomalias: crop do objeto com anomalias redimensionado
        """
        current_time = time.time()
        
        # Só salva se passou o intervalo configurado desde a última foto
        if (self.get_clock().now().nanoseconds / 1e9) - self._last_anomaly_photo_time < self._anomaly_photo_interval:
            return
        
        if not self._photos_folder or not os.path.exists(self._photos_folder):
            return
        
        try:
            self._last_anomaly_photo_time = self.get_clock().now().nanoseconds / 1e9
            self._photo_counter += 1
            seq_momento = self._photo_counter
            
            ponto_str = f"P{self._ponto_indice_atual + 1:02d}"
            obj_str = object_name.lower().replace(" ", "_")
            timestamp = datetime.now().strftime("%H%M%S")
            
            # Resolução da imagem original para redimensionar crops
            h, w = img_original.shape[:2]
            
            # Redimensiona crops para mesma resolução da imagem original
            crop_original_resized = cv2.resize(crop_original, (w, h), interpolation=cv2.INTER_LANCZOS4)
            crop_anomalias_resized = cv2.resize(crop_anomalias, (w, h), interpolation=cv2.INTER_LANCZOS4)
            
            # Salva as 5 fotos do MESMO momento (seq: momento_foto)
            ext = self._photo_format
            f1 = f"{ponto_str}_{obj_str}_{seq_momento}_1_original_{timestamp}.{ext}"
            f2 = f"{ponto_str}_{obj_str}_{seq_momento}_2_objeto_{timestamp}.{ext}"
            f3 = f"{ponto_str}_{obj_str}_{seq_momento}_3_anomalias_{timestamp}.{ext}"
            f4 = f"{ponto_str}_{obj_str}_{seq_momento}_4_crop_{timestamp}.{ext}"
            f5 = f"{ponto_str}_{obj_str}_{seq_momento}_5_crop_anomalias_{timestamp}.{ext}"
            
            # Parâmetros de compressão
            if self._photo_format.lower() == "jpg":
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._photo_quality]
            elif self._photo_format.lower() == "png":
                png_compression = max(0, min(9, 9 - int(self._photo_quality / 11)))
                encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression]
            else:
                encode_params = []
            
            cv2.imwrite(os.path.join(self._photos_folder, f1), img_original, encode_params)
            cv2.imwrite(os.path.join(self._photos_folder, f2), img_objeto, encode_params)
            cv2.imwrite(os.path.join(self._photos_folder, f3), img_anomalias, encode_params)
            cv2.imwrite(os.path.join(self._photos_folder, f4), crop_original_resized, encode_params)
            cv2.imwrite(os.path.join(self._photos_folder, f5), crop_anomalias_resized, encode_params)

            self.get_logger().info(f"📷 5 fotos anomalias: seq={seq_momento}, ts={timestamp}")

            # Registra o momento de anomalia e as 5 fotos no relatório da missão
            if self._report is not None:
                self._report.note_anomaly_moment(object_name, n_anomalias, ponto_idx=self._ponto_indice_atual)
                for _ in range(5):
                    self._report.note_photo("anomalia")

        except Exception as e:
            self.get_logger().error(f"Erro ao salvar fotos de anomalias: {e}")

    # ==================== SERVICE CALLBACKS ====================

    def detection_service_callback(self, request, response):
        """
        Callback do service de detecção.
        Monitora as detecções por timeout_seconds e retorna se encontrou o objeto.
        
        Args:
            request: CVDetectionSRV.Request com object_name, anomaly_types, timeout_seconds
            response: CVDetectionSRV.Response com success, bbox, bbox_center
        """
        object_name = request.object_name
        timeout = request.timeout_seconds if request.timeout_seconds > 0 else 2.0
        
        self.get_logger().info(f"Service de detecção chamado: buscando '{object_name}' por {timeout}s")
        
        import time
        start_time = self.get_clock().now().nanoseconds / 1e9
        found = False
        best_detection = None

        self.get_logger().info(f" ")
        self.get_logger().info(f"Requested: {object_name}")
        
        # Monitora detecções por timeout segundos
        while (self.get_clock().now().nanoseconds / 1e9) - start_time < timeout:
            # Faz cópia para evitar erro se lista for modificada durante iteração
            detections_snapshot = self._last_detections.copy()
            for det in detections_snapshot:
                det_class = det.get("class", "").lower()
                det_type = det.get("object_type", "").lower()

                self.get_logger().info(f"Detected: {det_class}, {det_type}")
                
                if object_name.lower() in det_class or object_name.lower() in det_type:
                    # Encontrou o objeto
                    if best_detection is None or det.get("confidence", 0) > best_detection.get("confidence", 0):
                        best_detection = det
                        found = True
            
            if found:
                break
            
            time.sleep(0.1)  # Pequena pausa para não sobrecarregar
        
        if found and best_detection:
            response.success = True
            response.message = f"Objeto '{object_name}' detectado com {best_detection['confidence']:.2f} de confiança"
            response.confidence = best_detection.get("confidence", 0.0)
            response.bbox = best_detection.get("bbox", [])
            response.bbox_center = [float(x) for x in best_detection.get("bbox_center", [])]
            self.get_logger().info(f"✅ {response.message}")
        else:
            response.success = False
            response.message = f"Objeto '{object_name}' não detectado em {timeout}s"
            response.confidence = 0.0
            response.bbox = []
            response.bbox_center = []
            self.get_logger().warn(f"❌ {response.message}")
        
        return response

    def record_service_callback(self, request, response):
        """
        Callback do service de gravação de vídeo.
        Inicia ou para a gravação de vídeo com detecções.
        
        Args:
            request: RecordDetectionsSRV.Request com start_recording (bool)
            response: RecordDetectionsSRV.Response com success, message, video_path
        """
        if request.start_recording:
            # Iniciar gravação
            if self._is_recording:
                response.success = False
                response.message = "Gravação já está em andamento"
                response.video_path = ""
                return response
            
            try:
                # Determina extensão baseada no codec
                video_ext = "avi" if self._video_codec in ["MJPG", "XVID"] else "mp4"
                
                # Gera caminho para o vídeo na pasta videos_cv da missão
                if self._videos_folder:
                    # Cria pasta de vídeos CV se não existir
                    if not os.path.exists(self._videos_folder):
                        os.makedirs(self._videos_folder, exist_ok=True)
                    
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    self._video_path = os.path.join(
                        self._videos_folder,
                        f"detection_{timestamp}.{video_ext}"
                    )
                else:
                    # Fallback se não houver missão ativa
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    self._video_path = f"/tmp/detection_{timestamp}.{video_ext}"
                
                # Configura VideoWriter usando parâmetros configuráveis
                fourcc = cv2.VideoWriter_fourcc(*self._video_codec)
                self._video_writer = cv2.VideoWriter(
                    self._video_path,
                    fourcc,
                    float(self._video_fps),
                    self._video_frame_size
                )
                
                if not self._video_writer.isOpened():
                    raise Exception("Não foi possível abrir VideoWriter")
                
                self._is_recording = True
                response.success = True
                response.message = f"Gravação iniciada: {os.path.basename(self._video_path)}"
                response.video_path = self._video_path
                self.get_logger().info(f"🔴 {response.message}")
                
            except Exception as e:
                response.success = False
                response.message = f"Erro ao iniciar gravação: {e}"
                response.video_path = ""
                self.get_logger().error(response.message)
                
        else:
            # Parar gravação
            if not self._is_recording:
                response.success = False
                response.message = "Nenhuma gravação em andamento"
                response.video_path = ""
                return response
            
            try:
                # Para de aceitar novos frames antes de liberar o writer e faz ambos
                # sob o lock, para que image_callback não escreva durante o release().
                with self._video_lock:
                    self._is_recording = False
                    if self._video_writer:
                        self._video_writer.release()
                        self._video_writer = None

                response.success = True
                response.message = f"Gravação finalizada: {os.path.basename(self._video_path)}"
                response.video_path = self._video_path
                self.get_logger().info(f"⬛ {response.message}")

                # Registra o vídeo salvo no relatório da missão
                if self._report is not None and self._video_path:
                    self._report.note_video(self._video_path)
                
            except Exception as e:
                response.success = False
                response.message = f"Erro ao parar gravação: {e}"
                response.video_path = ""
                self.get_logger().error(response.message)
        
        return response

    def enable_anomaly_detection_callback(self, request, response):
        """
        Callback do service para habilitar/desabilitar detecção de anomalias.
        
        Args:
            request: EnableAnomalyDetectionSRV.Request com enable (bool)
            response: EnableAnomalyDetectionSRV.Response com success, message
        """
        self._anomaly_detection_enabled = request.enable
        
        if request.enable:
            response.success = True
            response.message = "Detecção de anomalias HABILITADA"
            self.get_logger().info("🔴 Detecção de anomalias HABILITADA")
        else:
            response.success = True
            response.message = "Detecção de anomalias DESABILITADA"
            self.get_logger().info("⬛ Detecção de anomalias DESABILITADA")
        
        return response

    def cv_models_service_callback(self, request, response):
        """
        Callback do service para listar modelos disponíveis e atuais.
        
        Args:
            request: CVModelsSRV.Request (vazio)
            response: CVModelsSRV.Response com listas de modelos e modelos atuais
        """
        try:
            # Retorna a lista completa de modelos como JSON string
            response.models_data_json = json.dumps(self._available_models)
            response.current_object_model = self._current_object_model_file
            response.current_anomaly_model = self._current_anomaly_model_file
        except Exception as e:
            self.get_logger().error(f"Erro ao serializar modelos: {e}")
            response.models_data_json = "[]"
            response.current_object_model = ""
            response.current_anomaly_model = ""
        
        return response

    # ==================== MÉTODOS DE PROCESSAMENTO (ALGORITMOS DE VISÃO COMPUTACIONAL) ================

    def detect_objects(self, image):
        """
        Detecta objetos e anomalias usando detecção hierárquica em dois estágios.
        
        Estágio 1: Detecta objetos da plataforma (Flare, roldanas, etc.) usando yolo_model_objects
        Estágio 2: Para cada objeto detectado, faz crop e detecta anomalias (corrosão) usando yolo_model_anomalies
        
        Quando o filtro de objeto está ativo (seja por detecção de anomalias habilitada ou pelos estados DETECTANDO/ESCANEANDO):
        - Mostra APENAS o bounding box do objeto alvo (_objeto_alvo)
        - Detecta e desenha anomalias apenas dentro desse objeto (apenas se _anomaly_detection_enabled=True)
        
        Args:
            image (numpy.ndarray): Imagem OpenCV no formato BGR (numpy array)
            
        Returns:
            tuple: (imagem_anotada, lista_deteccoes, timing)
                - imagem_anotada: Imagem com bounding boxes e labels desenhados
                - lista_deteccoes: Lista de dicionários com informações das detecções
                - timing: dict com tempos de inferência em ms ('obj_ms', 'anom_ms',
                  'anomaly_active') usados pelo relatório da missão para calcular a FR.
        """
        # Tempos de inferência (ms) deste frame, para o relatório de FR.
        timing = {"obj_ms": None, "anom_ms": None,
                  "anomaly_active": bool(self._anomaly_detection_enabled)}

        if self.yolo_model_objects is None:
            return image, [], timing

        try:
            annotated_image = image.copy()
            detections = []
            anom_ms_total = 0.0   # soma do tempo de inferência de anomalias neste frame
            anom_ran = False      # True se o modelo de anomalias rodou ao menos uma vez

            # ==================== ESTÁGIO 1: Detecção de objetos da plataforma ====================
            target_class_id = None
            
            # Define se deve filtrar apenas o objeto alvo com base nos estados da FSM que requerem foco:
            # - DETECTANDO: Buscando o objeto para confirmar presença e posição
            # - ESCANEANDO: Analisando o objeto encontrado em busca de anomalias
            # - ESCANEAMENTO_FINALIZADO: Finalizando o processo e salvando dados
            should_filter_object = (
                    self._current_fsm_state == "EXECUTANDO_INSPECIONANDO_DETECTANDO" or
                    self._current_fsm_state == "EXECUTANDO_INSPECIONANDO_ESCANEANDO" or
                    self._current_fsm_state == "EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO"
                )
            if should_filter_object and self._objeto_alvo:
                # Encontra o ID da classe que corresponde ao objeto alvo
                for class_id, class_name in self.yolo_model_objects.names.items():
                    if self._objeto_alvo.lower() == class_name.lower():
                        target_class_id = class_id
                
                if target_class_id is None:
                    # Objeto alvo não existe nas classes do modelo, então não conseguirá detectar nada
                    # self.get_logger().warn(f"Objeto alvo '{self._objeto_alvo}' não encontrado nas classes do modelo.")
                    results = None
                else:
                    # Executa predição filtrando apenas pela classe do objeto alvo
                    t_obj = time.perf_counter()
                    results = self.yolo_model_objects.predict(image, verbose=False, device=0, classes=target_class_id)
                    timing["obj_ms"] = (time.perf_counter() - t_obj) * 1000.0
            else:
                # Modo geral: detecta qualquer objeto conhecido pelo modelo
                t_obj = time.perf_counter()
                results = self.yolo_model_objects.predict(image, verbose=False, device=0, classes=None)
                timing["obj_ms"] = (time.perf_counter() - t_obj) * 1000.0

            if not results:
                return image, [], timing

            result = results[0]
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                return image, [], timing

            # Converte UMA vez por frame (GPU -> CPU)
            xyxy = boxes.xyxy.cpu().numpy()              # (N, 4)
            conf = boxes.conf.cpu().numpy()              # (N,)
            cls  = boxes.cls.cpu().numpy().astype(int)   # (N,)

            for (x1, y1, x2, y2), confidence, class_id in zip(xyxy, conf, cls):
                # Filtra detecções com confiança mínima configurável
                if confidence <= self._object_min_confidence:
                    continue

                class_name = self.yolo_model_objects.names[class_id]
                x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                
                # Desenha bounding box do objeto (VERDE)
                cv2.rectangle(annotated_image, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                label = f"{class_name}: {float(confidence):.2f}"
                cv2.putText(annotated_image, label, (x1i, y1i - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calcula centro do bounding box
                bbox_center_x = (x1 + x2) / 2.0
                bbox_center_y = (y1 + y2) / 2.0

                # Cria entrada de detecção
                detection = {
                    "object_type": class_name.lower(),
                    "class": class_name,
                    "confidence": float(confidence),
                    "bbox": [x1i, y1i, x2i, y2i],
                    "bbox_center": [float(bbox_center_x), float(bbox_center_y)],
                    "anomalies": []  # Lista de anomalias detectadas neste objeto
                }
                
                # ==================== ESTÁGIO 2: Detecção de anomalias no crop ====================
                # Só executa se a detecção de anomalias estiver habilitada via service
                if self._anomaly_detection_enabled and self.yolo_model_anomalies is not None:
                    # === Copia imagens ANTES de desenhar anomalias ===
                    img_original = image.copy()  # Imagem sem nenhuma anotação
                    img_objeto = annotated_image.copy()  # Imagem com BB do objeto (verde)
                    
                    # Faz crop da região do objeto
                    crop = image[y1i:y2i, x1i:x2i]
                    
                    if crop.size > 0:  # Verifica se crop é válido
                        t_anom = time.perf_counter()
                        anomaly_results = self.yolo_model_anomalies.predict(crop, verbose=False, device=0)
                        anom_ms_total += (time.perf_counter() - t_anom) * 1000.0
                        anom_ran = True
                        
                        if anomaly_results and anomaly_results[0].boxes is not None:
                            anom_boxes = anomaly_results[0].boxes
                            if len(anom_boxes) > 0:
                                anom_xyxy = anom_boxes.xyxy.cpu().numpy()
                                anom_conf = anom_boxes.conf.cpu().numpy()
                                anom_cls = anom_boxes.cls.cpu().numpy().astype(int)
                                
                                anom_segments = None
                                if anomaly_results[0].masks is not None:
                                    anom_segments = anomaly_results[0].masks.xy

                                for idx, ((ax1, ay1, ax2, ay2), anom_confidence, anom_class_id) in enumerate(zip(anom_xyxy, anom_conf, anom_cls)):
                                    if anom_confidence <= self._anomaly_min_confidence:
                                        continue
                                    
                                    anom_class_name = self.yolo_model_anomalies.names[anom_class_id]
                                    
                                    # Converte coordenadas do crop para coordenadas da imagem original
                                    ax1_abs = int(ax1) + x1i
                                    ay1_abs = int(ay1) + y1i
                                    ax2_abs = int(ax2) + x1i
                                    ay2_abs = int(ay2) + y1i
                                    
                                    has_mask = False
                                    # Se houver máscaras de segmentação, desenha elas
                                    if anom_segments is not None and len(anom_segments) > idx:
                                        segment = anom_segments[idx]
                                        if segment.size > 0:
                                            has_mask = True
                                            # Copia para não alterar o original se for reusado (embora aqui seja novo por frame)
                                            seg_global = segment.copy()
                                            # Ajusta coordenadas (x += x1i, y += y1i)
                                            seg_global[:, 0] += x1i
                                            seg_global[:, 1] += y1i
                                            seg_global = seg_global.astype(np.int32)
                                            
                                            # Desenha contorno preenchido com transparência
                                            overlay = annotated_image.copy()
                                            cv2.fillPoly(overlay, [seg_global], (0, 0, 255))
                                            cv2.addWeighted(overlay, 0.4, annotated_image, 0.6, 0, annotated_image)
                                            # Desenha borda sólida
                                            cv2.drawContours(annotated_image, [seg_global], -1, (0, 0, 255), 2)
                                    
                                    # Se NÃO tiver máscara, desenha bounding box e texto (fallback)
                                    if not has_mask:
                                        # Desenha bounding box da anomalia (VERMELHO)
                                        cv2.rectangle(annotated_image, (ax1_abs, ay1_abs), (ax2_abs, ay2_abs), (0, 0, 255), 2)
                                        anom_label = f"{anom_class_name}: {float(anom_confidence):.2f}"
                                        cv2.putText(annotated_image, anom_label, (ax1_abs, ay1_abs - 5),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                                    
                                    # Adiciona anomalia à lista do objeto
                                    detection["anomalies"].append({
                                        "class": anom_class_name,
                                        "confidence": float(anom_confidence),
                                        "bbox": [ax1_abs, ay1_abs, ax2_abs, ay2_abs],
                                        "bbox_relative": [int(ax1), int(ay1), int(ax2), int(ay2)]
                                    })
                                
                                # === Captura 5 fotos do mesmo momento (a cada 1s) ===
                                if len(detection["anomalies"]) > 0:
                                    img_anomalias = annotated_image.copy()
                                    # Crop com anomalias desenhadas
                                    crop_anomalias = annotated_image[y1i:y2i, x1i:x2i].copy()
                                    self._capture_anomaly_photos(
                                        img_original, img_objeto, img_anomalias,
                                        crop.copy(), crop_anomalias, class_name,
                                        n_anomalias=len(detection["anomalies"])
                                    )
                
                detections.append(detection)

            # Consolida o tempo total de inferência de anomalias deste frame
            if anom_ran:
                timing["anom_ms"] = anom_ms_total

            return annotated_image, detections, timing

        except Exception as e:
            self.get_logger().error(f"Erro na detecção de objetos: {e}")
            return image, [], timing

def main(args=None):
    """Função principal do nó."""
    import signal
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    
    rclpy.init(args=args)
    cv_node = CVNode()
    
    # Usa MultiThreadedExecutor para evitar deadlock entre serviços e callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(cv_node)
    
    # Handler para SIGINT (Ctrl+C) - encerramento limpo
    def signal_handler(sig, frame):
        cv_node.get_logger().info("Encerrando cv_node...")
        # Encerramento forçado se necessário, mas o executor normalmente cuida disso
        # rclpy.shutdown() deve ser chamado no final
        pass
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass  # Ignora exceções durante shutdown
    finally:
        try:
            cv_node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()

