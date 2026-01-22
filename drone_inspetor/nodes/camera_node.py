"""
camera_node.py
=================================================================================================
Nó ROS2 responsável por gerenciar a comunicação com a câmera principal.

Este nó atua como intermediário entre o tópico externo da câmera (proveniente do simulador
ou driver de hardware) e o tópico interno padronizado do sistema. Suas funções principais são:
1. Republicar imagens em formato padronizado para o dashboard
2. Capturar fotos em estados específicos (DETECTANDO, CENTRALIZANDO)
3. Gravar vídeo durante toda a missão

PARÂMETROS ROS2 (definidos em params.yaml):
- photo_format: Formato das imagens (jpg, png)
- photo_quality: Qualidade de compressão JPEG (1-100)
- video_enabled: Habilita gravação de vídeo
- video_fps: FPS do vídeo
- video_codec: Codec de vídeo (MJPG recomendado para RPi, XVID alternativa)

ARQUITETURA:
- Assina tópico externo: /drone_inspetor/externo/camera/compressed
- Publica tópico interno: /drone_inspetor/interno/camera_node/compressed
- Assina FSMStateMSG: /drone_inspetor/interno/fsm_node/fsm_state
=================================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

# Importação das mensagens ROS customizadas
from drone_inspetor_msgs.msg import FSMStateMSG


class CameraNode(Node):
    """
    Nó ROS2 para gerenciamento da câmera principal.
    
    Responsabilidades:
    - Republicar imagens em tópico interno padronizado
    - Capturar fotos em estados de detecção
    - Gravar vídeo durante toda a missão
    """
    
    def __init__(self):
        """Inicializa o nó da câmera principal."""
        super().__init__("camera_node")
        self.get_logger().info("Nó CameraNode iniciado.")
        
        # ==================== DECLARAÇÃO DE PARÂMETROS ROS2 =============================================
        # Parâmetros de foto
        self.declare_parameter("photo_format", "jpg")
        self.declare_parameter("photo_quality", 95)
        
        # Parâmetros de vídeo
        self.declare_parameter("video_enabled", True)
        self.declare_parameter("video_fps", 15)
        self.declare_parameter("video_codec", "MJPG")
        
        # Obtém valores dos parâmetros
        self._photo_format = self.get_parameter("photo_format").get_parameter_value().string_value
        self._photo_quality = self.get_parameter("photo_quality").get_parameter_value().integer_value
        self._video_enabled = self.get_parameter("video_enabled").get_parameter_value().bool_value
        self._video_fps = self.get_parameter("video_fps").get_parameter_value().integer_value
        self._video_codec = self.get_parameter("video_codec").get_parameter_value().string_value
        
        self.get_logger().info(f"Foto: formato={self._photo_format}, qualidade={self._photo_quality}")
        self.get_logger().info(f"Vídeo: enabled={self._video_enabled}, fps={self._video_fps}, codec={self._video_codec}")
        
        # ==================== INICIALIZAÇÃO =============================================================
        self.bridge = CvBridge()
        
        # Estado da FSM
        self._current_fsm_state = ""
        self._on_mission = False
        self._current_mission_name = ""
        self._last_image = None
        self._photo_counter = 0
        
        # Informações do ponto de inspeção atual
        self._ponto_indice_atual = 0
        self._objeto_alvo = ""
        
        # Pastas da missão
        self._mission_folder_path = ""
        self._photos_folder = ""
        self._videos_folder = ""
        
        # Controle de gravação de vídeo
        self._video_writer = None
        self._video_path = ""
        self._frame_size = None  # (width, height) - detectado do primeiro frame
        self._video_recording = False  # Flag para controlar início/fim de gravação
        
        # ==================== CONFIGURAÇÃO DE QoS ========================
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ==================== SUBSCRIBERS =========================
        self.camera_raw_sub = self.create_subscription(
            CompressedImage,
            "/drone_inspetor/externo/camera/compressed",
            self.camera_raw_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado: {self.camera_raw_sub.topic_name}")
        
        self.fsm_state_sub = self.create_subscription(
            FSMStateMSG,
            "/drone_inspetor/interno/fsm_node/fsm_state",
            self.fsm_state_callback,
            qos_status
        )
        self.get_logger().info("Assinado: FSM State")

        # ==================== PUBLISHERS ====================
        self.dashboard_camera_image_pub = self.create_publisher(
            CompressedImage,
            "/drone_inspetor/interno/camera_node/compressed",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando: {self.dashboard_camera_image_pub.topic_name}")
        
        # Publisher para status de gravação (RELIABLE para garantir entrega)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.recording_status_pub = self.create_publisher(
            Bool,
            "/drone_inspetor/interno/camera_node/recording",
            qos_reliable
        )
        self.get_logger().info(f"Publicando: {self.recording_status_pub.topic_name}")

    # ==================== MÉTODOS DE VÍDEO ====================
    
    def _start_video_recording(self):
        """Inicia a gravação de vídeo para a missão atual."""
        if not self._video_enabled:
            return
        
        if not self._videos_folder:
            self.get_logger().warn("Pasta de vídeos não definida. Gravação não iniciada.")
            return
        
        # Gera nome do arquivo de vídeo
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        extension = "avi" if self._video_codec == "MJPG" else "avi"
        self._video_path = os.path.join(self._videos_folder, f"mission_{timestamp}.{extension}")
        
        # VideoWriter será criado quando recebermos o primeiro frame (para saber o tamanho)
        self._frame_size = None
        self._video_writer = None
        
        self.get_logger().info(f"🎬 Gravação de vídeo preparada: {os.path.basename(self._video_path)}")
    
    def _write_video_frame(self, cv_image):
        """
        Escreve um frame no vídeo.
        
        Args:
            cv_image: Imagem OpenCV (numpy array BGR)
        """
        if not self._video_enabled or not self._on_mission:
            return
        
        if not self._videos_folder or not self._video_path:
            return
        
        # Inicializa VideoWriter no primeiro frame
        if self._video_writer is None:
            height, width = cv_image.shape[:2]
            self._frame_size = (width, height)
            
            fourcc = cv2.VideoWriter_fourcc(*self._video_codec)
            self._video_writer = cv2.VideoWriter(
                self._video_path,
                fourcc,
                float(self._video_fps),
                self._frame_size
            )
            
            if not self._video_writer.isOpened():
                self.get_logger().error(f"Erro ao abrir VideoWriter: {self._video_path}")
                self._video_writer = None
                return
            
            self.get_logger().info(f"🎬 Gravação iniciada: {self._frame_size[0]}x{self._frame_size[1]} @ {self._video_fps}fps")
        
        # Escreve o frame
        self._video_writer.write(cv_image)
    
    def _stop_video_recording(self):
        """Para a gravação de vídeo e salva o arquivo."""
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
            self.get_logger().info(f"🎬 Vídeo salvo: {os.path.basename(self._video_path)}")
        
        self._video_path = ""
        self._frame_size = None
    
    def _publish_recording_status(self, is_recording: bool):
        """
        Publica o status de gravação no tópico.
        
        Args:
            is_recording (bool): True se está gravando, False caso contrário.
        """
        msg = Bool()
        msg.data = is_recording
        self.recording_status_pub.publish(msg)
        self.get_logger().info(f"📹 Status de gravação publicado: {'GRAVANDO' if is_recording else 'PARADO'}")

    # ==================== MÉTODOS DE FOTO ====================
    
    def _get_photo_path(self, prefix: str, ponto_indice: int, objeto_alvo: str) -> str:
        """
        Gera o caminho completo para salvar uma foto.
        
        Args:
            prefix: Tipo de captura (detectando, centralizando)
            ponto_indice: Índice do ponto de inspeção (0-based)
            objeto_alvo: Nome do objeto alvo
        """
        self._photo_counter += 1
        timestamp = datetime.now().strftime("%H%M%S")
        
        # Formato: 001_P01_detectando_flare_093012.jpg
        ponto_str = f"P{ponto_indice + 1:02d}"  # P01, P02, etc. (1-based para exibição)
        objeto_str = f"_{objeto_alvo}" if objeto_alvo else ""
        filename = f"{self._photo_counter:03d}_{ponto_str}_{prefix}{objeto_str}_{timestamp}.{self._photo_format}"
        
        return os.path.join(self._photos_folder, filename)
    
    def _capture_photo(self, state_name: str):
        """Captura e salva uma foto na subpasta 'fotos' da missão."""
        if self._last_image is None:
            self.get_logger().warn("Nenhuma imagem disponível para captura.")
            return
        
        if not self._photos_folder or not os.path.exists(self._photos_folder):
            self.get_logger().warn("Pasta de fotos não existe. Foto não salva.")
            return
        
        try:
            # Converte CompressedImage para OpenCV
            np_arr = np.frombuffer(self._last_image.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error("Falha ao decodificar imagem.")
                return
            
            # Gera caminho para a foto
            prefix = state_name.lower().replace("executando_inspecionando_", "")
            photo_path = self._get_photo_path(prefix, self._ponto_indice_atual, self._objeto_alvo)
            
            # Configura parâmetros de compressão
            if self._photo_format.lower() == "jpg":
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._photo_quality]
            elif self._photo_format.lower() == "png":
                png_compression = max(0, min(9, 9 - int(self._photo_quality / 11)))
                encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression]
            else:
                encode_params = []
            
            # Salva a imagem
            cv2.imwrite(photo_path, cv_image, encode_params)
            self.get_logger().info(f"📷 Foto capturada: {os.path.basename(photo_path)}")
            
        except Exception as e:
            self.get_logger().error(f"Erro ao capturar foto: {e}")

    # ==================== CALLBACKS ====================

    def camera_raw_callback(self, msg: CompressedImage):
        """Callback para imagens da câmera. Republica e grava vídeo se em missão."""
        self._last_image = msg
        self.dashboard_camera_image_pub.publish(msg)
        
        # Grava frame no vídeo se estiver em missão
        if self._on_mission and self._video_enabled:
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    self._write_video_frame(cv_image)
            except Exception as e:
                self.get_logger().error(f"Erro ao processar frame para vídeo: {e}", throttle_duration_sec=5)
    
    def fsm_state_callback(self, msg: FSMStateMSG):
        """
        Callback para estado da FSM.
        Controla início/fim de gravação de vídeo e captura de fotos.
        """
        new_state = msg.state_name
        new_on_mission = msg.on_mission
        new_mission_folder = msg.mission_folder_path
        
        # ========== INÍCIO DE MISSÃO (configura pastas) ==========
        if new_on_mission and not self._on_mission:
            self._current_mission_name = msg.mission_name
            self._mission_folder_path = new_mission_folder
            self._photos_folder = os.path.join(new_mission_folder, "fotos") if new_mission_folder else ""
            self._videos_folder = os.path.join(new_mission_folder, "videos") if new_mission_folder else ""
            self._photo_counter = 0
            self._video_recording = False
            
            self.get_logger().info(f"Missão iniciada: {msg.mission_name}")
        
        # ========== INÍCIO DA GRAVAÇÃO (quando começa a decolar) ==========
        if new_on_mission and new_state == "EXECUTANDO_DECOLANDO" and not self._video_recording:
            self._start_video_recording()
            self._video_recording = True
            # Publica status de gravação
            self._publish_recording_status(True)
        
        # ========== ATUALIZAÇÃO DE PASTA ==========
        if new_on_mission and new_mission_folder and new_mission_folder != self._mission_folder_path:
            self._mission_folder_path = new_mission_folder
            self._photos_folder = os.path.join(new_mission_folder, "fotos")
            self._videos_folder = os.path.join(new_mission_folder, "videos")
            self.get_logger().info(f"Pasta da missão atualizada: {new_mission_folder}")
        
        # ========== ATUALIZAÇÃO DE INFORMAÇÕES DO PONTO ==========
        if new_on_mission:
            self._ponto_indice_atual = msg.ponto_de_inspecao_indice_atual
            self._objeto_alvo = msg.objeto_alvo
        
        # ========== PARAR GRAVAÇÃO (DESATIVADO ou PRONTO) ==========
        # Para gravação APENAS quando FSM entra em estado DESATIVADO ou PRONTO
        # Isso garante que a gravação continue durante RETORNANDO (volta ao home)
        should_stop_recording = (
            self._video_recording and new_state in ["DESATIVADO", "PRONTO"]
        )
        
        if should_stop_recording:
            self._stop_video_recording()
            self._video_recording = False
            # Publica status de gravação
            self._publish_recording_status(False)
        
        # ========== FIM DE MISSÃO (limpa variáveis) ==========
        if not new_on_mission and self._on_mission:
            self.get_logger().info(f"Missão finalizada. Fotos: {self._photo_counter}")
            self._current_mission_name = ""
            self._mission_folder_path = ""
            self._photos_folder = ""
            self._videos_folder = ""
        
        # ========== CAPTURA DE FOTO ==========
        # Tira foto sempre que FSM transicionar de INSPECIONANDO para DETECTANDO
        # IMPORTANTE: verificar ANTES de atualizar _current_fsm_state
        if new_on_mission and new_state == "EXECUTANDO_INSPECIONANDO_DETECTANDO":
            if self._current_fsm_state == "EXECUTANDO_INSPECIONANDO":
                self._capture_photo(new_state)
        
        # Atualiza estado (APÓS verificação de captura de foto)
        self._on_mission = new_on_mission
        self._current_fsm_state = new_state


def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante que o vídeo seja salvo se o nó for encerrado
        if camera_node._video_writer is not None:
            camera_node._stop_video_recording()
        camera_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
