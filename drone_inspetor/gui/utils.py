"""
utils.py
=================================================================================================
Módulo de utilitários compartilhados para o dashboard ROS2 do drone.

Contém classes e funções auxiliares utilizadas por todos os módulos de tela, incluindo:
- Classes de janelas auxiliares (ExpandedWindow, BaseScreen)
- Processamento de imagens (ImageProcessor)
- Funções matemáticas e de formatação
- Sistema de logging padronizado para GUI
=================================================================================================
"""

# ==================== IMPORTAÇÕES ====================
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt6.QtGui import QPixmap, QImage
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
from datetime import datetime
import sys

# ==================== SISTEMA DE LOGGING PARA GUI ====================

def gui_log_info(module_name, message):
    """
    Registra mensagem de informação do módulo de GUI.
    
    Args:
        module_name (str): Nome do módulo de origem (ex: "CVScreen", "CameraScreen")
        message (str): Mensagem a ser registrada
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [INFO] [{module_name}] {message}", file=sys.stdout)

def gui_log_warn(module_name, message):
    """
    Registra mensagem de aviso do módulo de GUI.
    
    Args:
        module_name (str): Nome do módulo de origem (ex: "CVScreen", "CameraScreen")
        message (str): Mensagem a ser registrada
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [WARN] [{module_name}] {message}", file=sys.stderr)

def gui_log_error(module_name, message):
    """
    Registra mensagem de erro do módulo de GUI.
    
    Args:
        module_name (str): Nome do módulo de origem (ex: "CVScreen", "CameraScreen")
        message (str): Mensagem a ser registrada
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [ERROR] [{module_name}] {message}", file=sys.stderr)

def gui_log_debug(module_name, message):
    """
    Registra mensagem de debug do módulo de GUI.
    
    Args:
        module_name (str): Nome do módulo de origem (ex: "CVScreen", "CameraScreen")
        message (str): Mensagem a ser registrada
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    # print(f"[{timestamp}] [DEBUG] [{module_name}] {message}", file=sys.stdout)

    
# ==================== CLASSES DE JANELAS AUXILIARES ====================

class ExpandedWindow(QMainWindow):
    """
    Janela independente para exibir conteúdo ampliado em tela cheia.
    
    Características:
    - Janela independente (parent=None)
    - Abre maximizada automaticamente
    - Fecha com duplo clique
    - Tema escuro consistente
    
    Utilizada por todas as telas para mostrar versões expandidas.
    """

    def __init__(self, title, content_widget, parent=None):
        """
        Inicializa janela expandida.

        Args:
            title: Título da janela (ex: "Câmera Principal - AMPLIADO")
            content_widget: Widget a ser exibido ampliado
            parent: Widget pai (None para janela independente)
        """
        super().__init__(parent)
        self.setWindowTitle(f"{title} - AMPLIADO")
        self.setGeometry(0, 0, 1920, 1080)
        self.showMaximized()

        # Estilização com tema escuro
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2c3e50;  /* Fundo azul escuro */
                color: #ecf0f1;             /* Texto claro */
            }
        """)

        # Define o widget de conteúdo como central
        self.setCentralWidget(content_widget)

    def mouseDoubleClickEvent(self, event):
        """
        Fecha a janela com duplo clique do mouse.

        Args:
            event: Evento de duplo clique
        """
        self.close()

# ==================== CLASSES DE INTERFACE COMUM (CORRIGIDA) ====================

class BaseScreen:
    """
    Classe base para todas as telas do dashboard.
    
    Fornece funcionalidades comuns como:
    - Gerenciamento de janelas expandidas
    - Métodos de atualização de display
    - Sistema de logging padronizado
    """
    
    def __init__(self, video_label, screen_name):
        """
        Inicializa tela base.
        
        Args:
            video_label: QLabel para exibição principal
            screen_name: Nome da tela para logging
        """
        self.video_label = video_label
        self.screen_name = screen_name
        self.expanded_windows = []  # Lista de janelas expandidas
        
        # ========== CORREÇÕES PARA ESTABILIZAR REDIMENSIONAMENTO ==========
        self._cached_size = None  # Cache das dimensões do widget
        self._target_size = IMAGE_QUALITY["main_display_size"]  # Tamanho padrão fixo
        self._size_threshold = 5  # Threshold menor para permitir mais atualizações
        
        # Configura cursor clicável se video_label fornecido
        if self.video_label:
            self.video_label.setCursor(Qt.CursorShape.PointingHandCursor)
            self.video_label.mousePressEvent = self.expand_screen
            # Define tamanho mínimo para evitar redimensionamentos extremos
            self.video_label.setMinimumSize(320, 240)
            # Política de tamanho mais flexível
            if hasattr(self.video_label, 'setSizePolicy'):
                from PyQt6.QtWidgets import QSizePolicy
                self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
    
    # Método removido - era muito restritivo
    def expand_screen(self, event=None):
        """
        Expande a tela em janela separada.
        
        Args:
            event: Evento de clique (opcional)
        """
        gui_log_info(self.screen_name, f"Expandindo {self.screen_name}")
        
        # Cria widget expandido
        expanded_label = QLabel()
        expanded_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        expanded_label.setStyleSheet("""
            background-color: #1a252f; 
            color: #ecf0f1; 
            border: 2px solid #7f8c8d;
            border-radius: 5px;
            font-size: 20px;
        """)
        
        # Copia conteúdo atual se disponível
        if self.video_label and self.video_label.pixmap():
            # Usa tamanho fixo para janelas expandidas (sem redimensionamento dinâmico)
            expanded_size = IMAGE_QUALITY["expanded_display_size"]
            original_pixmap = self.video_label.pixmap()
            expanded_label.setPixmap(original_pixmap.scaled(
                expanded_size[0], expanded_size[1], 
                Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            expanded_label.setText(f"Aguardando {self.screen_name}...")
        
        # Cria e exibe janela expandida
        window = ExpandedWindow(self.screen_name, expanded_label, None)
        self.expanded_windows.append(window)
        window.show()
    
    def update_display(self, q_image):
        """
        Atualiza o display principal com nova imagem (VERSÃO BALANCEADA).
        
        Args:
            q_image: QImage para exibir
        """
        if not q_image or not self.video_label:
            return
            
        try:
            # Pega dimensões atuais do widget
            current_width = self.video_label.width()
            current_height = self.video_label.height()
            
            # Usa dimensões cached apenas se disponíveis e estáveis
            if (self._cached_size and 
                abs(current_width - self._cached_size[0]) <= self._size_threshold and
                abs(current_height - self._cached_size[1]) <= self._size_threshold):
                # Usa cache se as dimensões não mudaram muito
                target_width, target_height = self._cached_size
            else:
                # Atualiza cache e usa dimensões atuais
                if current_width > 100 and current_height > 100:
                    self._cached_size = (current_width, current_height)
                    target_width, target_height = current_width, current_height
                else:
                    # Fallback para tamanho padrão se dimensões são inválidas
                    target_width, target_height = self._target_size
            
            # Cria pixmap redimensionado
            pixmap = QPixmap.fromImage(q_image).scaled(
                target_width, target_height,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            # SEMPRE atualiza a imagem (removido o bloqueio de pixmap igual)
            self.video_label.setPixmap(pixmap)
            
            # Atualiza janelas expandidas também
            self.update_expanded_windows(pixmap)
            
        except Exception as e:
            gui_log_error(self.screen_name, f"Erro ao atualizar display: {e}")
    
    # Método removido - estava bloqueando atualizações
    def update_expanded_windows(self, pixmap):
        """
        Atualiza todas as janelas expandidas com novo conteúdo (VERSÃO OTIMIZADA).
        
        Args:
            pixmap: QPixmap para exibir nas janelas expandidas
        """
        if not self.expanded_windows:
            return
            
        # Tamanho fixo para janelas expandidas (evita recálculos)
        expanded_size = IMAGE_QUALITY["expanded_display_size"]
        
        # Remove janelas fechadas da lista
        self.expanded_windows = [w for w in self.expanded_windows if w.isVisible()]
        
        for window in self.expanded_windows:
            try:
                central_widget = window.centralWidget()
                if isinstance(central_widget, QLabel):
                    # Redimensiona apenas uma vez por frame
                    scaled_pixmap = pixmap.scaled(
                        expanded_size[0], expanded_size[1], 
                        Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                    central_widget.setPixmap(scaled_pixmap)
            except Exception as e:
                gui_log_warn(self.screen_name, f"Erro ao atualizar janela expandida: {e}")

# ==================== CONFIGURAÇÕES ATUALIZADAS ====================

# Configurações de qualidade de imagem (com tamanhos mais estáveis)
IMAGE_QUALITY = {
    "main_display_size": (640, 480),        # Tamanho padrão fixo para display principal
    "expanded_display_size": (1600, 1200),  # Tamanho fixo para janelas expandidas
    "thumbnail_size": (320, 240),           # Tamanho para thumbnails
    "compression_quality": 85,
    "min_widget_size": (320, 240),          # Tamanho mínimo do widget
    "size_change_threshold": 5              # Threshold menor e mais permissivo
}

# ==================== CLASSE DE PROCESSAMENTO OTIMIZADA ====================

class ImageProcessor:
    """
    Classe utilitária para processamento comum de imagens ROS2 (VERSÃO OTIMIZADA).
    """
    
    def __init__(self):
        """
        Inicializa o processador de imagens.
        """
        self.bridge = CvBridge()
        
        # Cache para otimização
        self._last_encoding = None
        self._conversion_cache = {}
    
    def _log_error(self, message):
        """Método auxiliar para logging."""
        gui_log_error("ImageProcessor", message)

    def ros_to_qimage(self, msg, encoding="bgr8"):
        """
        Converte mensagem ROS Image para QImage (VERSÃO OTIMIZADA).
        
        Args:
            msg: Mensagem sensor_msgs/Image
            encoding: Codificação desejada (bgr8, rgb8, mono8, etc.)
            
        Returns:
            QImage: Imagem convertida para Qt ou None se erro
        """
        try:
            # Verifica se os parâmetros da imagem são válidos
            if not hasattr(msg, 'width') or not hasattr(msg, 'height') or msg.width == 0 or msg.height == 0:
                self._log_error("Mensagem de imagem com dimensões inválidas")
                return None
            
            # Converte mensagem ROS para OpenCV com cache de encoding
            if encoding != self._last_encoding:
                self._last_encoding = encoding
                
            cv_image = self.bridge.imgmsg_to_cv2(msg, encoding)
            
            # Verifica se a conversão foi bem-sucedida
            if cv_image is None or cv_image.size == 0:
                self._log_error("Falha na conversão ROS->OpenCV")
                return None
            
            # Converte OpenCV para QImage
            return self.cv_to_qimage(cv_image)
            
        except Exception as e:
            self._log_error(f"Erro na conversão ROS->QImage: {e}")
            return None
    
    def cv_to_qimage(self, cv_image):
        """
        Converte imagem OpenCV para QImage (VERSÃO MAIS ROBUSTA).
        
        Args:
            cv_image: Imagem OpenCV (numpy array)
            
        Returns:
            QImage: Imagem convertida para Qt ou None se erro
        """
        try:
            if cv_image is None or cv_image.size == 0:
                return None
                
            h, w = cv_image.shape[:2]
            
            # Verifica dimensões válidas
            if h <= 0 or w <= 0:
                gui_log_error("ImageProcessor", f"Dimensões de imagem inválidas: {w}x{h}")
                return None
            
            if len(cv_image.shape) == 3:  # Imagem colorida
                ch = cv_image.shape[2]
                bytes_per_line = ch * w
                
                if ch == 3:  # BGR -> RGB
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    # Garante contiguidade da memória
                    cv_image = np.ascontiguousarray(cv_image)
                    return QImage(cv_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                
                if ch == 4:  # BGRA -> RGBA
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGBA)
                    cv_image = np.ascontiguousarray(cv_image)
                    return QImage(cv_image.data, w, h, bytes_per_line, QImage.Format.Format_RGBA8888)
            
            # Imagem em tons de cinza
            if len(cv_image.shape) == 2:
                bytes_per_line = w
                cv_image = np.ascontiguousarray(cv_image)
                return QImage(cv_image.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8)
                
        except Exception as e:
            self._log_error(f"Erro na conversão CV->QImage: {e}")
            return None
        
# ==================== FUNÇÕES MATEMÁTICAS ====================

def quaternion_to_euler(q):
    """
    Converte um quaternion para ângulos de Euler, especificamente o ângulo yaw (rotação Z).
    
    Os quaternions são uma forma matemática de representar rotações 3D sem ambiguidade,
    mas para visualização humana, ângulos de Euler são mais intuitivos.
    
    Args:
        q (list): Quaternion no formato [w, x, y, z] onde:
                 - w: Componente escalar (parte real)
                 - x, y, z: Componentes vetoriais (partes imaginárias)
    
    Returns:
        float: Ângulo yaw em graus (0-360°)
        
    Note:
        - Yaw é a rotação em torno do eixo Z (vertical)
        - Para um drone, yaw representa a direção que ele está "olhando"
        - 0° = Norte, 90° = Leste, 180° = Sul, 270° = Oeste
    """
    # Extrai os componentes individuais do quaternion
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Cálculo do yaw usando fórmulas de conversão quaternion -> Euler
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Converte de radianos para graus
    yaw_degrees = math.degrees(yaw)
    
    # Normaliza para 0-360°
    if yaw_degrees < 0:
        yaw_degrees += 360
        
    return yaw_degrees

def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Calcula a distância entre dois pontos GPS usando a fórmula de Haversine.
    
    Args:
        lat1, lon1: Latitude e longitude do primeiro ponto (em graus)
        lat2, lon2: Latitude e longitude do segundo ponto (em graus)
    
    Returns:
        float: Distância em metros
    """
    # Raio da Terra em metros
    R = 6371000
    
    # Converte graus para radianos
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Diferenças
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Fórmula de Haversine
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

# ==================== FUNÇÕES DE FORMATAÇÃO ====================

def format_timestamp():
    """
    Retorna timestamp formatado para logs.
    
    Returns:
        str: Timestamp no formato HH:MM:SS
    """
    return datetime.now().strftime("%H:%M:%S")

def format_coordinates(lat, lon, alt=None):
    """
    Formata coordenadas para exibição.
    
    Args:
        lat: Latitude em graus
        lon: Longitude em graus
        alt: Altitude em metros (opcional)
    
    Returns:
        str: Coordenadas formatadas
    """
    if alt is not None:
        return f"{lat:.6f}°, {lon:.6f}°, {alt:.1f}m"
    
    return f"{lat:.6f}°, {lon:.6f}°"

# ==================== CONSTANTES COMPARTILHADAS ====================

# Estilos CSS comuns para todas as telas
COMMON_STYLES = {
    "dark_background": "#2c3e50",
    "light_background": "#34495e",
    "text_color": "#ecf0f1",
    "border_color": "#7f8c8d",
    "accent_color": "#3498db",
    "error_color": "#e74c3c",
    "success_color": "#27ae60"
}

# Configurações de tópicos ROS2
ROS_TOPICS = {
    "camera_raw": "/camera/image_raw",
    "camera_processed": "/camera/image_processed",
    "depth_camera": "/depth_camera/image_raw",
    "lidar_image": "/lidar/image",
    "lidar_pointcloud": "/lidar/pointcloud",
    "fsm_state": "/drone_inspetor/fsm_state",
    "vehicle_position": "/fmu/out/vehicle_global_position",
    "vehicle_attitude": "/fmu/out/vehicle_attitude"
}

