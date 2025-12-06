"""
camera_screen.py
=================================================================================================
Tela de visualização da câmera principal (raw/não processada).

Gerencia a exibição do feed de vídeo da câmera principal. Esta classe é um componente
da interface gráfica (GUI) e não se comunica diretamente com o ROS2. Ela recebe os dados
de imagem processados pelo DashboardNode através de sinais PyQt6 e os exibe.
=================================================================================================
"""

from PyQt6.QtWidgets import QLabel, QWidget, QVBoxLayout
from PyQt6.QtGui import QPixmap, QCursor
from PyQt6.QtCore import Qt
from .utils import ExpandedWindow, BaseScreen, ImageProcessor, COMMON_STYLES, gui_log_info, gui_log_error

class CameraScreen(BaseScreen):
    """
    Gerencia a exibição do feed de vídeo da câmera principal (raw/não processada).
    
    Esta classe é um componente da interface gráfica (GUI) e não se comunica
    diretamente com o ROS2. Ela recebe os dados de imagem processados pelo
    DashboardNode através de sinais PyQt6 e os exibe.
    """
    
    def __init__(self, signals, video_label):
        """
        Inicializa a tela da câmera principal.

        Args:
            signals: Objeto de sinais PyQt6 para comunicação (não utilizado diretamente aqui)
            video_label (QLabel): O widget QLabel onde o feed de vídeo será exibido.
        """
        # Chama o construtor da classe base BaseScreen
        super().__init__(video_label, "Câmera Principal")
        
        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        self.image_processor = ImageProcessor()
        
        # Configura a aparência inicial do display da câmera
        self.setup_camera_display()
        gui_log_info("CameraScreen", "CameraScreen inicializada - foco em exibição de câmera raw")
    
    def setup_camera_display(self):
        """
        Configura o estilo visual e o comportamento do QLabel que exibe o feed da câmera.
        Define cores de fundo, bordas, texto inicial e o cursor do mouse.
        """
        if self.video_label:
            self.video_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {COMMON_STYLES["dark_background"]};
                    color: {COMMON_STYLES["text_color"]};
                    border: 2px solid {COMMON_STYLES["border_color"]};
                    border-radius: 5px;
                    font-size: 16px;
                }}
            """)
            self.video_label.setText("Aguardando Câmera Principal...")
            self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.video_label.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
            gui_log_info("CameraScreen", "Display da câmera configurado")
    
    def update_camera_feed(self, cv_image):
        """
        Atualiza o feed da câmera com uma nova imagem.
        Este método é chamado pelo DashboardNode quando uma nova mensagem de imagem
        é recebida do tópico ROS2 da câmera através de sinais PyQt6.

        Args:
            cv_image (numpy.ndarray): A imagem no formato OpenCV (numpy array).
        """
        try:
            # Converte a imagem OpenCV para QImage, que pode ser exibida em um QLabel
            q_image = self.image_processor.cv_to_qimage(cv_image)

            if q_image:
                # Atualiza o QLabel principal e quaisquer janelas expandidas com a nova imagem
                self.update_display(q_image)
            else:
                gui_log_error("CameraScreen", "Falha na conversão da imagem da câmera")
                
        except Exception as e:
            gui_log_error("CameraScreen", f"Erro no callback da câmera: {e}")

    def expand_screen(self, event=None):
        """
        Sobrescreve o método `expand_screen` da classe base `BaseScreen`.
        Cria e exibe uma janela expandida dedicada para a visualização da câmera principal.

        Args:
            event (QMouseEvent, optional): O evento de clique do mouse que disparou a expansão.
                                           Pode ser None se chamado programaticamente.
        """
        gui_log_info("CameraScreen", "Expandindo visualização da câmera principal")
        
        # Cria um novo QLabel para a janela expandida e aplica estilos
        expanded_label = QLabel()
        expanded_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        expanded_label.setStyleSheet(f"""
            QLabel {{
                background-color: {COMMON_STYLES["dark_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid {COMMON_STYLES["accent_color"]};
                border-radius: 5px;
                font-size: 20px;
            }}
        """)
        
        # Copia o conteúdo atual do QLabel principal para a janela expandida, se houver
        if self.video_label and self.video_label.pixmap():
            original_pixmap = self.video_label.pixmap()
            # Redimensiona a imagem para a janela expandida, mantendo a proporção
            expanded_label.setPixmap(original_pixmap.scaled(
                1800, 1000, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            expanded_label.setText("Aguardando Câmera Principal...")
        
        # Cria e exibe a janela expandida usando a classe ExpandedWindow
        window = ExpandedWindow("Câmera Principal", expanded_label, None)
        self.expanded_windows.append(window)  # Mantém uma referência para a janela expandida
        window.show()
        
        gui_log_info("CameraScreen", "Janela expandida da câmera criada")
    
    def get_current_frame(self):
        """
        Retorna o frame atual da câmera como um QPixmap.
        Útil para outras partes do sistema que possam precisar acessar a imagem atual.

        Returns:
            QPixmap: O frame atual da câmera, ou None se não houver imagem disponível.
        """
        if self.video_label and self.video_label.pixmap():
            return self.video_label.pixmap()
        return None
    
    def set_camera_status(self, status_text, is_error=False):
        """
        Define o texto de status exibido na tela da câmera.
        Pode ser usado para mostrar mensagens como "Conectando...", "Erro de conexão", etc.

        Args:
            status_text (str): O texto a ser exibido.
            is_error (bool, optional): Se True, o texto será exibido com uma cor de erro.
                                       Padrão para False.
        """
        if self.video_label:
            color = COMMON_STYLES["error_color"] if is_error else COMMON_STYLES["text_color"]
            
            self.video_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {COMMON_STYLES["dark_background"]};
                    color: {color};
                    border: 2px solid {COMMON_STYLES["border_color"]};
                    border-radius: 5px;
                    font-size: 16px;
                }}
            """)
            
            self.video_label.setText(status_text)
            gui_log_info("CameraScreen", f"Status da câmera atualizado: {status_text}")

def create_camera_widget():
    """
    Função utilitária para criar um widget de câmera independente.
    Pode ser útil para testar a CameraScreen isoladamente ou para integrar
    em outras partes da aplicação que não sejam o Dashboard principal.

    Returns:
        tuple: Uma tupla contendo (QWidget, CameraScreen) - o widget container
               e a instância da CameraScreen.
    """
    widget = QWidget()
    layout = QVBoxLayout()
    
    video_label = QLabel()
    layout.addWidget(video_label)
    widget.setLayout(layout)
    
    camera_screen = CameraScreen(signals=None, video_label=video_label)
    
    return widget, camera_screen
