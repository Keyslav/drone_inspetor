"""
cv_screen.py
=================================================================================================
Tela de visualização de visão computacional.

Gerencia a exibição de imagens processadas por visão computacional. Esta classe é um
componente da interface gráfica (GUI) e não se comunica diretamente com o ROS2. Ela recebe
os dados de imagem e análise processados pelo DashboardNode através de sinais PyQt6 e os exibe.
=================================================================================================
"""

from PyQt6.QtWidgets import (QLabel, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSizePolicy)
from PyQt6.QtGui import QPixmap, QCursor
from PyQt6.QtCore import Qt
from .widgets.screens import ExpandedWindow, BaseScreen
from .widgets.images import ImageProcessor
from .widgets.model_selector import ModelSelector
from .widgets.analysis_window import AnalysisWindow
from .theme import COMMON_STYLES, IMAGE_QUALITY
from .logging import gui_log_info, gui_log_error, gui_log_warn, gui_log_debug
from .presentation.detections import DetectionFrame, format_analysis_report
from datetime import datetime
import os

class CVScreen(BaseScreen):
    """
    Gerencia a exibição de imagens processadas por visão computacional.
    
    Esta classe é um componente da interface gráfica (GUI) e não se comunica
    diretamente com o ROS2. Ela recebe os dados de imagem e análise processados
    pelo DashboardNode através de sinais PyQt6 e os exibe.
    """
    
    def __init__(self, signals, video_label):
        """
        Inicializa a tela de visão computacional.

        Args:
            signals: Objeto CVSignals PyQt6 para comunicação
            video_label (QLabel): O widget QLabel onde a imagem processada será exibida.
        """
        # Chama o construtor da classe base BaseScreen
        super().__init__(video_label, "Visão Computacional")
        
        # Armazena referência aos signals para publicação de comandos
        self.signals = signals
        self.model_selector = ModelSelector(signals)
        
        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        self.image_processor = ImageProcessor()

        # Variáveis para armazenar dados de análise e detecções
        self.analysis_logs = []
        self.current_detections = DetectionFrame(timestamp='')
        self.analysis_window = None  # Referência para a janela de logs de análise
        
        self._expanded_label = None  # Label de imagem na janela expandida
        
        # Configura a aparência inicial do display de CV
        self.setup_cv_display()
        
        # Solicita lista de modelos inicial para popular cache
        if hasattr(self.signals, 'models_requested'):
            self.signals.models_requested.emit()
        
        gui_log_info("CVScreen", "CVScreen inicializada")
    
    
    def setup_cv_display(self):
        """
        Configura o estilo visual e o comportamento do QLabel que exibe o feed de CV.
        Define cores de fundo, bordas, texto inicial e o cursor do mouse.
        """
        if self.video_label:
            self.video_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {COMMON_STYLES["dark_background"]};
                    color: {COMMON_STYLES["text_color"]};
                    border: 2px solid {COMMON_STYLES["accent_color"]};
                    border-radius: 5px;
                    font-size: 16px;
                }}
            """)
            
            self.video_label.setText("Aguardando Processamento CV...")
            self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            self.video_label.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
            
            gui_log_info("CVScreen", "Display CV configurado")
    
    def expand_screen(self, event=None):
        """
        Expande a tela em janela separada com dropdowns para seleção de modelos.
        Sobrescreve o método da classe base para adicionar controles customizados.
        
        Args:
            event: Evento de clique (opcional)
        """
        # Limpa janelas fechadas da lista
        self.expanded_windows = [w for w in self.expanded_windows if w.isVisible()]
        
        # Se já existe uma janela expandida, apenas foca nela
        if self.expanded_windows:
            existing_window = self.expanded_windows[0]
            existing_window.raise_()
            existing_window.activateWindow()
            gui_log_info("CVScreen", f"Focando janela existente de {self.screen_name}")
            return
        
        gui_log_info("CVScreen", f"Expandindo {self.screen_name} com controles de modelo")

        # Solicita atualização dos modelos ao abrir
        if hasattr(self.signals, 'models_requested'):
            self.signals.models_requested.emit()
        
        
        # Cria container principal
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # Área de controles (dropdowns de modelos)
        controls_widget = self.model_selector.create_widget()
        layout.addWidget(controls_widget)
        
        # Label para imagem expandida
        self._expanded_label = QLabel()
        self._expanded_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self._expanded_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._expanded_label.setStyleSheet(f"""
            background-color: {COMMON_STYLES["dark_background"]}; 
            color: {COMMON_STYLES["text_color"]}; 
            border: 2px solid {COMMON_STYLES["border_color"]};
            border-radius: 5px;
            font-size: 20px;
        """)
        
        # Copia conteúdo atual se disponível
        if self.video_label and self.video_label.pixmap():
            expanded_size = IMAGE_QUALITY["expanded_display_size"]
            original_pixmap = self.video_label.pixmap()
            self._expanded_label.setPixmap(original_pixmap.scaled(
                expanded_size[0], expanded_size[1], 
                Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            self._expanded_label.setText(f"Aguardando {self.screen_name}...")
        
        layout.addWidget(self._expanded_label, stretch=1)
        
        # Estilo do container
        container.setStyleSheet(f"""
            QWidget {{
                background-color: {COMMON_STYLES["dark_background"]};
            }}
        """)
        
        # Cria janela expandida
        window = ExpandedWindow(self.screen_name, container, None)
        self.expanded_windows.append(window)
        window.show()

    def update_expanded_windows(self, pixmap):
        """
        Atualiza a janela expandida customizada com novo conteúdo.
        Sobrescreve o método base para atualizar o _expanded_label.
        Implementa redimensionamento dinâmico.
        
        Args:
            pixmap: QPixmap para exibir na janela expandida
        """
        if not self.expanded_windows:
            return
            
        # Remove janelas fechadas da lista
        self.expanded_windows = [w for w in self.expanded_windows if w.isVisible()]
        
        if not self.expanded_windows:
            self._expanded_label = None
            return
        
        try:
            # Verifica se o label ainda é válido (não foi deletado pelo C++)
            if self._expanded_label and self._expanded_label.isVisible():
                # Use o tamanho atual do label para responsividade
                target_size = self._expanded_label.size()
                
                # Se o tamanho for muito pequeno (ex: inicialização), usa o tamanho original da imagem
                if target_size.width() < 10 or target_size.height() < 10:
                    scaled_pixmap = pixmap
                else:
                    scaled_pixmap = pixmap.scaled(
                        target_size, 
                        Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                
                self._expanded_label.setPixmap(scaled_pixmap)
        except RuntimeError:
            # Objeto já foi deletado
            self._expanded_label = None
        except Exception as e:
            gui_log_warn(self.screen_name, f"Erro ao atualizar janela expandida: {e}")
    
    def update_processed_image(self, cv_image):
        """
        Atualiza a exibição com uma nova imagem processada por visão computacional.
        Este método é chamado pelo DashboardNode quando uma nova mensagem de imagem
        é recebida do tópico ROS2 de CV através de sinais PyQt6.

        Args:
            cv_image (numpy.ndarray): A imagem no formato OpenCV (numpy array).
        """
        gui_log_debug("CVScreen", "Atualizando imagem processada CV")
        
        try:
            # Converte a imagem OpenCV para QImage, que pode ser exibida em um QLabel
            q_image = self.image_processor.cv_to_qimage(cv_image)
            
            if q_image:
                # Atualiza o QLabel principal e quaisquer janelas expandidas com a nova imagem
                self.update_display(q_image)
                gui_log_debug("CVScreen", "Imagem CV processada exibida com sucesso")
            else:
                gui_log_warn("CVScreen", "Falha na conversão da imagem CV")
                
        except Exception as e:
            gui_log_error("CVScreen", f"Erro no callback CV: {e}")
    
    def update_analysis_data(self, analysis_data):
        """
        Atualiza os dados de análise recebidos do nó de CV.
        Os dados são esperados em formato Dict.

        Args:
            analysis_data (dict): Dicionário contendo os dados de análise.
        """
        gui_log_info("CVScreen", "Atualizando dados de análise CV")
        
        try:
            self.analysis_logs.append(analysis_data)
            
            # Mantém apenas os últimos 50 logs para evitar consumo excessivo de memória
            if len(self.analysis_logs) > 50:
                self.analysis_logs = self.analysis_logs[-50:]
            
            # Se a janela de análise estiver aberta, atualiza seu conteúdo
            if self.analysis_window and self.analysis_window.isVisible():
                self.update_analysis_window()
                
            gui_log_info("CVScreen", f"Dados de análise atualizados. Total de logs: {len(self.analysis_logs)}")
            
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao atualizar dados de análise: {e}")
    
    def update_detections(self, detections: DetectionFrame):
        """Recebe o snapshot imutável do subscriber sem serialização interna."""
        self.current_detections = detections
        if self.analysis_window and self.analysis_window.isVisible():
            self.update_analysis_window()

    def show_analysis_logs(self):
        """
        Exibe a janela com os logs de análise em tempo real.
        Se a janela já existir, ela é trazida para o primeiro plano.
        """
        gui_log_info("CVScreen", "Abrindo janela de logs de análise")
        
        try:
            if self.analysis_window is None or not self.analysis_window.isVisible():
                self.create_analysis_window()
            else:
                self.analysis_window.raise_()
                self.analysis_window.activateWindow()
                
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao mostrar logs de análise: {e}")
    
    def create_analysis_window(self):
        """
        Cria a janela de análise com logs e estatísticas de visão computacional.
        Esta janela é uma ExpandedWindow, permitindo ser exibida separadamente.
        """
        self.analysis_window = AnalysisWindow(self.clear_analysis_logs, self.export_analysis_logs)
        self.analysis_text = self.analysis_window.analysis_text
        self.expanded_windows.append(self.analysis_window)
        
        # Atualiza o conteúdo inicial da janela de análise
        self.update_analysis_window()
        
        self.analysis_window.show()
        
        gui_log_info("CVScreen", "Janela de análise CV criada")
    
    def update_analysis_window(self):
        """
        Atualiza o conteúdo da janela de análise com os logs e detecções mais recentes.
        """
        if not self.analysis_window or not hasattr(self, "analysis_text"):
            return
        
        try:
            content = self.generate_analysis_content()
            self.analysis_text.setPlainText(content)
            
            # Rola o texto para o final para mostrar os logs mais recentes
            cursor = self.analysis_text.textCursor()
            cursor.movePosition(cursor.MoveOperation.End)
            self.analysis_text.setTextCursor(cursor)
            
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao atualizar janela de análise: {e}")
    
    def generate_analysis_content(self):
        """Gera a mesma apresentação para a janela e a exportação."""
        return format_analysis_report(
            self.current_detections, self.analysis_logs, datetime.now().strftime("%H:%M:%S")
        )

    def clear_analysis_logs(self):
        """
        Limpa todos os logs de análise e detecções armazenados.
        Atualiza a janela de análise, se estiver aberta.
        """
        gui_log_info("CVScreen", "Limpando logs de análise")
        
        self.analysis_logs.clear()
        self.current_detections = DetectionFrame(timestamp='')
        
        if self.analysis_window and hasattr(self, "analysis_text"):
            self.update_analysis_window()
        
        gui_log_info("CVScreen", "Logs de análise limpos")
    
    def export_analysis_logs(self):
        """
        Exporta os logs de análise para um arquivo de texto.
        O nome do arquivo inclui um timestamp para garantir unicidade.
        """
        gui_log_info("CVScreen", "Exportando logs de análise")
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cv_analysis_logs_{timestamp}.txt"
            
            content = self.generate_analysis_content()
            
            with open(filename, "w", encoding="utf-8") as f:
                f.write(content)
            
            gui_log_info("CVScreen", f"Logs exportados para: {filename}")
            
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao exportar logs: {e}")
    
    def reset_analysis_data(self):
        """
        Reseta todos os dados de análise e detecções.
        """
        gui_log_info("CVScreen", "Resetando dados de análise CV")
        
        self.analysis_logs.clear()
        self.current_detections = DetectionFrame(timestamp='')
        
        if self.analysis_window and hasattr(self, "analysis_text"):
            self.update_analysis_window()
        
        gui_log_info("CVScreen", "Dados de análise CV resetados")
