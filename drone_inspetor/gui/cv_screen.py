"""
cv_screen.py
=================================================================================================
Tela de visualização de visão computacional.

Gerencia a exibição de imagens processadas por visão computacional. Esta classe é um
componente da interface gráfica (GUI) e não se comunica diretamente com o ROS2. Ela recebe
os dados de imagem e análise processados pelo DashboardNode através de sinais PyQt6 e os exibe.
=================================================================================================
"""

from PyQt6.QtWidgets import QMenu
from PyQt6.QtGui import QCursor, QShortcut, QKeySequence
from PyQt6.QtCore import Qt
from .widgets.screens import ExpandedWindow, BaseScreen
from .widgets.images import ImageProcessor, ResponsiveImageLabel
from .widgets.model_selector import ModelSelector
from .widgets.analysis_window import AnalysisWindow
from .theme import COMMON_STYLES
from .logging import gui_log_info, gui_log_error, gui_log_warn, gui_log_debug
from .presentation.detections import DetectionFrame, format_analysis_report
from datetime import datetime

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
        self.model_selector = ModelSelector(signals, video_label)
        self._video_window = None
        
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
    
    def show_model_selector(self, parent=None):
        """Catálogo em janela própria; o vídeo ampliado continua visível."""
        self.model_selector.open(parent or self.video_label.window())

    def expand_screen(self, event=None):
        """A ampliação dedica toda a área à imagem original, sem formulários."""
        if self._video_window is not None and self._video_window.isVisible():
            self._video_window.raise_()
            self._video_window.activateWindow()
            return
        self._expanded_label = ResponsiveImageLabel(f'Aguardando {self.screen_name}…')
        self._expanded_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._expanded_label.setStyleSheet('background: #090f1a; color: #93a4bb;')
        if self._last_pixmap is not None:
            self._expanded_label.set_source_pixmap(self._last_pixmap)
        window = ExpandedWindow(self.screen_name, self._expanded_label)
        self._video_window = window
        self.expanded_windows.append(window)
        # Atalho e menu contextual permitem trocar a rede sem abandonar o vídeo.
        shortcut = QShortcut(QKeySequence('Ctrl+R'), window)
        shortcut.activated.connect(lambda: self.show_model_selector(window))
        self._expanded_label.setToolTip('Botão direito ou Ctrl+R: selecionar redes CV')
        self._expanded_label.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        def menu(position):
            popup = QMenu(window)
            popup.addAction('Selecionar redes CV…', lambda: self.show_model_selector(window))
            popup.exec(self._expanded_label.mapToGlobal(position))
        self._expanded_label.customContextMenuRequested.connect(menu)
        window.show()

    def update_expanded_windows(self, pixmap):
        """A janela de análise não disputa a referência da janela de vídeo."""
        if self._video_window is not None and self._video_window.isVisible():
            self._expanded_label.set_source_pixmap(pixmap)

    def close(self):
        self.model_selector.close()
        super().close()

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
