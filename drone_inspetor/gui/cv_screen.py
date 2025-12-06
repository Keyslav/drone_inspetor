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
                             QPushButton, QTextEdit, QScrollArea)
from PyQt6.QtGui import QPixmap, QCursor, QFont
from PyQt6.QtCore import Qt
from .utils import ExpandedWindow, BaseScreen, ImageProcessor, COMMON_STYLES, gui_log_info, gui_log_error, gui_log_warn, gui_log_debug
import json
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
            signals: Objeto de sinais PyQt6 para comunicação (não utilizado diretamente aqui)
            video_label (QLabel): O widget QLabel onde a imagem processada será exibida.
        """
        # Chama o construtor da classe base BaseScreen
        super().__init__(video_label, "Visão Computacional")
        
        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        self.image_processor = ImageProcessor()

        # Variáveis para armazenar dados de análise e detecções
        self.analysis_logs = []
        self.current_detections = []
        self.analysis_window = None  # Referência para a janela de logs de análise
        
        # Configura a aparência inicial do display de CV
        self.setup_cv_display()
        
        gui_log_info("CVScreen", "CVScreen inicializada - foco em exibição de CV")
    
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
    
    def update_detections(self, detections_json):
        """
        Atualiza as detecções de objetos recebidas do nó de CV.
        As detecções são esperadas em formato JSON (string).

        Args:
            detections_json (str): String JSON contendo a lista de detecções.
        """
        gui_log_debug("CVScreen", "Atualizando detecções CV")
        
        try:
            detections = json.loads(detections_json)
            self.current_detections = detections
            
            # Se a janela de análise estiver aberta, atualiza seu conteúdo
            if self.analysis_window and self.analysis_window.isVisible():
                self.update_analysis_window()
                
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao atualizar detecções: {e}")
    
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
        analysis_widget = QWidget()
        analysis_widget.setWindowTitle("Logs de Análise CV")
        analysis_widget.setGeometry(200, 200, 800, 600)
        
        layout = QVBoxLayout()
        
        # Título da janela de análise
        title = QLabel("Análise de Visão Computacional")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setStyleSheet(f"""
            color: {COMMON_STYLES["text_color"]};
            background-color: {COMMON_STYLES["accent_color"]};
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 10px;
        """)
        layout.addWidget(title)
        
        # Layout para botões de controle (Limpar Logs, Exportar Logs)
        controls_layout = QHBoxLayout()
        
        clear_button = QPushButton("Limpar Logs")
        clear_button.clicked.connect(self.clear_analysis_logs)
        clear_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {COMMON_STYLES["error_color"]};
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #c0392b;
            }}
        """)
        controls_layout.addWidget(clear_button)
        
        export_button = QPushButton("Exportar Logs")
        export_button.clicked.connect(self.export_analysis_logs)
        export_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {COMMON_STYLES["success_color"]};
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #229954;
            }}
        """)
        controls_layout.addWidget(export_button)
        
        controls_layout.addStretch()
        layout.addLayout(controls_layout)
        
        # Área de texto para exibir os logs de análise
        self.analysis_text = QTextEdit()
        self.analysis_text.setReadOnly(True)
        self.analysis_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COMMON_STYLES["light_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid {COMMON_STYLES["border_color"]};
                border-radius: 5px;
                font-family: "Courier New", monospace;
                font-size: 12px;
            }}
        """)
        layout.addWidget(self.analysis_text)
        
        analysis_widget.setLayout(layout)
        analysis_widget.setStyleSheet(f"""
            QWidget {{
                background-color: {COMMON_STYLES["dark_background"]};
            }}
        """)
        
        # Cria a janela expandida e a adiciona à lista de janelas abertas
        self.analysis_window = ExpandedWindow("Análise CV", analysis_widget, None)
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
            cursor.movePosition(cursor.End)
            self.analysis_text.setTextCursor(cursor)
            
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao atualizar janela de análise: {e}")
    
    def generate_analysis_content(self):
        """
        Gera o conteúdo formatado para exibição na janela de análise.
        Inclui cabeçalho, detecções atuais, estatísticas e logs recentes.

        Returns:
            str: Uma string contendo o conteúdo formatado dos logs de análise.
        """
        content = []
        
        content.append("=" * 80)
        content.append("LOGS DE ANÁLISE DE VISÃO COMPUTACIONAL")
        content.append("=" * 80)
        content.append(f"Última atualização: {datetime.now().strftime("%H:%M:%S")}")
        content.append(f"Total de análises: {len(self.analysis_logs)}")
        content.append(f"Detecções atuais: {len(self.current_detections)}")
        content.append("")
        
        if self.current_detections:
            content.append("DETECÇÕES ATUAIS:")
            content.append("-" * 40)
            for i, detection in enumerate(self.current_detections):
                obj_type = detection.get("object_type", detection.get("label", "unknown"))
                confidence = detection.get("confidence", 0.0)
                content.append(f"{i+1}. {obj_type} (confiança: {confidence:.2f})")
            content.append("")
        
        if self.analysis_logs:
            content.append("ESTATÍSTICAS:")
            content.append("-" * 40)
            
            avg_quality = sum(log.get("quality_score", 0) for log in self.analysis_logs) / len(self.analysis_logs)
            content.append(f"Qualidade média: {avg_quality:.1f}/100")
            
            avg_sharpness = sum(log.get("sharpness_score", 0) for log in self.analysis_logs) / len(self.analysis_logs)
            content.append(f"Nitidez média: {avg_sharpness:.1f}")
            
            content.append("")
        
        content.append("LOGS RECENTES (últimos 10):")
        content.append("-" * 40)
        
        recent_logs = self.analysis_logs[-10:] if len(self.analysis_logs) >= 10 else self.analysis_logs
        
        for log in reversed(recent_logs):  # Itera de trás para frente para mostrar os mais recentes primeiro
            timestamp = log.get("timestamp", "N/A")
            quality = log.get("quality_score", 0)
            sharpness = log.get("sharpness_score", 0)
            detections = log.get("detections", [])
            
            content.append(f"[{timestamp}] Qualidade: {quality:.1f} | Nitidez: {sharpness:.1f} | Detecções: {len(detections)}")
            
            if detections:
                for detection in detections:
                    obj_type = detection.get("object_type", detection.get("label", "unknown"))
                    confidence = detection.get("confidence", 0.0)
                    content.append(f"    → {obj_type} ({confidence:.2f})")
            
            content.append("")
        
        return "\n".join(content)
    
    def clear_analysis_logs(self):
        """
        Limpa todos os logs de análise e detecções armazenados.
        Atualiza a janela de análise, se estiver aberta.
        """
        gui_log_info("CVScreen", "Limpando logs de análise")
        
        self.analysis_logs.clear()
        self.current_detections.clear()
        
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
        self.current_detections.clear()
        
        if self.analysis_window and hasattr(self, "analysis_text"):
            self.update_analysis_window()
        
        gui_log_info("CVScreen", "Dados de análise CV resetados")
