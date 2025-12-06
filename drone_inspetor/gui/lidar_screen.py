"""
lidar_screen.py
=================================================================================================
Tela de visualização dos dados do sensor LiDAR.

Responsável por exibir a página HTML do dashboard LiDAR e gerenciar as interações.
Esta classe é um componente da interface gráfica (GUI) e não se comunica diretamente
com o ROS2. Ela recebe os dados processados pelo DashboardNode através de sinais PyQt6.
=================================================================================================
"""

# Importações do PyQt6 para interface gráfica
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import pyqtSignal, QUrl
from PyQt6.QtGui import QFont

# Importações do sistema para manipulação de arquivos
import os
import json

# Importações de utilitários
from .utils import gui_log_info, gui_log_error

class LidarScreen(QWidget):
    """
    Classe responsável pela tela de visualização dos dados do sensor LiDAR.
    
    Recebe dados já processados em tipos nativos do Python através de sinais PyQt6.
    """
    
    point_vector_received = pyqtSignal(object)
    statistics_received = pyqtSignal(object)
    obstacle_detections_received = pyqtSignal(object)
    
    def __init__(self, signals, original_label: QLabel):
        """
        Inicializa a tela do LiDAR.
        
        Args:
            signals: Objeto de sinais PyQt6 para comunicação (não utilizado diretamente aqui)
            original_label (QLabel): O widget QLabel onde o dashboard LiDAR será exibido.
        """
        super().__init__()
        
        self.original_label = original_label
        self.is_page_loaded = False
        self.setup_ui()
        self.load_lidar_dashboard()
    
    def setup_ui(self):
        # (Método sem alterações)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.web_view = QWebEngineView()
        self.web_view.setStyleSheet("QWebEngineView { background-color: transparent; border: 2px solid #e67e22; border-radius: 5px; }")
        layout.addWidget(self.web_view)
        self.original_label.setLayout(layout)
        self.original_label.setText("")
    
    def load_lidar_dashboard(self):
        # (Método sem alterações)
        possible_paths = [os.path.join(os.path.dirname(__file__), 'lidar_dashboard.html'), os.path.join(os.path.dirname(__file__), '..', 'lidar_dashboard.html')]
        html_path = next((path for path in possible_paths if os.path.exists(path)), None)
        if html_path:
            self.web_view.loadFinished.connect(self.on_load_finished)
            self.web_view.load(QUrl.fromLocalFile(os.path.abspath(html_path)))
        else:
            self.web_view.setHtml("<h1>Erro: lidar_dashboard.html não encontrado.</h1>")
            gui_log_error("LidarScreen", "Arquivo lidar_dashboard.html não encontrado")

    def on_load_finished(self, success: bool):
        # (Método sem alterações)
        if not success:
            gui_log_error("LidarScreen", "Falha ao carregar a página do dashboard LiDAR")
            return
        self.is_page_loaded = True
        self.web_view.page().runJavaScript("startRadarInitialization();")

    def update_point_vector(self, point_vector: list):
        """
        Recebe o vetor de pontos JÁ COMO UMA LISTA PYTHON e o envia para o JavaScript.

        Args:
            point_vector (list[float]): A lista de floats [dist1, ang1, dist2, ang2, ...].
        """
        if not self.is_page_loaded:
            return

        try:
            # 1. A conversão já foi feita no subscriber. Agora só precisamos serializar.
            # Serializa a lista Python para uma string no formato JSON.
            json_vector = json.dumps(point_vector)

            # 2. Constrói e executa o comando JavaScript.
            js_command = f"dashboard_update_point_vector({json_vector});"

            # 3. Executa o comando JavaScript na página carregada.
            self.web_view.page().runJavaScript(js_command)

        except Exception as e:
            gui_log_error("LidarScreen", f"Erro ao enviar vetor de pontos para o JavaScript: {e}")

    def update_lidar_statistics(self, statistics: dict):
        # (Placeholder)
        pass
    
    def update_obstacle_detections(self, detections: dict):
        # (Placeholder)
        pass
