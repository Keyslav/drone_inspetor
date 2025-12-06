from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QMainWindow, QTextBrowser)
from PyQt6.QtGui import QPixmap, QImage, QCursor
from PyQt6.QtCore import Qt
from datetime import datetime
import random
import os

class PhotoWindow(QMainWindow):
    """
    Janela para exibir fotos individuais capturadas durante a inspeção.
    Esta janela é utilizada pelo sistema de logs para mostrar imagens específicas
    quando o usuário clica nos links gerados no log de análise.
    """
    
    def __init__(self, photo_name, parent=None):
        """
        Inicializa a janela de exibição de fotos.

        Args:
            photo_name (str): O nome ou título da foto a ser exibida (ex: "20241221_143052_Flare_123456.jpg").
            parent (QWidget, optional): O widget pai desta janela. Padrão para None.
        """
        super().__init__(parent)
        self.setWindowTitle(photo_name)
        self.setGeometry(200, 200, 800, 600)
        
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2c3e50;
                color: #ecf0f1;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        
        photo_label = QLabel()
        photo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        photo_label.setStyleSheet("""
            background-color: #34495e;
            border: 2px solid #7f8c8d;
            border-radius: 5px;
            color: #ecf0f1;
            font-size: 16px;
        """)
        
        photo_label.setText(f"📸 {photo_name}\n\n(Foto simulada)")
        layout.addWidget(photo_label)
        
        central_widget.setLayout(layout)
    
    def mouseDoubleClickEvent(self, event):
        """
        Fecha a janela quando um duplo clique do mouse é detectado.

        Args:
            event (QMouseEvent): O evento de duplo clique do mouse.
        """
        self.close()

class LogAnalysisWindow(QMainWindow):
    """
    Janela para exibir logs de análise da inspeção, com suporte a links clicáveis para fotos.
    Esta janela simula um sistema de log em tempo real, mostrando o progresso da inspeção,
    objetos detectados e permitindo a visualização de imagens associadas aos eventos.
    """
    
    def __init__(self, parent=None):
        """
        Inicializa a janela de logs de análise.

        Args:
            parent (QWidget, optional): O widget pai desta janela. Padrão para None.
        """
        super().__init__(parent)
        self.setWindowTitle("Drone Inspetor - Log de Análise")
        self.setGeometry(200, 200, 1000, 700)
        self.parent_dashboard = parent
        self.photo_windows = []
        self.log_content = ""
        
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2c3e50;
                color: #ecf0f1;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        
        title_label = QLabel("Log de Análise da Inspeção")
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setStyleSheet("""
            font-size: 26px;
            font-weight: bold;
            color: #ecf0f1;
            background-color: #e74c3c;
            padding: 13px;
            border-radius: 5px;
            margin-bottom: 10px;
        """)
        layout.addWidget(title_label)
        
        self.log_text = QTextBrowser()
        self.log_text.setStyleSheet("""
            QTextBrowser {
                background-color: #34495e;
                color: #ecf0f1;
                border: 2px solid #7f8c8d;
                border-radius: 5px;
                font-family: 'Courier New', monospace;
                font-size: 16px;
                padding: 10px;
            }
        """)
        self.log_text.setReadOnly(True)
        self.log_text.setOpenExternalLinks(False)
        layout.addWidget(self.log_text)
        
        central_widget.setLayout(layout)
        
        self.add_sample_logs()
    
    def mouseDoubleClickEvent(self, event):
        """
        Fecha a janela quando um duplo clique do mouse é detectado.

        Args:
            event (QMouseEvent): O evento de duplo clique do mouse.
        """
        self.close()
    
    def add_sample_logs(self):
        """
        Adiciona logs de exemplo para simular uma inspeção em andamento.
        Esta função gera dados realistas, incluindo timestamps, sequência de eventos,
        nomes de arquivos de fotos e links clicáveis para visualização das fotos.
        """
        current_time = datetime.now()
        inspection_type = "Flare"
        
        photo_names = []
        for i in range(1, 4):
            serial = f"{random.randint(100000, 999999):06d}"
            photo_name = f"{current_time.strftime('%Y%m%d_%H%M%S')}_{inspection_type}_{serial}.jpg"
            photo_names.append(photo_name)
        
        logs = [
            f"[{current_time.strftime('%H:%M:%S')}] Iniciando análise de inspeção...",
            f"[{current_time.strftime('%H:%M:%S')}] Câmera principal ativada",
            f"[{current_time.strftime('%H:%M:%S')}] Processamento YOLO iniciado",
            f"[{current_time.strftime('%H:%M:%S')}] Depth camera calibrada",
            f"[{current_time.strftime('%H:%M:%S')}] LIDAR operacional",
            f"[{current_time.strftime('%H:%M:%S')}] Análise em andamento...",
            "",
            "=== RESULTADOS DA ANÁLISE ===",
            "• Objetos detectados: 3",
            "• Anomalias encontradas: 1",
            "• Área inspecionada: 85%",
            "• Status: Em progresso",
            "",
            "=== FOTOS CAPTURADAS ===",
        ]
        
        self.log_content = "\n".join(logs) + "\n"
        
        for i, photo_name in enumerate(photo_names):
            descriptions = ["Visão geral da área", "Detalhe da anomalia", "Medição de profundidade"]
            link_text = f"📸 <a href='photo:{photo_name}' style='color: #3498db; text-decoration: underline;'>{photo_name}</a> - {descriptions[i]}"
            self.log_content += link_text + "\n"
        
        self.log_content += f"\n[{current_time.strftime('%H:%M:%S')}] Log atualizado automaticamente"
        
        self.log_text.setHtml(self.log_content.replace('\n', '<br>'))
        
        self.log_text.anchorClicked.connect(self.open_photo)
    
    def open_photo(self, url):
        """
        Abre uma janela de foto quando um link de foto é clicado no log.
        Esta função é chamada automaticamente quando um link com o esquema 'photo' é clicado.

        Args:
            url (QUrl): A URL do link clicado, contendo o nome da foto no caminho.
        """
        if url.scheme() == "photo":
            photo_name = url.path()
            
            photo_window = PhotoWindow(photo_name, self)
            self.photo_windows.append(photo_window)
            photo_window.show()
            
            self.log_text.setHtml(self.log_content.replace('\n', '<br>'))
            
            self.raise_()
            self.activateWindow()


