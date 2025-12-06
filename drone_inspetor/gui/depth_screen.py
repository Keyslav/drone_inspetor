"""
depth_screen.py
=================================================================================================
Tela de visualização de câmera de profundidade.

Gerencia a exibição de dados de câmera de profundidade. Esta classe é um componente
da interface gráfica (GUI) e não se comunica diretamente com o ROS2. Ela recebe os dados
de imagem e análise processados pelo DashboardNode através de sinais PyQt6 e os exibe.
=================================================================================================
"""

from PyQt6.QtWidgets import (QLabel, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton)
from PyQt6.QtGui import QPixmap, QCursor
from PyQt6.QtCore import Qt
from .utils import ExpandedWindow, BaseScreen, ImageProcessor, COMMON_STYLES, gui_log_debug, gui_log_info, gui_log_error, gui_log_warn
import json
from datetime import datetime

class DepthScreen(BaseScreen):
    """
    Gerencia a exibição de dados de câmera de profundidade.
    
    Esta classe é um componente da interface gráfica (GUI) e não se comunica
    diretamente com o ROS2. Ela recebe os dados de imagem e análise processados
    pelo DashboardNode através de sinais PyQt6 e os exibe.
    """
    
    def __init__(self, signals, video_label):
        """
        Inicializa a tela da câmera de profundidade.

        Args:
            signals: Objeto de sinais PyQt6 para comunicação (não utilizado diretamente aqui)
            video_label (QLabel): O widget QLabel onde a imagem de profundidade será exibida.
        """
        # Chama o construtor da classe base BaseScreen
        super().__init__(video_label, "Câmera de Profundidade")

        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        self.image_processor = ImageProcessor()

        # Variáveis para armazenar estatísticas e alertas de proximidade.
        self.current_statistics = {}
        self.proximity_alerts = []
        self.visualization_mode = "grayscale"  # Modo de visualização padrão: "grayscale" ou "colormap"
        
        # Configura a aparência inicial do display de profundidade
        self.setup_depth_display()
        
        gui_log_info("DepthScreen", "DepthScreen inicializada - foco em exibição de profundidade")
    
    def setup_depth_display(self):
        """
        Configura o estilo visual e o comportamento do QLabel que exibe o feed de profundidade.
        Define cores de fundo, bordas, texto inicial e o cursor do mouse.
        """
        if self.video_label:
            self.video_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {COMMON_STYLES["dark_background"]};
                    color: {COMMON_STYLES["text_color"]};
                    border: 2px solid #9b59b6;  /* Roxo para profundidade */
                    border-radius: 5px;
                    font-size: 16px;
                }}
            """)
            
            self.video_label.setText("Aguardando Câmera de Profundidade...")
            self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            self.video_label.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
            
            gui_log_info("DepthScreen", "Display de profundidade configurado")
    
    def update_depth_image(self, cv_image):
        """
        Atualiza a exibição com uma nova imagem de profundidade.
        Este método é chamado pelo DashboardNode quando uma nova mensagem de imagem
        é recebida do tópico ROS2 de profundidade através de sinais PyQt6.

        Args:
            cv_image (numpy.ndarray): A imagem no formato OpenCV (numpy array).
        """
        gui_log_debug("DepthScreen", "Atualizando imagem de profundidade")
        
        try:
            # Converte a imagem OpenCV para QImage, que pode ser exibida em um QLabel
            q_image = self.image_processor.cv_to_qimage(cv_image)
            
            if q_image:
                # Atualiza o QLabel principal e quaisquer janelas expandidas com a nova imagem
                self.update_display(q_image)
                gui_log_debug("DepthScreen", "Imagem de profundidade exibida com sucesso")
            else:
                gui_log_warn("DepthScreen", "Falha na conversão da imagem de profundidade")
                
        except Exception as e:
            gui_log_error("DepthScreen", f"Erro no callback de profundidade: {e}")
    
    def update_depth_statistics(self, statistics_dict):
        """
        Atualiza as estatísticas de profundidade recebidas do nó de profundidade.
        As estatísticas são esperadas em formato Dict.

        Args:
            statistics_dict (dict): Dicionário contendo as estatísticas de profundidade.
        """
        gui_log_debug("DepthScreen", "Atualizando estatísticas de profundidade")
        
        try:
            self.current_statistics = statistics_dict
            gui_log_debug("DepthScreen", f"Estatísticas atualizadas: {len(statistics_dict)} campos")
            
        except Exception as e:
            gui_log_error("DepthScreen", f"Erro ao atualizar estatísticas: {e}")
    
    def update_proximity_alerts(self, alert_list):
        """
        Atualiza os alertas de proximidade recebidos do nó de profundidade.
        Os alertas são esperados em formato List.

        Args:
            alert_list (list): Lista contendo os dados de alertas.
        """
        gui_log_debug("DepthScreen", "Atualizando alertas de proximidade")
        
        try:
            self.proximity_alerts = alert_list
            gui_log_debug("DepthScreen", f"Alertas atualizados: {len(self.proximity_alerts)} alertas")
            
        except Exception as e:
            gui_log_error("DepthScreen", f"Erro ao atualizar alertas: {e}")
    
    def toggle_visualization_mode(self):
        """
        Alterna entre os modos de visualização da imagem de profundidade.
        Os modos podem ser "grayscale" (tons de cinza) ou "colormap" (mapa de cores).
        """
        match self.visualization_mode:
            case "grayscale":
                self.visualization_mode = "colormap"
            case "colormap":
                self.visualization_mode = "grayscale"
            case _:
                self.visualization_mode = "grayscale"
        
        gui_log_info("DepthScreen", f"Modo de visualização alterado para: {self.visualization_mode}")
    
    def reset_depth_data(self):
        """
        Reseta todos os dados de profundidade armazenados (estatísticas e alertas).
        """
        gui_log_info("DepthScreen", "Resetando dados de profundidade")
        
        self.current_statistics = {}
        self.proximity_alerts = []
        
        gui_log_info("DepthScreen", "Dados de profundidade resetados")
    
    def expand_screen(self, event=None):
        """
        Sobrescreve o método `expand_screen` da classe base `BaseScreen`.
        Cria e exibe uma janela expandida dedicada para a visualização da câmera de profundidade,
        incluindo controles para alternar modos de visualização e exibir estatísticas/alertas.

        Args:
            event (QMouseEvent, optional): O evento de clique do mouse que disparou a expansão.
                                           Pode ser None se chamado programaticamente.
        """
        gui_log_info("DepthScreen", "Expandindo visualização de profundidade")
        
        # Cria um widget container para a janela expandida.
        expanded_widget = QWidget()
        layout = QVBoxLayout()
        
        # Layout para os botões de controle da tela de profundidade.
        controls_layout = QHBoxLayout()
        
        # Botão para alternar o modo de visualização.
        mode_button = QPushButton(f"Modo: {self.visualization_mode}")
        mode_button.clicked.connect(self.toggle_visualization_mode)
        mode_button.setStyleSheet(f"""
            QPushButton {{
                background-color: #9b59b6;  /* Roxo para profundidade */
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #8e44ad;
            }}
        """)
        controls_layout.addWidget(mode_button)
        
        # Botão para exibir estatísticas de profundidade.
        stats_button = QPushButton("Estatísticas")
        stats_button.clicked.connect(self.show_depth_statistics)
        stats_button.setStyleSheet(f"""
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
        controls_layout.addWidget(stats_button)
        
        # Botão para resetar os dados de profundidade.
        reset_button = QPushButton("Reset Dados")
        reset_button.clicked.connect(self.reset_depth_data)
        reset_button.setStyleSheet(f"""
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
        controls_layout.addWidget(reset_button)
        
        controls_layout.addStretch()
        layout.addLayout(controls_layout)
        
        # QLabel para exibir a imagem de profundidade na janela expandida.
        expanded_label = QLabel()
        expanded_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        expanded_label.setStyleSheet(f"""
            QLabel {{
                background-color: {COMMON_STYLES["dark_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid #9b59b6;
                border-radius: 5px;
                font-size: 20px;
            }}
        """)
        
        # Copia o conteúdo atual do QLabel principal para a janela expandida, se houver.
        if self.video_label and self.video_label.pixmap():
            original_pixmap = self.video_label.pixmap()
            # Redimensiona a imagem para a janela expandida, mantendo a proporção.
            expanded_label.setPixmap(original_pixmap.scaled(
                1600, 900, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            expanded_label.setText("Aguardando Câmera de Profundidade...")
        
        layout.addWidget(expanded_label)
        expanded_widget.setLayout(layout)
        
        # Cria e exibe a janela expandida usando a classe ExpandedWindow.
        window = ExpandedWindow("Câmera de Profundidade", expanded_widget, None)
        self.expanded_windows.append(window) # Mantém uma referência para a janela expandida.
        window.show()
    
    def show_depth_statistics(self):
        """
        Exibe uma janela separada com estatísticas detalhadas da câmera de profundidade.
        """
        gui_log_info("DepthScreen", "Mostrando estatísticas de profundidade")
        
        try:
            # Cria um widget para a janela de estatísticas.
            stats_widget = QWidget()
            stats_widget.setWindowTitle("Estatísticas de Profundidade")
            stats_widget.setGeometry(300, 300, 600, 500)
            
            layout = QVBoxLayout()
            
            # Título da janela de estatísticas.
            title = QLabel("Estatísticas de Câmera de Profundidade")
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setStyleSheet(f"""
                font-weight: bold;
                font-size: 16px;
                color: {COMMON_STYLES["text_color"]};
                background-color: #9b59b6;
                padding: 10px;
                border-radius: 5px;
                margin-bottom: 10px;
            """)
            layout.addWidget(title)
            
            # Conteúdo das estatísticas gerado dinamicamente.
            stats_content = self.generate_statistics_content()
            
            stats_label = QLabel(stats_content)
            stats_label.setAlignment(Qt.AlignmentFlag.AlignTop)
            stats_label.setStyleSheet(f"""
                QLabel {{
                    background-color: {COMMON_STYLES["light_background"]};
                    color: {COMMON_STYLES["text_color"]};
                    border: 2px solid {COMMON_STYLES["border_color"]};
                    border-radius: 5px;
                    padding: 15px;
                    font-family: "Courier New", monospace;
                    font-size: 12px;
                }}
            """)
            layout.addWidget(stats_label)
            
            # Botão para fechar a janela de estatísticas.
            close_button = QPushButton("Fechar")
            close_button.clicked.connect(stats_widget.close)
            close_button.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COMMON_STYLES["accent_color"]};
                    color: {COMMON_STYLES["text_color"]};
                    border: none;
                    padding: 10px 20px;
                    border-radius: 4px;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: {COMMON_STYLES["success_color"]};
                }}
            """)
            layout.addWidget(close_button)
            
            stats_widget.setLayout(layout)
            stats_widget.setStyleSheet(f"""
                QWidget {{
                    background-color: {COMMON_STYLES["dark_background"]};
                }}
            """)
            
            stats_widget.show()
            
            self.expanded_windows.append(stats_widget)
            
        except Exception as e:
            gui_log_error("DepthScreen", f"Erro ao mostrar estatísticas: {e}")
    
    def generate_statistics_content(self):
        """
        Gera o conteúdo formatado das estatísticas de profundidade para exibição.

        Returns:
            str: Uma string contendo o conteúdo formatado das estatísticas.
        """
        if not self.current_statistics:
            return "Nenhuma estatística disponível.\nAguardando dados de profundidade..."
        
        content = []
        
        content.append("ESTATÍSTICAS DE PROFUNDIDADE")
        content.append("=" * 40)
        
        timestamp = self.current_statistics.get("timestamp", "N/A")
        content.append(f"Última atualização: {timestamp}")
        content.append("")
        
        total_pixels = self.current_statistics.get("total_pixels", 0)
        valid_pixels = self.current_statistics.get("valid_pixels", 0)
        valid_percentage = self.current_statistics.get("valid_percentage", 0)
        
        content.append("DADOS DE PIXELS:")
        content.append(f"  Total de pixels: {total_pixels:,}")
        content.append(f"  Pixels válidos: {valid_pixels:,}")
        content.append(f"  Porcentagem válida: {valid_percentage:.1f}%")
        content.append("")
        
        min_distance = self.current_statistics.get("min_distance", 0)
        max_distance = self.current_statistics.get("max_distance", 0)
        mean_distance = self.current_statistics.get("mean_distance", 0)
        median_distance = self.current_statistics.get("median_distance", 0)
        std_distance = self.current_statistics.get("std_distance", 0)
        range_meters = self.current_statistics.get("range_meters", 0)
        
        content.append("ANÁLISE DE DISTÂNCIAS:")
        content.append(f"  Distância mínima: {min_distance:.3f} m")
        content.append(f"  Distância máxima: {max_distance:.3f} m")
        content.append(f"  Distância média: {mean_distance:.3f} m")
        content.append(f"  Distância mediana: {median_distance:.3f} m")
        content.append(f"  Desvio padrão: {std_distance:.3f} m")
        content.append(f"  Range total: {range_meters:.3f} m")
        content.append("")
        
        close_pixels = self.current_statistics.get("close_pixels", 0)
        medium_pixels = self.current_statistics.get("medium_pixels", 0)
        far_pixels = self.current_statistics.get("far_pixels", 0)
        close_percentage = self.current_statistics.get("close_percentage", 0)
        medium_percentage = self.current_statistics.get("medium_percentage", 0)
        far_percentage = self.current_statistics.get("far_percentage", 0)
        
        content.append("DISTRIBUIÇÃO POR FAIXAS:")
        content.append(f"  Próximo (≤1m): {close_pixels:,} pixels ({close_percentage:.1f}%)")
        content.append(f"  Médio (1-5m): {medium_pixels:,} pixels ({medium_percentage:.1f}%)")
        content.append(f"  Longe (>5m): {far_pixels:,} pixels ({far_percentage:.1f}%)")
        content.append("")
        
        if self.proximity_alerts:
            content.append("ALERTAS DE PROXIMIDADE:")
            for alert in self.proximity_alerts:
                level = alert.get("level", "N/A")
                min_distance = alert.get("min_distance", 0)
                affected_percentage = alert.get("affected_percentage", 0)
                
                content.append(f"  • {level}: {min_distance:.3f}m ({affected_percentage:.1f}% da imagem)")
        else:
            content.append("ALERTAS DE PROXIMIDADE:")
            content.append("  Nenhum alerta ativo")
        
        return "\n".join(content)


