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
                             QPushButton, QTextEdit, QScrollArea, QComboBox, QSizePolicy)
from PyQt6.QtGui import QPixmap, QCursor, QFont
from PyQt6.QtCore import Qt
from .utils import ExpandedWindow, BaseScreen, ImageProcessor, COMMON_STYLES, IMAGE_QUALITY, gui_log_info, gui_log_error, gui_log_warn, gui_log_debug
import json
from datetime import datetime
import os
from .utils import ExpandedWindow, BaseScreen, ImageProcessor, COMMON_STYLES, IMAGE_QUALITY, gui_log_info, gui_log_error, gui_log_warn, gui_log_debug
import json
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
        if hasattr(self.signals, 'models_received'):
            self.signals.models_received.connect(self._on_models_received)
        
        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        
        # Instancia o ImageProcessor para converter mensagens de imagem ROS para formatos PyQt
        self.image_processor = ImageProcessor()

        # Variáveis para armazenar dados de análise e detecções
        self.analysis_logs = []
        self.current_detections = []
        self.analysis_window = None  # Referência para a janela de logs de análise
        
        # Listas de modelos (populadas via serviço)
        self._equipment_models = []
        self._anomaly_models = []
        
        # Modelos selecionados atualmente
        self._selected_equipment_model = ""
        self._selected_anomaly_model = ""
        
        # Widgets de dropdown (referências para atualização na janela expandida)
        self._equipment_dropdown = None
        self._anomaly_dropdown = None
        self._current_equipment_label = None
        self._current_anomaly_label = None
        self._expanded_label = None  # Label de imagem na janela expandida
        
        # Configura a aparência inicial do display de CV
        self.setup_cv_display()
        
        # Solicita lista de modelos inicial para popular cache
        if hasattr(self.signals, 'models_requested'):
            self.signals.models_requested.emit()
        
        gui_log_info("CVScreen", f"CVScreen inicializada - {len(self._equipment_models)} modelos de equipamento, {len(self._anomaly_models)} modelos de anomalia")
    
    
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
        controls_widget = self._create_model_controls()
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
    
    def _create_model_controls(self):
        """
        Cria widget com dropdowns para seleção de modelos de detecção e exibição detalhada.
        Layout: 3 Colunas (Detalhes Equip, Detalhes Anom, Seleção/Controles)
        
        Returns:
            QWidget: Widget contendo os controles de seleção de modelos.
        """
        controls = QWidget()
        main_layout = QHBoxLayout(controls)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(15)
        
        # Estilo comum para labels
        label_style = f"""
            color: {COMMON_STYLES["text_color"]};
            font-weight: bold;
            font-size: 12px;
        """
        
        # Estilo comum para dropdowns
        dropdown_style = f"""
            QComboBox {{
                background-color: #2b2b2b;
                color: #ffffff;
                border: 1px solid #555555;
                border-radius: 3px;
                padding: 5px;
                font-size: 11px;
            }}
            QComboBox::drop-down {{
                border: none;
                width: 20px;
            }}
        """

        # --- Coluna 1: Detalhes Equipamento ---
        self.equip_details_group = self._create_details_group("Equipamento Ativo")
        main_layout.addWidget(self.equip_details_group, stretch=1)
        
        # --- Coluna 2: Detalhes Anomalia ---
        self.anom_details_group = self._create_details_group("Anomalia Ativa")
        main_layout.addWidget(self.anom_details_group, stretch=1)

        # --- Coluna 3: Seleção e Controles ---
        from PyQt6.QtWidgets import QGroupBox
        selection_group = QGroupBox("Selecionar Modelos")
        selection_group.setStyleSheet(f"""
            QGroupBox {{
                color: {COMMON_STYLES["text_color"]};
                font-weight: bold;
                border: 1px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 15px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
                background-color: {COMMON_STYLES["dark_background"]};
            }}
        """)
        
        selection_layout = QVBoxLayout(selection_group)
        selection_layout.setContentsMargins(10, 15, 10, 10)
        selection_layout.setSpacing(10)

        # Label Equipamento
        equip_label = QLabel("Selecionar Modelo de Equipamento:")
        equip_label.setStyleSheet(label_style)
        selection_layout.addWidget(equip_label)
        
        # Dropdown Equipamento
        self._equipment_dropdown = QComboBox()
        self._equipment_dropdown.setStyleSheet(dropdown_style)
        self._equipment_dropdown.currentIndexChanged.connect(self._on_equipment_model_changed)
        selection_layout.addWidget(self._equipment_dropdown)
        
        # Popular dropdown de equipamentos se já houver dados
        if self._equipment_models:
            self._equipment_dropdown.blockSignals(True)
            for m in self._equipment_models:
                self._equipment_dropdown.addItem(m.get("name", "Unknown"), m.get("file_name", ""))
            
            # Tenta selecionar o modelo atual
            if self._selected_equipment_model:
                idx = self._equipment_dropdown.findData(self._selected_equipment_model)
                if idx >= 0:
                    self._equipment_dropdown.setCurrentIndex(idx)
            self._equipment_dropdown.blockSignals(False)
        
        # Spacer pequeno
        selection_layout.addSpacing(5)
        
        # Label Anomalia
        anom_label = QLabel("Selecionar Modelo de Anomalia:")
        anom_label.setStyleSheet(label_style)
        selection_layout.addWidget(anom_label)
        
        # Dropdown Anomalia
        self._anomaly_dropdown = QComboBox()
        self._anomaly_dropdown.setStyleSheet(dropdown_style)
        self._anomaly_dropdown.currentIndexChanged.connect(self._on_anomaly_model_changed)
        selection_layout.addWidget(self._anomaly_dropdown)
        
        # Popular dropdown de anomalias se já houver dados
        if self._anomaly_models:
            self._anomaly_dropdown.blockSignals(True)
            for m in self._anomaly_models:
                self._anomaly_dropdown.addItem(m.get("name", "Unknown"), m.get("file_name", ""))
            
            # Tenta selecionar o modelo atual
            if self._selected_anomaly_model:
                idx = self._anomaly_dropdown.findData(self._selected_anomaly_model)
                if idx >= 0:
                    self._anomaly_dropdown.setCurrentIndex(idx)
            self._anomaly_dropdown.blockSignals(False)
        
        # Spacer expansível para empurrar o botão para baixo (opcional, mas bom pra alinhar)
        selection_layout.addStretch()
        
        # Botão Aplicar
        apply_button = QPushButton("APLICAR NOVOS MODELOS")
        apply_button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        apply_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {COMMON_STYLES["success_color"]};
                color: white;
                border: 1px solid #1e8449;
                padding: 12px 20px;
                border-radius: 6px;
                font-weight: bold;
                font-size: 13px;
            }}
            QPushButton:hover {{
                background-color: #2ecc71;
                border: 1px solid #27ae60;
            }}
            QPushButton:pressed {{
                background-color: #196f3d;
            }}
        """)
        apply_button.clicked.connect(self._apply_model_selection)
        selection_layout.addWidget(apply_button)
        
        # Adiciona grupo de seleção ao layout principal
        main_layout.addWidget(selection_group, stretch=1)
        
        return controls

    def _create_details_group(self, title):
        """Cria um grupo estilizado para exibir detalhes do modelo."""
        from PyQt6.QtWidgets import QGroupBox, QGridLayout
        
        group = QGroupBox(title)
        group.setStyleSheet(f"""
            QGroupBox {{
                color: {COMMON_STYLES["text_color"]};
                font-weight: bold;
                border: 1px solid #555555;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 15px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
                background-color: {COMMON_STYLES["dark_background"]};
            }}
        """)
        
        layout = QGridLayout(group)
        layout.setContentsMargins(10, 15, 10, 10)
        layout.setSpacing(5)
        
        # Labels estáticos e dinâmicos (armazenados em dict no objeto group para acesso fácil)
        group.field_labels = {}
        fields = [
            ("Name", "name"),
            ("Dataset", "dataset"),
            ("Classes", "classes"),
            ("Model", "model"),
            ("File", "file_name"),
            ("Type", "type")
        ]
        
        for i, (display_name, key) in enumerate(fields):
            lbl_key = QLabel(f"{display_name}:")
            lbl_key.setStyleSheet("color: #aaaaaa; font-weight: bold; font-size: 11px;")
            
            lbl_val = QLabel("-")
            lbl_val.setStyleSheet("color: #ffffff; font-size: 11px;")
            lbl_val.setWordWrap(True)
            
            layout.addWidget(lbl_key, i, 0)
            layout.addWidget(lbl_val, i, 1)
            
            group.field_labels[key] = lbl_val
            
        return group

    def _update_details_group(self, group, model_data):
        """Atualiza os labels de um grupo de detalhes com os dados do modelo."""
        if not hasattr(group, 'field_labels') or not model_data:
            return
            
        for key, label_widget in group.field_labels.items():
            val = model_data.get(key, "-")
            if isinstance(val, list):
                val = ", ".join(val)
            label_widget.setText(str(val))
    
    def _on_equipment_model_changed(self, index):
        """Callback quando o modelo de equipamentos é alterado."""
        if self._equipment_dropdown and index >= 0:
            self._selected_equipment_model = self._equipment_dropdown.currentData()
            gui_log_info("CVScreen", f"Modelo de equipamentos selecionado: {self._selected_equipment_model}")
    
    def _on_anomaly_model_changed(self, index):
        """Callback quando o modelo de anomalias é alterado."""
        if self._anomaly_dropdown and index >= 0:
            self._selected_anomaly_model = self._anomaly_dropdown.currentData()
            gui_log_info("CVScreen", f"Modelo de anomalias selecionado: {self._selected_anomaly_model}")

    def _on_models_received(self, data):
        """
        Recebe a lista de modelos disponíveis e atuais do ROS.
        Atualiza a interface gráfica.
        """
        try:
            models_json_str = data.get('models_data_json', '[]')
            all_models = json.loads(models_json_str)
            
            # Filtra modelos por tipo
            self._equipment_models = [m for m in all_models if m.get("object_type") == "equipment"]
            self._anomaly_models = [m for m in all_models if m.get("object_type") == "anomaly"]
            
            curr_obj = data.get('current_object_model', '')
            curr_anom = data.get('current_anomaly_model', '')
            
            self._selected_equipment_model = curr_obj
            self._selected_anomaly_model = curr_anom
            
            gui_log_info("CVScreen", f"Modelos recebidos via serviço: {len(self._equipment_models)} equip, {len(self._anomaly_models)} anom")
            gui_log_info("CVScreen", f"Modelos atuais: {curr_obj} obj, {curr_anom} anom")
            
            # --- Atualiza displays de detalhes ---
            # Encontra os objetos completos dos modelos atuais
            curr_obj_data = next((m for m in self._equipment_models if m["file_name"] == curr_obj), {})
            curr_anom_data = next((m for m in self._anomaly_models if m["file_name"] == curr_anom), {})

            self._update_details_group(getattr(self, 'equip_details_group', None), curr_obj_data)
            self._update_details_group(getattr(self, 'anom_details_group', None), curr_anom_data)

            # --- Atualiza Dropdowns ---
            if self._equipment_dropdown:
                gui_log_debug("CVScreen", f"Atualizando Dropdown Equipamentos com {len(self._equipment_models)} itens")
                self._equipment_dropdown.blockSignals(True)
                self._equipment_dropdown.clear()
                for m in self._equipment_models:
                    # Usa 'name' para exibição e 'file_name' como dado
                    self._equipment_dropdown.addItem(m.get("name", "Unknown"), m.get("file_name", ""))
                
                # Seleciona o atual
                index = self._equipment_dropdown.findData(curr_obj)
                if index >= 0:
                    self._equipment_dropdown.setCurrentIndex(index)
                self._equipment_dropdown.blockSignals(False)
            else:
                gui_log_warn("CVScreen", "Dropdown Equipamentos não encontrado para atualização")

            # Atualiza Dropdown de Anomalias
            if self._anomaly_dropdown:
                gui_log_debug("CVScreen", f"Atualizando Dropdown Anomalias com {len(self._anomaly_models)} itens")
                self._anomaly_dropdown.blockSignals(True)
                self._anomaly_dropdown.clear()
                for m in self._anomaly_models:
                    self._anomaly_dropdown.addItem(m.get("name", "Unknown"), m.get("file_name", ""))
                
                # Seleciona o atual
                index = self._anomaly_dropdown.findData(curr_anom)
                if index >= 0:
                    self._anomaly_dropdown.setCurrentIndex(index)
                self._anomaly_dropdown.blockSignals(False)
            else:
                gui_log_warn("CVScreen", "Dropdown Anomalias não encontrado para atualização")
                
        except Exception as e:
            gui_log_error("CVScreen", f"Erro ao atualizar modelos na GUI: {e}")
            import traceback
            traceback.print_exc()

    
    def _apply_model_selection(self):
        """Aplica a seleção de modelos e envia para o cv_node via signal."""
        gui_log_info("CVScreen", f"Aplicando modelos: equip={self._selected_equipment_model}, anom={self._selected_anomaly_model}")
        
        if self.signals:
            self.signals.send_model_selection(self._selected_equipment_model, self._selected_anomaly_model)
            
            # Solicita atualização da tela após um breve delay para dar tempo do nó processar
            if hasattr(self.signals, 'models_requested'):
                from PyQt6.QtCore import QTimer
                QTimer.singleShot(1000, self.signals.models_requested.emit)
        else:
            gui_log_warn("CVScreen", "Signals não configurados - não foi possível enviar seleção")
    
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
