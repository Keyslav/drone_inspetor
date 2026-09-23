"""Seleção de modelos e detalhes visuais, independentes da tela de vídeo."""

import json
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QComboBox, QPushButton, QGroupBox
from PyQt6.QtGui import QCursor
from PyQt6.QtCore import Qt
from ..theme import COMMON_STYLES
from ..logging import gui_log_info, gui_log_error, gui_log_warn, gui_log_debug

class ModelDetails(QGroupBox):
    def __init__(self, title):
        """Cria um grupo estilizado para exibir detalhes do modelo."""
        from PyQt6.QtWidgets import QGridLayout

        super().__init__(title)
        group = self
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

    def set_model(self, model_data):
        """Atualiza os labels de um grupo de detalhes com os dados do modelo."""
        group = self
        if not model_data:
            return

        for key, label_widget in group.field_labels.items():
            val = model_data.get(key, "-")
            if isinstance(val, list):
                val = ", ".join(val)
            label_widget.setText(str(val))

class ModelSelector:
    """Possui catálogo, seleção e widgets; transmite escolhas pelos sinais existentes."""

    def __init__(self, signals):
        self.signals = signals
        self._equipment_models = []
        self._anomaly_models = []
        self._selected_equipment_model = ''
        self._selected_anomaly_model = ''
        self._equipment_dropdown = None
        self._anomaly_dropdown = None
        self.equip_details_group = None
        self.anom_details_group = None
        if hasattr(signals, 'models_received'):
            signals.models_received.connect(self.update_models)

    def create_widget(self):
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
        self.equip_details_group = ModelDetails("Equipamento Ativo")
        main_layout.addWidget(self.equip_details_group, stretch=1)

        # --- Coluna 2: Detalhes Anomalia ---
        self.anom_details_group = ModelDetails("Anomalia Ativa")
        main_layout.addWidget(self.anom_details_group, stretch=1)

        # --- Coluna 3: Seleção e Controles ---
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

        self.equip_details_group.set_model(next((m for m in self._equipment_models if m.get('file_name') == self._selected_equipment_model), {}))
        self.anom_details_group.set_model(next((m for m in self._anomaly_models if m.get('file_name') == self._selected_anomaly_model), {}))
        return controls

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

    def update_models(self, data):
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

            if self.equip_details_group is not None:
                self.equip_details_group.set_model(curr_obj_data)
            if self.anom_details_group is not None:
                self.anom_details_group.set_model(curr_anom_data)

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

