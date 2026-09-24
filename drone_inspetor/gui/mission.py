"""Resumo da missão e diagnóstico recolhível da máquina de estados.

A apresentação recebe snapshots do MissionNode por sinais Qt. Ela não determina
se um comando foi aceito: apenas o próximo estado recebido confirma a transição.
"""

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QLabel, QProgressBar, QSizePolicy, QToolButton, QTreeWidget,
    QTreeWidgetItem, QVBoxLayout, QWidget,
)


STATE_LABELS = {
    "DESATIVADO": "Desativada",
    "PRONTO": "Pronta para iniciar",
    "EXECUTANDO": "Em execução",
    "EXECUTANDO_ARMANDO": "Armando drone",
    "EXECUTANDO_DECOLANDO": "Decolando",
    "EXECUTANDO_INSPECIONANDO": "Inspecionando",
    "EXECUTANDO_INSPECIONANDO_DETECTANDO": "Buscando equipamento",
    "EXECUTANDO_INSPECIONANDO_ESCANEANDO": "Escaneando equipamento",
    "EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO": "Escaneamento concluído",
    "EXECUTANDO_INSPECIONANDO_FALHA": "Falha na detecção",
    "INSPECAO_FINALIZADA": "Inspeção finalizada",
    "RETORNANDO": "Retornando à origem",
}


class MissionManager:
    """Apresenta o estado confirmado sem acoplar widgets ao executor ROS."""

    def __init__(self, signals=None):
        self.signals = signals
        self.mission_tree = None
        self.mission_states = {}
        self.mission_highlight_items = {}
        self.current_state = ""
        self.current_mission_data = {
            "mission_name": "",
            "ponto_de_inspecao_indice_atual": 0,
            "total_pontos_de_inspecao": 0,
            "objeto_alvo": "",
            "tipos_anomalia": [],
        }
        self.state_label = None

    @staticmethod
    def _label(text, color="#93a4bb", size=12):
        label = QLabel(text)
        label.setWordWrap(True)
        label.setMinimumWidth(0)
        label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        label.setStyleSheet(
            f"color: {color}; font-size: {size}px; background: transparent; border: none;"
        )
        return label

    def setup_b2_mission(self):
        """Cria o resumo sempre visível e deixa a árvore fora do fluxo principal."""
        panel = QWidget()
        panel.setObjectName("missionSummary")
        panel.setStyleSheet("""
            QWidget#missionSummary { background: #111c2e; border: 1px solid #26354b;
                                     border-radius: 12px; }
        """)
        panel.setMinimumWidth(0)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(6)
        layout.addWidget(self._label("ESTADO DA MISSÃO", size=11))
        self.state_label = self._label("Aguardando telemetria", "#e6edf5", 20)
        self.state_label.setAccessibleName("Estado atual da missão")
        layout.addWidget(self.state_label)
        self.mission_name_label = self._label("Nenhuma missão informada")
        layout.addWidget(self.mission_name_label)
        self.progress_label = self._label("Pontos de inspeção não informados")
        layout.addWidget(self.progress_label)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 1)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(5)
        self.progress_bar.setAccessibleName("Posição do ponto atual na missão")
        self.progress_bar.setStyleSheet("""
            QProgressBar { background: #26354b; border: none; border-radius: 2px; }
            QProgressBar::chunk { background: #4de0c1; border-radius: 2px; }
        """)
        layout.addWidget(self.progress_bar)
        self.target_label = self._label("")
        self.anomalies_label = self._label("")
        layout.addWidget(self.target_label)
        layout.addWidget(self.anomalies_label)
        self.details_toggle = QToolButton()
        self.details_toggle.setText("Máquina de estados")
        self.details_toggle.setCheckable(True)
        self.details_toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.details_toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.details_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.details_toggle.setMinimumHeight(30)
        self.details_toggle.setStyleSheet("""
            QToolButton { color: #93a4bb; background: transparent; border: none;
                          text-align: left; font-size: 12px; }
            QToolButton:hover, QToolButton:checked { color: #4de0c1; }
        """)
        layout.addWidget(self.details_toggle)
        self.setup_mission_tree(layout)
        self.details_toggle.toggled.connect(self._toggle_details)
        self._toggle_details(False)
        self.update_state(self.current_state)
        return panel

    def _toggle_details(self, expanded):
        self.mission_tree.setVisible(expanded)
        self.details_toggle.setArrowType(
            Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow
        )

    def setup_mission_tree(self, layout):
        """A árvore mantém os nomes de diagnóstico sem disputar espaço com o resumo."""
        self.mission_tree = QTreeWidget()
        self.mission_tree.setHeaderHidden(True)
        self.mission_tree.setIndentation(12)
        self.mission_tree.setMinimumWidth(0)
        self.mission_tree.setMinimumHeight(170)
        self.mission_tree.setMaximumHeight(220)
        self.mission_tree.setStyleSheet("""
            QTreeWidget { background: #0b1321; color: #93a4bb; border: none;
                          border-radius: 8px; font-size: 11px; }
            QTreeWidget::item { padding: 5px 2px; }
            QTreeWidget::item:selected { background: #183c3d; color: #4de0c1; }
        """)
        self.create_mission_tree_structure()
        layout.addWidget(self.mission_tree)

    def create_mission_tree_structure(self):
        self.mission_tree.clear()
        self.mission_states.clear()
        self.mission_highlight_items.clear()
        root = QTreeWidgetItem(self.mission_tree, ["Ciclo da missão"])
        # A hierarquia é explícita: separar por '_' quebraria ESCANEAMENTO_FINALIZADO.
        parents = {
            "EXECUTANDO_ARMANDO": "EXECUTANDO",
            "EXECUTANDO_DECOLANDO": "EXECUTANDO",
            "EXECUTANDO_INSPECIONANDO": "EXECUTANDO",
            **{
                state: "EXECUTANDO_INSPECIONANDO"
                for state in STATE_LABELS if state.startswith("EXECUTANDO_INSPECIONANDO_")
            },
        }
        for state, label in STATE_LABELS.items():
            parent_state = parents.get(state)
            parent = self.mission_states[parent_state] if parent_state else root
            item = QTreeWidgetItem(parent, [label])
            item.setToolTip(0, state)
            self.mission_states[state] = item
            ancestors = self.mission_highlight_items.get(parent_state, [])
            self.mission_highlight_items[state] = [*ancestors, item]
        self.mission_tree.expandAll()

    def highlight_current_state_in_tree(self, state):
        if self.mission_tree is None:
            return
        self.mission_tree.clearSelection()
        active_items = self.mission_highlight_items.get(state, [])
        for item in self.mission_states.values():
            font = item.font(0)
            font.setBold(item in active_items)
            item.setFont(0, font)
        if active_items:
            self.mission_tree.setCurrentItem(active_items[-1])

    def mission_state_callback(self, msg):
        """Compatibilidade com conexões antigas do sinal da missão."""
        self.update_state(msg)

    def update_state(self, state_data):
        """Mostra o snapshot recebido; cliques em Iniciar/Cancelar não o alteram."""
        if isinstance(state_data, dict):
            self.current_state = state_data.get("state_name", "")
            for key, default in (
                ("mission_name", ""), ("ponto_de_inspecao_indice_atual", 0),
                ("total_pontos_de_inspecao", 0), ("objeto_alvo", ""),
                ("tipos_anomalia", []),
            ):
                self.current_mission_data[key] = state_data.get(key, default)
        else:
            self.current_state = str(state_data)
        if self.state_label is None:
            return
        label = STATE_LABELS.get(
            self.current_state,
            self.current_state.replace("_", " ").capitalize() or "Aguardando telemetria",
        )
        self.state_label.setText(label)
        self.state_label.setToolTip(self.current_state)
        color = "#ff7f88" if "FALHA" in self.current_state else "#e6edf5"
        self.state_label.setStyleSheet(
            f"color: {color}; font-size: 20px; font-weight: 600; border: none;"
            "background: transparent;"
        )
        self._update_dynamic_labels()
        self.highlight_current_state_in_tree(self.current_state)

    def _update_dynamic_labels(self):
        data = self.current_mission_data
        self.mission_name_label.setText(data["mission_name"] or "Nenhuma missão informada")
        total = max(0, int(data["total_pontos_de_inspecao"]))
        # O índice informa o ponto atual, não o número de inspeções concluídas.
        # Por isso a barra não é apresentada como uma porcentagem de conclusão.
        current = min(total, max(0, int(data["ponto_de_inspecao_indice_atual"])) + 1)
        self.progress_label.setText(
            f"Ponto atual · {current} de {total}" if total else "Pontos de inspeção não informados"
        )
        self.progress_bar.setRange(0, max(1, total))
        self.progress_bar.setValue(current)
        # Sem pontos recebidos, o estado de espera já comunica a ausência de dados.
        self.progress_label.setVisible(total > 0)
        self.progress_bar.setVisible(total > 0)
        target = data["objeto_alvo"]
        anomalies = ", ".join(str(value).replace("_", " ") for value in data["tipos_anomalia"])
        self.target_label.setText(f"Equipamento · {target}" if target else "")
        self.target_label.setVisible(bool(target))
        self.anomalies_label.setText(f"Análises · {anomalies}" if anomalies else "")
        self.anomalies_label.setVisible(bool(anomalies))
