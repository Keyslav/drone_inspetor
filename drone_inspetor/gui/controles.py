"""Controles de missão desacoplados da confirmação de estado.

O seletor publica apenas a prévia da rota. Os botões enviam solicitações pelos
sinais do dashboard; aceitação e execução continuam sob autoridade do MissionNode.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QPushButton, QComboBox, QGridLayout, QScrollArea, QMainWindow, QSizePolicy)
from PyQt6.QtGui import QCursor
from PyQt6.QtCore import Qt
import os
import subprocess
import yaml
import json
from .utils import gui_log_info, gui_log_error, gui_log_warn

class ControlesManager:
    """
    Gerencia os controles da missão e a interface de simulação Gazebo.

    Esta classe é um componente da interface gráfica (GUI) e interage com
    o sistema ROS2 através dos sinais PyQt6, que contêm métodos de publicação de comandos.
    """
    def __init__(self, signals, mapa_signals=None):
        """
        Inicializa o gerenciador de controles.

        Args:
            signals (DashboardSignals.DroneSignals): Objeto de sinais de controle do drone.
                                                     Contém métodos de publicação de comandos incorporados.
            mapa_signals (DashboardSignals.MapaSignals): Sinais do mapa para emitir seleção de missão.
        """
        self.signals = signals  # Armazena a referência aos sinais de controle
        self.mapa_signals = mapa_signals  # Sinais do mapa para missão selecionada

        # Inicializa os atributos que armazenarão os widgets e janelas relacionadas aos controles.
        self.inspection_selector = None
        self.start_button = None
        self.cancel_button = None
        self.log_button = None
        self.gazebo_window = None

        # Missões serão recebidas do dashboard_gui via set_missions()
        self.missions = {}

    def set_missions(self, missions: dict):
        """Atualiza a lista sem trocar uma seleção ainda disponível nem disparar comandos."""
        self.missions = missions
        if self.inspection_selector is not None:
            selected = self.inspection_selector.currentText()
            self.inspection_selector.blockSignals(True)
            self.inspection_selector.clear()
            self.inspection_selector.addItems(list(missions))
            if selected in missions:
                self.inspection_selector.setCurrentText(selected)
            self.inspection_selector.blockSignals(False)
            self._on_mission_selected(self.inspection_selector.currentText())
            self.start_button.setEnabled(bool(missions))

    def setup_b3_controls(self):
        """Cria controles fluidos: nomes longos não forçam a largura do painel."""
        panel = QWidget()
        panel.setObjectName("missionControls")
        panel.setMinimumWidth(0)
        panel.setStyleSheet("""
            QWidget#missionControls { background: #111c2e; border: 1px solid #26354b;
                                      border-radius: 12px; }
            QLabel { background: transparent; border: none; color: #93a4bb; }
        """)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(6)
        title = QLabel("PLANEJAR E EXECUTAR")
        title.setStyleSheet("font-size: 11px; color: #93a4bb; border: none;")
        layout.addWidget(title)
        selector_label = QLabel("Missão de inspeção")
        selector_label.setStyleSheet("color: #e6edf5; font-size: 13px; border: none;")
        layout.addWidget(selector_label)
        self.inspection_selector = QComboBox()
        self.inspection_selector.setAccessibleName("Missão de inspeção")
        self.inspection_selector.setMinimumWidth(0)
        self.inspection_selector.setMinimumHeight(38)
        self.inspection_selector.setMinimumContentsLength(1)
        self.inspection_selector.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.inspection_selector.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self.inspection_selector.setStyleSheet("""
            QComboBox { background: #0b1321; color: #e6edf5; border: 1px solid #26354b;
                        border-radius: 7px; padding: 7px 10px; font-size: 13px; }
            QComboBox:hover, QComboBox:focus { border-color: #4de0c1; }
            QComboBox::drop-down { border: none; width: 24px; }
            QComboBox QAbstractItemView { background: #111c2e; color: #e6edf5;
                selection-background-color: #183c3d; selection-color: #4de0c1;
                border: 1px solid #26354b; }
        """)
        self.inspection_selector.addItems(list(self.missions))
        self.inspection_selector.currentTextChanged.connect(self._on_mission_selected)
        layout.addWidget(self.inspection_selector)
        self.setup_control_buttons(layout)
        # Selecionar uma missão apenas desenha a rota no mapa. O envio exige clique.
        self._on_mission_selected(self.inspection_selector.currentText())
        return panel

    def _on_mission_selected(self, mission_name: str):
        """
        Callback chamado quando uma missão é selecionada no dropdown.
        Emite o sinal mission_selected para que o mapa exiba os pontos.

        Args:
            mission_name (str): Nome da missão selecionada.
        """
        if self.inspection_selector is not None:
            self.inspection_selector.setToolTip(mission_name)
        if mission_name and self.mapa_signals:
            gui_log_info("ControlesManager", f"Missão selecionada: {mission_name}")
            self.mapa_signals.mission_selected.emit(mission_name)

    def setup_control_buttons(self, layout):
        """Dá destaque à ação principal mantendo o cancelamento acessível."""
        buttons_layout = QGridLayout()
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(8)
        self.start_button = CleanButton("Iniciar missão", "#4de0c1", primary=True)
        self.start_button.setEnabled(bool(self.missions))
        self.start_button.clicked.connect(self.iniciar_missao)
        buttons_layout.addWidget(self.start_button, 0, 0, 1, 2)
        self.cancel_button = CleanButton("Cancelar", "#ff7f88")
        self.cancel_button.setToolTip(
            "Solicitar cancelamento e retorno à origem. "
            "Acompanhe a confirmação no estado da missão."
        )
        self.cancel_button.clicked.connect(self.cancelar_missao)
        buttons_layout.addWidget(self.cancel_button, 1, 0)
        self.log_button = CleanButton("Análises", "#e6edf5")
        self.log_button.setToolTip("Abrir análises e registros de inspeção")
        self.log_button.clicked.connect(self.open_log_analysis)
        buttons_layout.addWidget(self.log_button, 1, 1)
        buttons_layout.setColumnStretch(0, 1)
        buttons_layout.setColumnStretch(1, 1)
        layout.addLayout(buttons_layout)

    def open_gazebo_simulation(self):
        """
        Abre a janela de controle da simulação Gazebo.
        Cria uma nova instância da janela se ela não existir ou estiver fechada.
        """
        gui_log_info("ControlesManager", "Abrindo janela de simulação Gazebo")

        # Verifica se a janela do Gazebo já existe e está visível.
        if self.gazebo_window is None or not self.gazebo_window.isVisible():
            # Se não, cria uma nova instância da janela.
            self.gazebo_window = GazeboSimulationWindow(self)

        # Exibe a janela, traz para a frente e ativa-a.
        self.gazebo_window.show()
        self.gazebo_window.raise_()
        self.gazebo_window.activateWindow()

    def open_log_analysis(self):
        """
        Abre a janela de análise de logs.
        Importa a classe `LogAnalysisWindow` dinamicamente para evitar dependências circulares.
        """
        # Importa a classe LogAnalysisWindow dinamicamente
        from .log_analise import LogAnalysisWindow
        gui_log_info("ControlesManager", "Abrindo janela de análise de logs")

        # Verifica se a janela de log já existe e está visível.
        if not hasattr(self, 'log_window') or self.log_window is None or not self.log_window.isVisible():
            # Se não, cria uma nova instância da janela.
            self.log_window = LogAnalysisWindow(None)

        # Exibe a janela, traz para a frente e ativa-a.
        self.log_window.show()
        self.log_window.raise_()
        self.log_window.activateWindow()

    def iniciar_missao(self):
        """
        Publica um comando ROS2 para iniciar uma nova missão.
        O nome da missão é obtido do seletor de missão e deve corresponder
        a uma missão definida em missions.json.
        """
        gui_log_info("ControlesManager", "Botão Iniciar Missão clicado")

        # Obtém o nome da missão selecionada no QComboBox
        selected_mission = self.inspection_selector.currentText()
        if not selected_mission:
            return
        # Publicar a solicitação não confirma aceitação nem altera o estado visual.
        # Somente a telemetria do MissionNode confirma a execução da missão.
        gui_log_info("ControlesManager", f"Missão selecionada: {selected_mission}")

        # Usa o método de compatibilidade para publicar o comando
        # O publisher converte internamente para o novo formato
        command_dict = {
            "command": "iniciar_missao",
            "mission": selected_mission
        }
        command_json = json.dumps(command_dict)
        self.signals.send_mission_command(command_json)

    def cancelar_missao(self):
        """
        Publica um comando ROS2 para cancelar a missão atual.
        O drone irá executar RTL (Return To Launch) automaticamente.
        """
        gui_log_info("ControlesManager", "Botão Cancelar Missão clicado")

        # Usa o método de compatibilidade para publicar o comando
        command_dict = {
            "command": "cancelar_missao"
        }
        command_json = json.dumps(command_dict)
        self.signals.send_mission_command(command_json)

    # Métodos legados mantidos para compatibilidade
    def start_inspection(self):
        """DEPRECATED: Use iniciar_missao() ao invés."""
        self.iniciar_missao()

    def cancel_inspection(self):
        """DEPRECATED: Use cancelar_missao() ao invés."""
        self.cancelar_missao()

class CleanButton(QPushButton):
    """Botão plano com foco visível e área de clique independente do texto."""

    def __init__(self, text, icon_color, parent=None, *, primary=False):
        super().__init__(text, parent)
        self.icon_color = icon_color
        self.setMinimumWidth(0)
        self.setMinimumHeight(38)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        background = "#4de0c1" if primary else "#0b1321"
        foreground = "#092522" if primary else icon_color
        hover = "#76ebd3" if primary else "#1b2c43"
        self.setStyleSheet(f"""
            QPushButton {{ background: {background}; color: {foreground};
                border: 1px solid {"#4de0c1" if primary else "#26354b"};
                border-radius: 7px; padding: 7px 10px; font-size: 13px; font-weight: 600; }}
            QPushButton:hover {{ background: {hover}; border-color: #4de0c1; }}
            QPushButton:focus {{ border: 2px solid #e6edf5; }}
            QPushButton:pressed {{ background: #23483f; color: #e6edf5; }}
            QPushButton:disabled {{ background: #172337; color: #60718a; border-color: #26354b; }}
        """)


class GazeboSimulationWindow(QMainWindow):
    """
    Janela dedicada para controlar e visualizar a simulação Gazebo.
    Permite ao usuário interagir com o ambiente simulado, como iniciar/parar a simulação,
    resetar o ambiente e visualizar o mapa da simulação.
    """
    def __init__(self, parent=None):
        """
        Inicializa a janela de simulação Gazebo.

        Args:
            parent (QWidget, optional): O widget pai (geralmente ControlesManager).
        """
        super().__init__(parent)

        # Define o título e a geometria da janela
        self.setWindowTitle("Drone Inspetor - Simulação Gazebo")
        self.setGeometry(200, 200, 1200, 800)
        # Armazena referência ao ControlesManager (parent) para acesso aos signals
        self.parent_manager = parent

        # Define o estilo CSS para a janela principal.
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2c3e50;
                color: #ecf0f1;
            }
        """)

        # Configura o widget central e seu layout principal.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Configura as áreas de mapa e botões.
        self.setup_map_area(main_layout)
        self.setup_buttons_area(main_layout)

        # Define o layout para o widget central.
        central_widget.setLayout(main_layout)

        # Carrega os parâmetros específicos do Gazebo.
        self.load_gazebo_parameters()

    def setup_map_area(self, main_layout):
        """
        Configura a área do mapa na janela de simulação Gazebo.
        Inclui um QLabel para o mapa e um QScrollArea para logs.

        Args:
            main_layout (QHBoxLayout): O layout principal da janela.
        """
        # Cria o widget e layout para a área do mapa.
        map_area_widget = QWidget()
        map_area_layout = QVBoxLayout()
        map_area_layout.setContentsMargins(0, 0, 0, 0)
        map_area_layout.setSpacing(5)

        # Cria o QLabel para o título da visualização do mapa.
        map_label = QLabel("Visualização do Mapa da Simulação")
        map_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        map_label.setStyleSheet("""
            QLabel {
                background-color: #34495e;
                color: #ecf0f1;
                padding: 5px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        map_area_layout.addWidget(map_label)

        # Cria o QLabel para exibir o mapa da simulação.
        self.map_display = QLabel("Mapa da Simulação Aqui")
        self.map_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_display.setStyleSheet("""
            QLabel {
                background-color: #2c3e50;
                border: 1px solid #7f8c8d;
                border-radius: 5px;
            }
        """)
        map_area_layout.addWidget(self.map_display)

        # Cria uma área de rolagem para exibir logs.
        log_scroll_area = QScrollArea()
        log_scroll_area.setWidgetResizable(True)
        log_scroll_area.setStyleSheet("""
            QScrollArea {
                border: 1px solid #7f8c8d;
                border-radius: 5px;
            }
            QScrollArea > QWidget > QWidget {
                background-color: #2c3e50;
            }
        """)

        # Cria o QLabel para exibir o texto dos logs.
        self.log_text_edit = QLabel("Logs da Simulação Gazebo:\n")
        self.log_text_edit.setStyleSheet("""
            QLabel {
                background-color: #2c3e50;
                color: #ecf0f1;
                padding: 5px;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }
        """)
        self.log_text_edit.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.log_text_edit.setWordWrap(True)
        log_scroll_area.setWidget(self.log_text_edit)

        # Adiciona a área de logs ao layout da área do mapa.
        map_area_layout.addWidget(log_scroll_area)
        map_area_widget.setLayout(map_area_layout)
        main_layout.addWidget(map_area_widget, 4)

    def setup_buttons_area(self, main_layout):
        """
        Configura a área dos botões de controle da simulação Gazebo.

        Args:
            main_layout (QHBoxLayout): O layout principal da janela.
        """
        # Cria o widget e layout para a área dos botões.
        buttons_area_widget = QWidget()
        buttons_area_layout = QVBoxLayout()
        buttons_area_layout.setContentsMargins(0, 0, 0, 0)
        buttons_area_layout.setSpacing(10)

        # Cria e configura o botão 'Iniciar Simulação'.
        start_sim_button = CleanButton("▶ Iniciar Simulação", "#27ae60")
        start_sim_button.clicked.connect(self.start_gazebo_simulation)
        buttons_area_layout.addWidget(start_sim_button)

        # Cria e configura o botão 'Parar Simulação'.
        stop_sim_button = CleanButton("⏹ Parar Simulação", "#e74c3c")
        stop_sim_button.clicked.connect(self.stop_gazebo_simulation)
        buttons_area_layout.addWidget(stop_sim_button)

        # Cria e configura o botão 'Resetar Simulação'.
        reset_sim_button = CleanButton("🔄 Resetar Simulação", "#f39c12")
        reset_sim_button.clicked.connect(self.reset_gazebo_simulation)
        buttons_area_layout.addWidget(reset_sim_button)

        # Adiciona um espaçador para empurrar os botões para cima.
        buttons_area_layout.addStretch()
        buttons_area_widget.setLayout(buttons_area_layout)
        main_layout.addWidget(buttons_area_widget, 1)

    def load_gazebo_parameters(self):
        """
        Carrega parâmetros de configuração do Gazebo a partir de um arquivo YAML.
        Define o caminho do launch file do Gazebo.
        """
        self.gazebo_launch_file = "" # Inicializa o caminho do arquivo de launch do Gazebo.
        try:
            # Constrói o caminho para o arquivo param_gui.yaml.
            params_path = os.path.join(os.path.dirname(__file__), "..", "config", "param_gui.yaml")
            if not os.path.exists(params_path):
                params_path = os.path.join(os.getcwd(), "config", "param_gui.yaml")

            # Se o arquivo existir, carrega os parâmetros.
            if os.path.exists(params_path):
                with open(params_path, 'r') as file:
                    params = yaml.safe_load(file)

                # Extrai o caminho do arquivo de launch do Gazebo se ele estiver presente.
                if 'gazebo_simulation' in params:
                    gazebo_config = params['gazebo_simulation']
                    self.gazebo_launch_file = gazebo_config.get('launch_file', '')

        except Exception as e:
            # Em caso de erro ao carregar os parâmetros, registra mensagem de erro
            gui_log_error("GazeboSimulationWindow", f"Erro ao carregar parâmetros do Gazebo: {e}")

    def start_gazebo_simulation(self):
        """
        Inicia a simulação Gazebo executando o arquivo de launch configurado.
        """
        if self.gazebo_launch_file:
            gui_log_info("GazeboSimulationWindow", f"Iniciando simulação Gazebo com: {self.gazebo_launch_file}")
            try:
                # Executa o comando ros2 launch em um subprocesso
                subprocess.Popen(["ros2", "launch", self.gazebo_launch_file])
                self.log_text_edit.setText(self.log_text_edit.text() + f"\nSimulação Gazebo iniciada: {self.gazebo_launch_file}")
            except Exception as e:
                self.log_text_edit.setText(self.log_text_edit.text() + f"\nErro ao iniciar simulação Gazebo: {e}")
                gui_log_error("GazeboSimulationWindow", f"Erro ao iniciar simulação Gazebo: {e}")
        else:
            self.log_text_edit.setText(self.log_text_edit.text() + "\nCaminho do arquivo de launch do Gazebo não configurado.")
            gui_log_warn("GazeboSimulationWindow", "Caminho do arquivo de launch do Gazebo não configurado")

    def stop_gazebo_simulation(self):
        """
        Para a simulação Gazebo, matando todos os processos relacionados.
        """
        gui_log_info("GazeboSimulationWindow", "Parando simulação Gazebo")
        try:
            # Mata todos os processos relacionados ao Gazebo
            subprocess.run(["killall", "gzserver", "gzclient"], check=True)
            self.log_text_edit.setText(self.log_text_edit.text() + "\nSimulação Gazebo parada.")
        except subprocess.CalledProcessError as e:
            self.log_text_edit.setText(self.log_text_edit.text() + f"\nErro ao parar simulação Gazebo: {e}")
            gui_log_error("GazeboSimulationWindow", f"Erro ao parar simulação Gazebo: {e}")
        except Exception as e:
            self.log_text_edit.setText(self.log_text_edit.text() + f"\nErro inesperado ao parar simulação Gazebo: {e}")
            gui_log_error("GazeboSimulationWindow", f"Erro inesperado ao parar simulação Gazebo: {e}")

    def reset_gazebo_simulation(self):
        """
        Reseta o ambiente da simulação Gazebo.
        """
        gui_log_info("GazeboSimulationWindow", "Resetando simulação Gazebo")
        # Este comando pode variar dependendo de como o Gazebo está configurado
        # Uma abordagem comum é publicar em um tópico de serviço do Gazebo
        # Por simplicidade, aqui apenas registra uma mensagem
        self.log_text_edit.setText(self.log_text_edit.text() + "\nComando de reset de simulação Gazebo enviado (funcionalidade a ser implementada). ")
        gui_log_warn("GazeboSimulationWindow", "Funcionalidade de reset do Gazebo ainda não implementada")

