"""
controles.py
=================================================================================================
Gerenciador de controles da missão e simulação Gazebo.

Gerencia os controles da missão e a interface de simulação Gazebo. Esta classe é um
componente da interface gráfica (GUI) e interage com o nó ROS2 do dashboard para enviar
comandos de missão. Mantém referência ao node apenas para publicação de comandos.
=================================================================================================
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QComboBox, QGridLayout, QScrollArea, QMainWindow, QSizePolicy)
from PyQt6.QtGui import QCursor
from PyQt6.QtCore import Qt
from std_msgs.msg import String
import os
import subprocess
import yaml
import json
from .utils import gui_log_info, gui_log_error, gui_log_warn
from ament_index_python.packages import get_package_share_directory

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
        """
        Recebe as missões carregadas pelo dashboard_gui.
        
        Args:
            missions (dict): Dicionário com todas as missões disponíveis.
        """
        self.missions = missions
        gui_log_info("ControlesManager", f"Missões recebidas: {list(missions.keys())}")
        
        # Atualiza o dropdown de seleção de missão
        if self.inspection_selector:
            self.inspection_selector.clear()
            self.inspection_selector.addItems(list(missions.keys()))

    def setup_b3_controls(self):
        """
        Configura o painel B3 com os controles da missão e o seletor de tipo de inspeção.
        Este painel será integrado ao layout principal do dashboard.

        Returns:
            QWidget: O widget contendo todos os controles configurados.
        """
        # Registra o início da configuração do painel B3
        gui_log_info("ControlesManager", "Configurando painel B3 - Controles e seletor")
        
        # Cria o widget principal para o painel B3 e seu layout vertical.
        b3_widget = QWidget()
        b3_layout = QVBoxLayout()
        
        # Define as margens e espaçamento do layout.
        b3_layout.setContentsMargins(2, 2, 2, 2)
        b3_layout.setSpacing(0)
        
        # Cria o QLabel para o título dos controles e configura seu estilo.
        control_title = QLabel("Controles")
        control_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        control_title.setMaximumHeight(26)
        control_title.setStyleSheet("""
            font-weight: bold; 
            color: #ecf0f1; 
            background-color: #A93226; 
            padding: 3px; 
            border-radius: 3px;
            font-size: 13px;
            border: 1px solid #e74c3c;
            margin-bottom: 6px;
        """)
        b3_layout.addWidget(control_title, 0, Qt.AlignmentFlag.AlignTop)
        
        # Cria um layout horizontal para o seletor de missão.
        selector_layout = QHBoxLayout()
        selector_layout.setContentsMargins(0, 0, 0, 0)
        selector_layout.setSpacing(2)
        
        # Cria o QLabel para o rótulo do seletor.
        selector_label = QLabel("Selecione a Missão:")
        selector_label.setStyleSheet("""
            font-size: 12px;
            font-weight: bold;
            color: #ecf0f1;
            background: transparent;
            border: none;
        """)
        selector_label.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        selector_layout.addWidget(selector_label)
        
        # Cria o QComboBox para o seletor de missão e adiciona os nomes das missões carregadas.
        self.inspection_selector = QComboBox()
        self.inspection_selector.addItems(list(self.missions.keys()))
        self.inspection_selector.setMaximumHeight(33)
        self.inspection_selector.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.inspection_selector.setStyleSheet("""
            QComboBox {
                background-color: #34495e;
                color: #ecf0f1;
                border: 1px solid #7f8c8d;
                border-radius: 3px;
                padding: 4px;
                font-size: 12px;
                font-weight: bold;
            }
        """)
        selector_layout.addWidget(self.inspection_selector)
        
        # Conecta o dropdown para emitir sinal quando missão for selecionada
        self.inspection_selector.currentTextChanged.connect(self._on_mission_selected)
        
        # Emite sinal para a seleção inicial (para carregar pontos no mapa ao iniciar)
        if self.inspection_selector.count() > 0:
            self._on_mission_selected(self.inspection_selector.currentText())
        
        # Adiciona o layout do seletor ao layout principal do painel B3.
        b3_layout.addLayout(selector_layout)
        
        # Configura os botões de controle da missão.
        self.setup_control_buttons(b3_layout)
        
        # Define o layout para o widget principal do painel B3 e o retorna.
        b3_widget.setLayout(b3_layout)
        return b3_widget
    
    def _on_mission_selected(self, mission_name: str):
        """
        Callback chamado quando uma missão é selecionada no dropdown.
        Emite o sinal mission_selected para que o mapa exiba os pontos.
        
        Args:
            mission_name (str): Nome da missão selecionada.
        """
        if mission_name and self.mapa_signals:
            gui_log_info("ControlesManager", f"Missão selecionada: {mission_name}")
            self.mapa_signals.mission_selected.emit(mission_name)

    def setup_control_buttons(self, layout):
        """
        Configura os botões de controle da missão e os adiciona ao layout fornecido.
        
        Botões disponíveis:
        - Iniciar Missão: Envia INICIAR_MISSAO com nome da missão selecionada
        - Cancelar Missão: Envia CANCELAR_MISSAO (drone retorna automaticamente)
        - Log de Análise: Abre janela de análise de logs

        Args:
            layout (QLayout): O layout onde os botões serão adicionados.
        """
        # Registra o início da configuração dos botões
        gui_log_info("ControlesManager", "Configurando botões de controle de missão")
        
        # Cria um layout de grade para organizar os botões.
        buttons_layout = QGridLayout()
        buttons_layout.setSpacing(5)
        
        # Cria e configura o botão 'Iniciar Missão'.
        self.start_button = CleanButton("▶ Iniciar Missão", "#27ae60")
        self.start_button.clicked.connect(self.iniciar_missao)
        buttons_layout.addWidget(self.start_button, 0, 0)

        # Cria e configura o botão 'Cancelar Missão'.
        self.cancel_button = CleanButton("⏹ Cancelar Missão", "#e74c3c")
        self.cancel_button.clicked.connect(self.cancelar_missao)
        buttons_layout.addWidget(self.cancel_button, 0, 1)

        # Cria e configura o botão 'Log de Análise'.
        self.log_button = CleanButton("📊 Log de Análise", "#17a2b8")
        self.log_button.clicked.connect(self.open_log_analysis)
        buttons_layout.addWidget(self.log_button, 1, 0, 1, 2)
        
        # Adiciona o layout dos botões ao layout fornecido.
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
    """
    Classe customizada para botões com um design limpo e moderno.
    Herda de QPushButton e aplica estilos CSS customizados para uma aparência consistente.
    """
    def __init__(self, text, icon_color, parent=None):
        """
        Inicializa um botão customizado.

        Args:
            text (str): O texto a ser exibido no botão (pode incluir emojis).
            icon_color (str): A cor hexadecimal para o texto/ícone do botão.
            parent (QWidget, optional): O widget pai deste botão. Padrão para None.
        """
        super().__init__(text, parent)
        self.icon_color = icon_color
        
        # Define a altura máxima e mínima do botão.
        self.setMaximumHeight(50)
        self.setMinimumHeight(50)
        
        # Define o estilo CSS para o botão.
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: #5a6c7d;
                color: {icon_color};
                border: 2px solid #7f8c8d;
                padding: 8px;
                border-radius: 5px;
                font-size: 18px;
                font-weight: bold;
                margin: 2px;
                text-align: center;
                border-style: outset;
                border-width: 3px;
            }}
            QPushButton:hover {{
                background-color: #6c7b8a;
                border-color: #95a5a6;
                border-style: outset;
            }}
            QPushButton:pressed {{
                background-color: #4a5568;
                border-style: inset;
                border-width: 2px;
            }}
            QPushButton:disabled {{
                background-color: #3a4a5a;
                color: #7f8c8d;
                border-color: #5a6c7d;
            }}
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


