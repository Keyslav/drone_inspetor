"""Composição visual responsiva; transporte ROS chega apenas pelos sinais Qt.

Os painéis são reposicionados, nunca recriados durante resize: subscriptions,
seleção de missão e instâncias do mapa permanecem as mesmas.
"""

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton, QScrollArea, QSizePolicy
from cv_bridge import CvBridge
from .camera_screen import CameraScreen
from .cv_screen import CVScreen
from .depth_screen import DepthScreen
from .lidar_screen import LidarScreen
from .controles import ControlesManager
from .mission import MissionManager
from .mapa import MapaManager, InteractiveMapWidget
from .widgets.cockpit import Panel, FlightSummary, DASHBOARD_STYLE
from .widgets.images import ResponsiveImageLabel
from .presentation.telemetry import MonitorStore


class DashboardGUI(QWidget):
    """Organiza instrumentos e ações sem publicar comandos ao mudar o layout."""

    def __init__(self, signals, missions_file=None, monitor_store=None):
        super().__init__()
        self.signals, self.missions_file = signals, missions_file
        self.monitor_store = monitor_store if monitor_store is not None else MonitorStore()
        self.monitor_window = None
        self.bridge = CvBridge()
        self.setWindowTitle('Drone Inspetor · Centro de operação')
        self.resize(1440, 940)
        self.setMinimumSize(560, 480)
        self.expanded_windows, self.screen_instances = [], {}
        self.mapa_manager = MapaManager(signals=self.signals.mapa)
        self.mission_manager = MissionManager(signals=self.signals.mission)
        self.controles_manager = ControlesManager(signals=self.signals.control, mapa_signals=self.signals.mapa)
        self._load_missions()
        self.camera_screen = self.cv_screen = self.depth_screen = self.lidar_screen = None
        self._layout_mode = None
        self.setStyleSheet(DASHBOARD_STYLE)
        self.init_ui()
        self.connect_signals()
        self.summary_timer = QTimer(self)
        self.summary_timer.timeout.connect(self.summary.refresh)
        self.summary_timer.start(400)

    def init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(22, 18, 22, 12)
        root.setSpacing(16)
        header = QHBoxLayout()
        titles = QVBoxLayout()
        titles.setSpacing(3)
        brand, heading = QLabel('DRONE INSPETOR'), QLabel('Centro de operação')
        brand.setObjectName('brand')
        heading.setObjectName('heading')
        titles.addWidget(brand)
        titles.addWidget(heading)
        header.addLayout(titles)
        header.addStretch()
        self.monitor_button = QPushButton('Monitor do drone  ↗')
        self.monitor_button.setObjectName('primary')
        self.monitor_button.setToolTip('Abrir telemetria completa · Ctrl+M')
        self.monitor_button.clicked.connect(self.open_monitor)
        header.addWidget(self.monitor_button)
        root.addLayout(header)
        self.monitor_shortcut = QShortcut(QKeySequence('Ctrl+M'), self)
        self.monitor_shortcut.activated.connect(self.open_monitor)

        # A rolagem só entra quando a janela não comporta o tamanho legível dos
        # instrumentos. Não reduzimos texto/controles abaixo desse tamanho.
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        content = QWidget()
        outer = QVBoxLayout(content)
        outer.setContentsMargins(0, 0, 8, 0)
        outer.setSpacing(14)
        self.summary = FlightSummary(self.monitor_store)
        outer.addWidget(self.summary)
        self.workspace = QGridLayout()
        self.workspace.setSpacing(14)
        outer.addLayout(self.workspace, 1)
        self.sensor_area = self.setup_sensor_grid()
        self.control_area = self.setup_control_area()
        self.scroll.setWidget(content)
        root.addWidget(self.scroll, 1)
        footer = QLabel('Duplo clique ou Ampliar para explorar um painel · Estados e comandos confirmados pela telemetria')
        footer.setObjectName('muted')
        footer.setWordWrap(True)
        root.addWidget(footer)
        self._reflow()

    def setup_sensor_grid(self):
        area = QWidget()
        self.sensor_grid = QGridLayout(area)
        self.sensor_grid.setContentsMargins(0, 0, 0, 0)
        self.sensor_grid.setSpacing(12)
        self.video_labels, self.title_labels, self.sensor_panels = [], [], []
        titles = ('Câmera principal', 'Visão computacional', 'Profundidade', 'Mapa da operação')
        subtitles = ('Imagem do sensor', 'Detecções e análise de equipamentos', 'Distância medida pela câmera', 'Posição global · altitude AMSL')
        for index, (title, subtitle) in enumerate(zip(titles, subtitles)):
            panel = Panel(title, subtitle, lambda _=False, i=index: self.expand_sensor(i))
            label = ResponsiveImageLabel('Aguardando imagem do sensor' if index != 3 else '')
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setMinimumSize(200, 150)
            panel.body.addWidget(label, 1)
            panel.setMinimumHeight(270)
            self.sensor_panels.append(panel)
            self.video_labels.append(label)
            self.title_labels.append(panel.title)
        self.camera_screen = CameraScreen(self.signals.camera, self.video_labels[0], self.title_labels[0])
        self.cv_screen = CVScreen(self.signals.cv, self.video_labels[1])
        self.depth_screen = DepthScreen(self.signals.depth, self.video_labels[2])
        for name, screen in (('camera', self.camera_screen), ('cv', self.cv_screen), ('depth', self.depth_screen)):
            self.screen_instances[name] = screen
        for label in self.video_labels:
            label.setMinimumSize(200, 150)
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
            label.setStyleSheet('background: #090f1a; color: #93a4bb; border: none; border-radius: 6px;')
        map_layout = QVBoxLayout(self.video_labels[3])
        map_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_map_widget = InteractiveMapWidget(parent=self.video_labels[3], mapa_manager=self.mapa_manager)
        map_layout.addWidget(self.grid_map_widget)
        self.mapa_manager.map_widget = self.grid_map_widget
        self.screen_instances['mapa'] = self.grid_map_widget
        return area

    def expand_sensor(self, index):
        if index == 3:
            self.expand_map_screen()
        else:
            (self.camera_screen, self.cv_screen, self.depth_screen)[index].expand_screen()

    def setup_control_area(self):
        area = QWidget()
        area.setMinimumWidth(280)
        layout = QVBoxLayout(area)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        self.radar_panel = Panel('Radar de proximidade', 'Referencial do drone · distâncias em metros')
        lidar_host = QLabel()
        lidar_host.setMinimumSize(220, 260)
        self.lidar_screen = LidarScreen(self.signals.lidar, lidar_host)
        self.screen_instances['lidar'] = self.lidar_screen
        self.radar_panel.body.addWidget(lidar_host, 1)
        layout.addWidget(self.radar_panel, 1)
        layout.addWidget(self.mission_manager.setup_b2_mission())
        layout.addWidget(self.controles_manager.setup_b3_controls())
        return area

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'sensor_panels'):
            self._reflow()

    def _reflow(self):
        narrow = self.width() < 1100
        single = self.width() < 720
        mode = (narrow, single)
        if mode == self._layout_mode:
            return
        self._layout_mode = mode
        # Retirar itens do layout não destrói widgets nem suas conexões.
        for widget in (self.sensor_area, self.control_area):
            self.workspace.removeWidget(widget)
        self.workspace.addWidget(self.sensor_area, 0, 0)
        self.workspace.addWidget(self.control_area, 1 if narrow else 0, 0 if narrow else 1)
        self.workspace.setColumnStretch(0, 3)
        self.workspace.setColumnStretch(1, 0 if narrow else 1)
        self.workspace.setRowStretch(0, 1)
        self.workspace.setRowStretch(1, 0)
        columns = 1 if single else 2
        for index, panel in enumerate(self.sensor_panels):
            self.sensor_grid.removeWidget(panel)
            self.sensor_grid.addWidget(panel, index // columns, index % columns)
        for column in range(2):
            self.sensor_grid.setColumnStretch(column, 1 if column < columns else 0)
        for row in range(4):
            self.sensor_grid.setRowStretch(row, 1 if row < (4 // columns) else 0)
        self.summary.reflow(single)

    def _load_missions(self):
        """Compartilha a validação do catálogo com o nó de missão."""
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        from drone_inspetor.missions.repository import MissionRepository
        from .logging import gui_log_error, gui_log_info

        try:
            path = self.missions_file or (
                Path(get_package_share_directory('drone_inspetor'))
                / 'missions' / 'missions.json'
            )
            repository = MissionRepository(path)
            repository.load()
            self.loaded_missions = repository.as_mapping()
            gui_log_info('DashboardGUI', f'Missões carregadas: {list(self.loaded_missions)}')
        except (OSError, ValueError) as exc:
            self.loaded_missions = {}
            gui_log_error('DashboardGUI', f'Erro ao carregar missões: {exc}')
        self.mapa_manager.set_missions(self.loaded_missions)
        self.controles_manager.set_missions(self.loaded_missions)

    def closeEvent(self, event):
        """Fecha janelas auxiliares para que o processo Qt termine por completo."""
        for screen in (self.camera_screen, self.cv_screen, self.depth_screen):
            if screen is not None:
                screen.close()
        for window in self.expanded_windows:
            window.close()
        if self.monitor_window is not None:
            self.monitor_window.close()
        if self.mapa_manager.expanded_window is not None:
            self.mapa_manager.expanded_window.close()
        simulation_window = getattr(self.controles_manager, 'gazebo_window', None)
        if simulation_window is not None:
            simulation_window.close()
        super().closeEvent(event)

    def open_monitor(self):
        """Abre uma única janela auxiliar e preserva a seleção ao reabrir."""
        from .monitor_screen import MonitorWindow
        if self.monitor_window is None:
            self.monitor_window = MonitorWindow(self.monitor_store, parent=self)
        self.monitor_window.show()
        self.monitor_window.raise_()
        self.monitor_window.activateWindow()

    def connect_signals(self):
        """
        Conecta os sinais PyQt6 (via self.signals) aos slots apropriados na GUI.
        Isso permite que a GUI reaja a eventos e atualizações de dados provenientes do sistema ROS2.
        """
        # Conexões para Câmera: Atualiza o feed da câmera principal.
        self.signals.camera.image_received.connect(self.camera_image_update)
        self.signals.camera.recording_status_received.connect(self.handle_recording_indicator)

        # Conexões para CV: Atualiza a imagem processada, dados de análise e detecções.
        self.signals.cv.image_received.connect(self.cv_image_update)
        self.signals.cv.analysis_data_received.connect(self.cv_analysis_data_update)
        self.signals.cv.detections_received.connect(self.cv_detections_update)

        # Conexões para Profundidade: Atualiza a imagem de profundidade, estatísticas e alertas de proximidade.
        self.signals.depth.image_received.connect(self.depth_image_update)
        self.signals.depth.statistics_received.connect(self.depth_statistics_update)
        self.signals.depth.proximity_alert_received.connect(self.proximity_alert_update)

        # Conexões para LiDAR: Atualiza os dados consolidados do LiDAR
        self.signals.lidar.lidar_data_received.connect(self.lidar_data_update)
        self.signals.lidar.statistics_received.connect(self.lidar_statistics_update)
        self.signals.lidar.obstacle_detections_received.connect(self.lidar_obstacle_detections_update)

        # Conexões para Missão: Atualiza o estado da Máquina de Estados de Missão.
        self.signals.mission.mission_state_updated.connect(self.mission_manager.update_state)
        self.signals.mission.mission_state_updated.connect(self.handle_mission_state_for_map)

        # Conexão única para estado do drone no mapa
        # O sinal drone_state_updated contém todos os campos de DroneStateMSG
        self.signals.mapa.drone_state_updated.connect(self.mapa_manager.update_drone_state)
        
        # Conexões para missão no mapa
        self.signals.mapa.mission_selected.connect(self.mapa_manager.display_mission_on_map)
        self.signals.mapa.mission_started.connect(self.handle_mission_started)
        self.signals.mapa.mission_ended.connect(self.handle_mission_ended)

    def camera_image_update(self, cv_image):
        """
        Atualiza o feed da câmera principal na GUI.
        """
        try:
            self.camera_screen.update_camera_feed(cv_image)
        except Exception as e:
            from .utils import gui_log_error
            gui_log_error("DashboardGUI", f"Erro ao processar imagem da câmera: {e}")

    def cv_image_update(self, cv_image):
        """
        Atualiza a imagem processada por Visão Computacional na GUI.
        Recebe diretamente uma imagem OpenCV (numpy array) do subscriber.
        """
        try:
            self.cv_screen.update_processed_image(cv_image)
        except Exception as e:
            from .utils import gui_log_error
            gui_log_error("DashboardGUI", f"Erro ao processar imagem CV: {e}")

    def cv_analysis_data_update(self, data):
        """
        Atualiza os dados de análise de Visão Computacional na GUI.
        """
        self.cv_screen.update_analysis_data(data)

    def cv_detections_update(self, detections):
        """
        Atualiza as detecções de objetos de Visão Computacional na GUI.
        """
        self.cv_screen.update_detections(detections)

    def depth_image_update(self, msg):
        """
        Atualiza a imagem da câmera de profundidade na GUI.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.depth_screen.update_depth_image(cv_image)
        except Exception as e:
            from .utils import gui_log_error
            gui_log_error("DashboardGUI", f"Erro ao processar imagem de profundidade: {e}")

    def depth_statistics_update(self, statistics):
        """
        Atualiza as estatísticas da câmera de profundidade na GUI.
        """
        self.depth_screen.update_depth_statistics(statistics)

    def proximity_alert_update(self, alerts):
        """
        Atualiza os alertas de proximidade da câmera de profundidade na GUI.
        """
        self.depth_screen.update_proximity_alerts(alerts)

    def lidar_data_update(self, lidar_data: dict):
        """
        Atualiza os dados consolidados do LiDAR na GUI.
        
        Args:
            lidar_data (dict): Dicionário com 'point_vector' e 'ground_distance'
        """
        # Atualiza o vetor de pontos
        if 'point_vector' in lidar_data:
            self.lidar_screen.update_point_vector(lidar_data['point_vector'])
        
        # Atualiza a distância inferior
        if 'ground_distance' in lidar_data:
            self.lidar_screen.update_ground_distance(lidar_data['ground_distance'])

    def lidar_statistics_update(self, statistics):
        """
        Atualiza as estatísticas do LiDAR na GUI.
        """
        self.lidar_screen.update_lidar_statistics(statistics)

    def lidar_obstacle_detections_update(self, detections):
        """
        Atualiza as detecções de obstáculos do LiDAR na GUI.
        """
        self.lidar_screen.update_obstacle_detections(detections)

    def handle_mission_state_for_map(self, state_data: dict):
        """
        Processa o estado da missão para exibir/limpar pontos de inspeção no mapa.

        Args:
            state_data (dict): Dados do estado de missão contendo 'on_mission' e 'mission_name'.
        """
        on_mission = state_data.get("on_mission", False)
        mission_name = state_data.get("mission_name", "")
        
        # Armazena o estado anterior para detectar mudança
        if not hasattr(self, '_last_on_mission_state'):
            self._last_on_mission_state = False
        
        # Se entrou em missão, exibe os pontos
        if on_mission and not self._last_on_mission_state:
            if mission_name and mission_name in self.loaded_missions:
                self.mapa_manager.display_mission_on_map(mission_name)
                from .utils import gui_log_info
                gui_log_info("DashboardGUI", f"Exibindo pontos da missão '{mission_name}' no mapa")
        
        # Se saiu da missão, limpa os marcadores
        elif not on_mission and self._last_on_mission_state:
            self.mapa_manager.clear_mission_display()
            from .utils import gui_log_info
            gui_log_info("DashboardGUI", "Marcadores de missão limpos do mapa")
        
        self._last_on_mission_state = on_mission

    def handle_recording_indicator(self, is_recording: bool):
        """
        Atualiza o indicador de gravação na tela da câmera.
        Chamado quando o camera_node publica o status de gravação.
        
        Args:
            is_recording (bool): True se está gravando, False caso contrário.
        """
        if self.camera_screen:
            self.camera_screen.set_recording_indicator(is_recording)

    def handle_mission_started(self, mission_name: str):
        """
        Handler para quando uma missão é iniciada.
        Exibe os pontos de inspeção no mapa.
        
        Args:
            mission_name (str): Nome da missão iniciada.
        """
        if mission_name and mission_name in self.loaded_missions:
            self.mapa_manager.display_mission_on_map(mission_name)

    def handle_mission_ended(self):
        """
        Handler para quando uma missão é finalizada.
        Limpa os marcadores de missão do mapa.
        """
        self.mapa_manager.clear_mission_display()

    def expand_map_screen(self, event=None):
        """
        Abre o mapa em uma janela expandida ao dar duplo clique no título.
        Limita a uma janela expandida.
        
        Args:
            event: Evento de duplo clique do mouse
        """
        from .utils import ExpandedWindow, gui_log_info
        from .mapa import InteractiveMapWidget
        
        # Limpa janelas fechadas da lista
        self.expanded_windows = [w for w in self.expanded_windows if w.isVisible()]
        
        # Se já existe janela de mapa expandida, foca nela
        for window in self.expanded_windows:
            if "Mapa" in window.windowTitle():
                window.raise_()
                window.activateWindow()
                gui_log_info("DashboardGUI", "Focando janela existente do Mapa GPS")
                return
        
        gui_log_info("DashboardGUI", "Expandindo tela do Mapa GPS")
        
        # Cria um novo widget de mapa para a janela expandida
        expanded_map = InteractiveMapWidget(parent=None, mapa_manager=self.mapa_manager)
        expanded_map.setMinimumSize(480, 320)
        
        # Cria e exibe a janela expandida
        window = ExpandedWindow("Mapa GPS", expanded_map, None)
        self.expanded_windows.append(window)
        window.show()

