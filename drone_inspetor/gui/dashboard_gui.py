"""
dashboard_gui.py
=================================================================================================
Interface gráfica principal do Dashboard do Drone Inspetor.

Este módulo implementa a interface gráfica PyQt6 do dashboard, fornecendo visualização
em tempo real dos dados dos sensores, controle da missão, e monitoramento do estado
do sistema. A GUI é organizada em uma arquitetura modular com telas dedicadas para
cada sensor e gerenciadores para funcionalidades específicas.

ARQUITETURA:
- Layout principal dividido em duas áreas: sensores (grid 2x2) e controles (mapa, FSM, controles)
- Comunicação com ROS2 através de sinais PyQt6 e interface de comandos
- Telas modulares para cada sensor (CameraScreen, CVScreen, DepthScreen, LidarScreen)
- Gerenciadores modulares para funcionalidades (MapaManager, FSMManager, ControlesManager)
- Tema escuro personalizado para melhor experiência visual
- Separação completa entre GUI e nó ROS2 através de sinais e interface de comandos

COMPONENTES:
- Grid 2x2 de telas de sensores (câmera, CV, profundidade, LiDAR)
- Área de controles com mapa GPS interativo
- Visualização hierárquica da máquina de estados finitos (FSM)
- Painel de controles de missão e simulação
=================================================================================================
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QGridLayout, QSplitter, QFrame)
from PyQt6.QtGui import QPixmap, QImage, QCursor
from PyQt6.QtCore import Qt

import cv2
from cv_bridge import CvBridge

# Importações das telas de sensores
# Cada tela gerencia a exibição de dados de um sensor específico
from .camera_screen import CameraScreen
from .cv_screen import CVScreen
from .depth_screen import DepthScreen
from .lidar_screen import LidarScreen

# Importações de utilitários e gerenciadores
# Utilitários: estilos comuns, tópicos ROS
from .utils import COMMON_STYLES, ROS_TOPICS
# Gerenciadores: controlam funcionalidades específicas do dashboard
from .controles import ControlesManager
from .fsm import FSMManager
from .mapa import MapaManager

# Importação de sinais PyQt6 para comunicação assíncrona com ROS2
from drone_inspetor.signals.dashboard_signals import DashboardSignals


class DashboardGUI(QWidget):
    """
    Classe principal da interface gráfica (GUI) do Dashboard do Drone Inspetor.
    
    Esta classe gerencia toda a interface gráfica do dashboard, incluindo layout,
    tema visual, inicialização de componentes, e conexão de sinais PyQt6 para
    comunicação assíncrona com o nó ROS2.
    
    Responsabilidades:
    - Gerenciar o layout geral da interface (grid de sensores + área de controles)
    - Aplicar tema visual consistente em todos os componentes
    - Inicializar e gerenciar telas de sensores e gerenciadores de funcionalidades
    - Conectar sinais PyQt6 aos slots apropriados para atualização da interface
    - Gerenciar janelas expandidas para visualização em tela cheia
    """
    
    def __init__(self, signals: DashboardSignals):
        """
        Inicializa a janela principal do Dashboard.

        Args:
            signals (DashboardSignals): Instância dos sinais PyQt6 para comunicação assíncrona com ROS2
                                       Os sinais já contêm métodos de publicação de comandos incorporados
        """
        super().__init__()
        
        # ==================== ARMAZENAMENTO DE REFERÊNCIAS ============================================
        # Armazena referência aos sinais PyQt6 para comunicação assíncrona com ROS2
        # Os sinais permitem comunicação assíncrona entre threads (ROS2 e GUI)
        # Os sinais também contêm métodos de publicação de comandos incorporados
        self.signals = signals
        
        # Cria instância do CvBridge para conversão entre formatos ROS e OpenCV/PyQt
        # Necessário para processar imagens recebidas via ROS2 e exibi-las na GUI
        self.bridge = CvBridge()

        # ==================== CONFIGURAÇÕES BÁSICAS DA JANELA =========================================
        # Define título da janela principal
        self.setWindowTitle("Drone Inspetor Dashboard - Arquitetura Modular")
        
        # Define geometria inicial da janela (posição x, y, largura, altura)
        self.setGeometry(100, 100, 1920, 1080)
        
        # Lista para manter referências a janelas expandidas
        # Permite gerenciar múltiplas janelas de visualização em tela cheia
        self.expanded_windows = []
        
        # ==================== INICIALIZAÇÃO DOS GERENCIADORES =========================================
        # Gerenciadores controlam funcionalidades específicas do dashboard
        # Cada gerenciador recebe sinais específicos e, quando necessário, a interface de comandos
        
        # Gerenciador do mapa GPS interativo
        self.mapa_manager = MapaManager(signals=self.signals.mapa)
        
        # Gerenciador da máquina de estados finitos (FSM)
        self.fsm_manager = FSMManager(signals=self.signals.fsm)
        
        # Gerenciador de controles de missão e simulação
        # Os sinais já contêm métodos de publicação de comandos incorporados
        self.controles_manager = ControlesManager(signals=self.signals.control)
        
        # ==================== INICIALIZAÇÃO DAS TELAS DE SENSORES =====================================
        # As telas de sensores são inicializadas como None aqui e serão criadas após
        # a criação dos QLabels no setup_sensor_grid(). Isso garante que os QLabels
        # estejam disponíveis quando as telas forem inicializadas.
        
        # Tela da câmera principal (imagens raw)
        self.camera_screen = None
        
        # Tela de visão computacional (imagens processadas com detecções)
        self.cv_screen = None
        
        # Tela de câmera de profundidade (imagens de profundidade processadas)
        self.depth_screen = None
        
        # Tela do LiDAR (visualização de varredura laser)
        self.lidar_screen = None
        
        # Dicionário para armazenar referências às instâncias das telas
        # Facilita acesso programático às telas por nome
        self.screen_instances = {}
        
        # ==================== CONFIGURAÇÃO DA INTERFACE ===============================================
        # Aplica tema escuro personalizado a todos os widgets
        self.apply_dark_theme()
        
        # Configura o layout e organiza todos os widgets na interface
        self.init_ui()
        
        # Conecta sinais PyQt6 aos slots da GUI
        # Isso permite que a GUI seja atualizada automaticamente quando novos dados chegam
        self.connect_signals()
    
    def apply_dark_theme(self):
        """
        Aplica um tema escuro personalizado a todos os widgets do Dashboard.
        Define cores de fundo, texto, bordas e estilos para diferentes elementos da GUI,
        garantindo uma aparência coesa e moderna.
        """
        dark_style = f"""
            QWidget {{
                background-color: {COMMON_STYLES["dark_background"]};
                color: {COMMON_STYLES["text_color"]};
                font-family: 'Segoe UI', Arial, sans-serif;
            }}
            
            QLabel {{
                color: {COMMON_STYLES["text_color"]};
            }}
            
            QComboBox {{
                background-color: {COMMON_STYLES["light_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid {COMMON_STYLES["border_color"]};
                border-radius: 5px;
                padding: 6px;
                font-weight: bold;
            }}
            
            QComboBox::drop-down {{
                border: none;
                background-color: {COMMON_STYLES["border_color"]};
            }}
            
            QComboBox::down-arrow {{
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid {COMMON_STYLES["text_color"]};
            }}
            
            QFrame {{
                background-color: {COMMON_STYLES["border_color"]};
            }}
            
            QPushButton {{
                background-color: {COMMON_STYLES["accent_color"]};
                color: {COMMON_STYLES["text_color"]};
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }}
            
            QPushButton:hover {{
                background-color: {COMMON_STYLES["success_color"]};
            }}
        """
        self.setStyleSheet(dark_style)
    
    def init_ui(self):
        """
        Configura o layout principal da interface do usuário do Dashboard.
        Divide a janela em duas áreas principais: sensores e controles,
        utilizando um QSplitter para permitir redimensionamento dinâmico.
        """
        # print("Configurando layout principal modular") # Comentado para evitar logs desnecessários.
        
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # QSplitter permite redimensionar as áreas A e B dinamicamente.
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Área A: Grid de telas de sensores (Câmera, CV, Profundidade, LiDAR).
        area_a = self.setup_sensor_grid()
        main_splitter.addWidget(area_a)
        
        # Área B: Controles, FSM e Mapa.
        area_b = self.setup_control_area()
        main_splitter.addWidget(area_b)
        
        # Define tamanhos iniciais e fatores de estiramento para as áreas.
        # Exemplo de proporção 4:1 para as áreas A e B, respectivamente.
        main_splitter.setSizes([1536, 384]) 
        main_splitter.setStretchFactor(0, 4)
        main_splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(main_splitter)
        self.setLayout(main_layout)
        
        # print("Layout principal configurado") # Comentado para evitar logs desnecessários.
    
    def setup_sensor_grid(self):
        """
        Configura o grid 2x2 para as telas dos sensores (Câmera, CV, Profundidade, LiDAR).
        Cada célula do grid contém um QLabel para exibir o feed do sensor e um título,
        proporcionando uma visualização organizada dos dados.
        """
        # print("Configurando grid de sensores 2x2") # Comentado para evitar logs desnecessários.
        
        sensor_widget = QWidget()
        sensor_layout = QGridLayout()
        sensor_layout.setSpacing(5) # Espaçamento entre as células do grid.
        sensor_layout.setContentsMargins(5, 5, 5, 5) # Margens internas do grid.
        
        # Configurações para cada tela de sensor, incluindo nome, posição no grid e cor de destaque.
        screen_configs = [
            {"name": "Câmera Principal", "pos": (0, 0), "color": COMMON_STYLES["accent_color"]},
            {"name": "Câmera Processada (CV)", "pos": (0, 1), "color": COMMON_STYLES["accent_color"]},
            {"name": "Câmera de Profundidade", "pos": (1, 0), "color": "#9b59b6"},
            {"name": "LiDAR", "pos": (1, 1), "color": "#e67e22"}
        ]
        
        self.video_labels = [] # Lista para armazenar os QLabels que exibem os vídeos/imagens.
        
        for i, config in enumerate(screen_configs):
            row, col = config["pos"]
            
            # Container para cada tela de sensor, com layout vertical para título e vídeo.
            screen_container = QWidget()
            screen_layout = QVBoxLayout()
            screen_layout.setContentsMargins(2, 2, 2, 2)
            screen_layout.setSpacing(0)
            
            # Título da tela do sensor.
            title_label = QLabel(config["name"])
            title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title_label.setMaximumHeight(36)
            title_label.setMinimumHeight(36)
            title_label.setStyleSheet(f"""
                font-weight: bold; 
                color: {COMMON_STYLES["text_color"]};
                background-color: {config["color"]};
                padding: 4px; 
                border-radius: 3px;
                font-size: 13px;
                border: 1px solid {COMMON_STYLES["border_color"]};
                margin-bottom: 6px;
            """)
            screen_layout.addWidget(title_label, 0, Qt.AlignmentFlag.AlignTop)
            
            # QLabel para exibir o feed de vídeo/imagem do sensor.
            video_label = QLabel()
            video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            video_label.setStyleSheet(f"""
                background-color: {COMMON_STYLES["dark_background"]};
                color: {COMMON_STYLES["text_color"]};
                border: 2px solid {config["color"]};
                border-radius: 5px;
                font-size: 16px;
            """)
            video_label.setText(f"Aguardando {config["name"]}...")
            video_label.setCursor(QCursor(Qt.CursorShape.PointingHandCursor)) # Cursor de mão para indicar clicável.
            
            self.video_labels.append(video_label)
            screen_layout.addWidget(video_label, 1)
            
            screen_container.setLayout(screen_layout)
            sensor_layout.addWidget(screen_container, row, col)
            
            # print(f"Tela {config["name"]} configurada na posição ({row}, {col})") # Comentado para evitar logs desnecessários.
        
        self.init_screen_modules() # Inicializa as instâncias das classes de tela.
        
        sensor_widget.setLayout(sensor_layout)
        return sensor_widget
    
    def init_screen_modules(self):
        """
        Inicializa as instâncias das classes de tela para cada sensor.
        Cada instância de tela (e.g., CameraScreen) é um widget PyQt puro
        que gerencia a exibição de dados em seu respectivo QLabel.
        
        Este método agora apenas inicializa as instâncias que foram declaradas no __init__()
        com os QLabels correspondentes criados no setup_sensor_grid().
        """
        # print("Inicializando módulos de tela (apenas GUI)") # Comentado para evitar logs desnecessários.
        
        # As instâncias das telas são criadas e associadas aos seus QLabels correspondentes.
        # Elas recebem apenas os sinais PyQt6 correspondentes, mantendo separação com ROS2.
        # Nota: As variáveis self.camera_screen, self.cv_screen, etc. foram declaradas no __init__()
        # e agora são inicializadas aqui com os QLabels correspondentes.
        self.camera_screen = CameraScreen(self.signals.camera, self.video_labels[0])
        self.screen_instances["camera"] = self.camera_screen
        
        self.cv_screen = CVScreen(self.signals.cv, self.video_labels[1])
        self.screen_instances["cv"] = self.cv_screen
        
        self.depth_screen = DepthScreen(self.signals.depth, self.video_labels[2])
        self.screen_instances["depth"] = self.depth_screen
        
        self.lidar_screen = LidarScreen(self.signals.lidar, self.video_labels[3])
        self.screen_instances["lidar"] = self.lidar_screen

    def connect_signals(self):
        """
        Conecta os sinais PyQt6 (via self.signals) aos slots apropriados na GUI.
        Isso permite que a GUI reaja a eventos e atualizações de dados provenientes do sistema ROS2.
        """
        # Conexões para Câmera: Atualiza o feed da câmera principal.
        self.signals.camera.image_received.connect(self.camera_image_update)

        # Conexões para CV: Atualiza a imagem processada, dados de análise e detecções.
        self.signals.cv.image_received.connect(self.cv_image_update)
        self.signals.cv.analysis_data_received.connect(self.cv_analysis_data_update)
        self.signals.cv.detections_received.connect(self.cv_detections_update)

        # Conexões para Profundidade: Atualiza a imagem de profundidade, estatísticas e alertas de proximidade.
        self.signals.depth.image_received.connect(self.depth_image_update)
        self.signals.depth.statistics_received.connect(self.depth_statistics_update)
        self.signals.depth.proximity_alert_received.connect(self.proximity_alert_update)

        # Conexões para LiDAR: Atualiza o vetor de pontos, estatísticas e detecções de obstáculos.
        self.signals.lidar.point_vector_received.connect(self.lidar_point_vector_update)
        self.signals.lidar.statistics_received.connect(self.lidar_statistics_update)
        self.signals.lidar.obstacle_detections_received.connect(self.lidar_obstacle_detections_update)

        # Conexões para FSM: Atualiza o estado da Máquina de Estados Finitos.
        self.signals.fsm.state_received.connect(self.fsm_manager.update_state)

        # Conexões para Controle (posição/atitude): Atualiza a posição e atitude do drone no mapa.
        self.signals.control.global_position_received.connect(self.mapa_manager.update_position_from_node)
        self.signals.control.attitude_received.connect(self.mapa_manager.update_attitude_from_node)

        # Conexões para Mapa (se houver sinais específicos do mapa para a GUI):
        # Estes sinais garantem que o mapa seja atualizado quando a posição ou atitude do drone muda.
        self.signals.mapa.position_updated.connect(self.mapa_manager.update_position_from_node)
        self.signals.mapa.attitude_updated.connect(self.mapa_manager.update_attitude_from_node)

    # Métodos de atualização da GUI (slots) que serão conectados aos sinais.
    def camera_image_update(self, cv_image):
        """
        Atualiza o feed da câmera principal na GUI.
        """
        try:
            self.camera_screen.update_camera_feed(cv_image)
        except Exception as e:
            from .utils import gui_log_error
            gui_log_error("DashboardGUI", f"Erro ao processar imagem da câmera: {e}")

    def cv_image_update(self, msg):
        """
        Atualiza a imagem processada por Visão Computacional na GUI.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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

    def lidar_point_vector_update(self, point_vector_list):
        """
        Atualiza o vetor de pontos do LiDAR na GUI.
        """
        self.lidar_screen.update_point_vector(point_vector_list)

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

    def setup_control_area(self):
        """
        Configura a área de controles, FSM e mapa com proporções definidas e separadores.
        """
        control_area_widget = QWidget()
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(5, 5, 5, 5)
        control_layout.setSpacing(5) # Espaço entre os widgets

        # --- Cria os painéis ---
        map_panel = self.mapa_manager.setup_b1_map()
        fsm_panel = self.fsm_manager.setup_b2_fsm()
        controls_panel = self.controles_manager.setup_b3_controls()

        # --- Adiciona os painéis ao layout ---
        control_layout.addWidget(map_panel)
        
        # --- Adiciona a primeira linha separadora ---
        separator1 = QFrame()
        separator1.setFrameShape(QFrame.Shape.HLine) # Linha horizontal
        separator1.setFrameShadow(QFrame.Shadow.Sunken)
        separator1.setStyleSheet(f"background-color: {COMMON_STYLES['border_color']};") # Usa a cor da borda do tema
        control_layout.addWidget(separator1)

        control_layout.addWidget(fsm_panel)

        # --- Adiciona a segunda linha separadora ---
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.Shape.HLine)
        separator2.setFrameShadow(QFrame.Shadow.Sunken)
        separator2.setStyleSheet(f"background-color: {COMMON_STYLES['border_color']};")
        control_layout.addWidget(controls_panel)

        # --- Define as proporções de espaço (Stretch Factors) ---
        # Esta é a parte mais importante. A soma total é 10 (6+2+2).
        # O mapa receberá 6/10 (60%) do espaço vertical.
        # O FSM e os Controles receberão 2/10 (20%) cada.
        # Os separadores e o stretch final terão um fator de 0, ou seja, não crescerão.
        
        control_layout.setStretch(0, 4)  # Índice 0 (map_panel) recebe 40% do espaço
        # O índice 1 é o separator1, não precisa de stretch
        control_layout.setStretch(2, 3)  # Índice 2 (fsm_panel) recebe 30% do espaço
        # O índice 3 é o separator2, não precisa de stretch
        control_layout.setStretch(4, 3)  # Índice 4 (controls_panel) recebe 30% do espaço

        # Adiciona um espaçador no final para garantir que tudo fique alinhado ao topo
        # se houver espaço sobrando, mas com os stretches definidos, isso é menos crítico.
        control_layout.addStretch(1)
        control_layout.setStretch(control_layout.count() - 1, 0) # Garante que o stretch final não ocupe espaço

        control_area_widget.setLayout(control_layout)
        return control_area_widget


