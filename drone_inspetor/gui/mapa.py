"""
mapa.py
=================================================================================================
Gerenciador do sistema de mapas interativos para monitoramento do drone.

Gerencia o sistema de mapas interativos para monitoramento do drone. Esta classe coordena
a interface do mapa, mantém os dados de posição do drone e gerencia a comunicação com o
widget de mapa interativo. Não requer acesso ao node ROS2.
=================================================================================================
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QMainWindow)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtGui import QCursor
from PyQt6.QtCore import Qt, QTimer, QUrl
import os
import yaml
from .utils import gui_log_info, gui_log_error

class MapaManager:
    """
    Gerencia o sistema de mapas interativos para monitoramento do drone.
    
    Esta classe coordena a interface do mapa, mantém os dados de posição do drone
    e gerencia a comunicação com o widget de mapa interativo.
    """
    
    def __init__(self, signals=None):
        """
        Inicializa o gerenciador de mapas.

        Args:
            signals (DashboardSignals.MapaSignals, optional): Objeto de sinais específicos do mapa para comunicação com a GUI.
        """
        self.signals = signals  # Armazena a referência aos sinais do mapa
        self.map_widget = None
        self.expanded_window = None
        
        # Dados iniciais de posição do drone (exemplo: Campos dos Goytacazes/RJ).
        # Estes valores serão atualizados pelos dados recebidos do nó ROS2.
        self.drone_lat = -22.633890
        self.drone_lon = -40.093330
        self.drone_alt = 0.0
        self.drone_heading = 0.0
        self.drone_speed = 0.0

    def setup_b1_map(self):
        """
        Configura o painel B1 com o mapa interativo para ser integrado ao dashboard principal.
        Cria o widget visual do mapa, define seu estilo e configura eventos de clique para expansão.

        Returns:
            QWidget: O widget configurado com o mapa GPS pronto para integração.
        """
        # Registra o início da configuração do painel
        gui_log_info("MapaManager", "Configurando painel B1 - Mapa Interativo")
        
        # Cria o widget principal para o painel B1 e seu layout vertical.
        b1_widget = QWidget()
        b1_layout = QVBoxLayout()
        
        # Define as margens e espaçamento do layout.
        b1_layout.setContentsMargins(2, 2, 2, 2)
        b1_layout.setSpacing(0)
        
        # Cria o QLabel para o título do mapa e configura seu alinhamento e altura máxima.
        map_title = QLabel("Mapa GPS")
        map_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        map_title.setMaximumHeight(33)
        
        # Define o estilo CSS para o título do mapa.
        map_title.setStyleSheet("""
            font-weight: bold; 
            color: #ecf0f1;
            background-color: #1e5631;
            padding: 4px; 
            border-radius: 3px;
            font-size: 14px;
            border: 1px solid #27ae60;
            margin-bottom: 6px;
        """)
        
        # Configura o cursor para indicar que o título é clicável e conecta o evento de clique.
        map_title.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        map_title.mousePressEvent = self.expand_map
        b1_layout.addWidget(map_title, 0, Qt.AlignmentFlag.AlignTop)
        
        # Cria uma instância do widget de mapa interativo, passando o widget pai e o MapaManager.
        self.map_widget = InteractiveMapWidget(parent=b1_widget, mapa_manager=self)
        
        # Define o estilo CSS para o widget do mapa.
        self.map_widget.setStyleSheet("""
            QWebEngineView {
                border: 2px solid #7f8c8d;
                border-radius: 5px;
            }
        """)
        
        # Configura o cursor para indicar que o mapa é clicável e conecta o evento de clique.
        self.map_widget.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.map_widget.mousePressEvent = self.expand_map
        
        # Adiciona o widget do mapa ao layout e define o layout para o widget principal.
        b1_layout.addWidget(self.map_widget, 1)
        b1_widget.setLayout(b1_layout)
        
        return b1_widget

    def expand_map(self, event):
        """
        Abre uma janela ampliada do mapa quando o usuário clica no mapa ou no título.
        Verifica se já existe uma janela expandida aberta para evitar duplicatas.

        Args:
            event (QMouseEvent): O evento de clique do mouse que disparou a expansão.
        """
        try:
            # Registra a expansão do mapa
            gui_log_info("MapaManager", "Expandindo mapa interativo")
            
            # Verifica se uma janela expandida já está aberta e visível.
            if self.expanded_window is not None and self.expanded_window.isVisible():
                # Se sim, traz a janela existente para a frente e ativa-a.
                self.expanded_window.raise_()
                self.expanded_window.activateWindow()
                return
            
            # Caso contrário, cria uma nova janela expandida e a exibe.
            self.expanded_window = ExpandedMapWindow(mapa_manager=self)
            self.expanded_window.show()
            
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao abrir janela expandida: {str(e)}")

    def update_map_drone_position(self):
        """
        Atualiza a posição do drone em todos os widgets de mapa ativos (principal e expandido).
        Este método é chamado sempre que os dados de posição do drone são atualizados.
        """
        # Verifica se o widget do mapa principal existe e está válido.
        if hasattr(self, 'map_widget') and self.map_widget:
            # Atualiza a posição do drone no widget do mapa principal.
            self.map_widget.update_drone_position(
                self.drone_lat,
                self.drone_lon,
                self.drone_alt,
                self.drone_heading,
                self.drone_speed
            )
        
        # Verifica se a janela expandida existe, está visível e possui um widget de mapa.
        if (hasattr(self, 'expanded_window') and 
            self.expanded_window is not None and 
            self.expanded_window.isVisible() and
            hasattr(self.expanded_window, 'map_widget')):
            
            # Atualiza a posição do drone no widget do mapa da janela expandida.
            self.expanded_window.map_widget.update_drone_position(
                self.drone_lat,
                self.drone_lon,
                self.drone_alt,
                self.drone_heading,
                self.drone_speed
            )

    def update_drone_state(self, data: dict):
        """
        Atualiza todos os dados do drone no mapa com dict completo do DroneStateMSG.
        Este é o método centralizado para receber estado do drone.
        
        Args:
            data (dict): Dicionário contendo todos os campos de DroneStateMSG.
        """
        import math
        
        # Extrai coordenadas
        new_lat = data.get("current_latitude", 0.0)
        new_lon = data.get("current_longitude", 0.0)
        new_alt = data.get("current_altitude", 0.0)
        new_heading = data.get("current_yaw_deg", 0.0)
        
        # Valida coordenadas (ignora se NaN ou zero)
        if (math.isnan(new_lat) or math.isnan(new_lon) or 
            (new_lat == 0.0 and new_lon == 0.0)):
            # Apenas atualiza status, não posição
            is_armed = data.get("is_armed", False)
            is_landed = data.get("is_landed", True)
            self.update_drone_status(is_armed, is_landed)
            return
        
        # Atualiza posição GPS apenas com coordenadas válidas
        self.drone_lat = new_lat
        self.drone_lon = new_lon
        self.drone_alt = new_alt
        self.drone_heading = new_heading
        
        # Extrai status do drone
        is_armed = data.get("is_armed", False)
        is_landed = data.get("is_landed", True)
        
        # Extrai posição home
        home_lat = data.get("home_global_lat", 0.0)
        home_lon = data.get("home_global_lon", 0.0)
        
        # Atualiza posição do drone no mapa
        self.update_map_drone_position()
        
        # Atualiza status Armed/Landed no mapa
        self.update_drone_status(is_armed, is_landed)
        
        # Atualiza marcador home (só se válido)
        self.update_home_position(home_lat, home_lon)
    
    def update_drone_status(self, is_armed: bool, is_landed: bool):
        """
        Atualiza o status Armed/Landed do drone no mapa.
        
        Args:
            is_armed (bool): True se o drone está armado.
            is_landed (bool): True se o drone está em solo.
        """
        if self.map_widget and self.map_widget.map_ready:
            is_armed_js = "true" if is_armed else "false"
            is_landed_js = "true" if is_landed else "false"
            script = f"updateDroneStatus({is_armed_js}, {is_landed_js});"
            self.map_widget.page().runJavaScript(script)

    # ==================== MÉTODOS DE MISSÃO ====================
    
    def set_missions(self, missions: dict):
        """
        Recebe o dicionário de missões carregado pelo dashboard_gui.
        
        Args:
            missions (dict): Dicionário com todas as missões disponíveis.
        """
        self.loaded_missions = missions
        gui_log_info("MapaManager", f"Missões recebidas: {list(missions.keys())}")
    
    def display_mission_on_map(self, mission_name: str):
        """
        Exibe os pontos de inspeção de uma missão específica no mapa.
        
        Args:
            mission_name (str): Nome da missão a ser exibida.
        """
        if not hasattr(self, 'loaded_missions') or not self.loaded_missions:
            gui_log_error("MapaManager", "Nenhuma missão carregada")
            return
        
        mission_data = self.loaded_missions.get(mission_name)
        if not mission_data:
            gui_log_error("MapaManager", f"Missão '{mission_name}' não encontrada")
            return
        
        # Exibe pontos no mapa principal
        if self.map_widget and hasattr(self.map_widget, 'display_mission_points'):
            self.map_widget.display_mission_points(mission_data)
    
    def update_home_position(self, home_lat: float, home_lon: float):
        """
        Atualiza o marcador de home position no mapa.
        
        Args:
            home_lat (float): Latitude do home.
            home_lon (float): Longitude do home.
        """
        if self.map_widget and hasattr(self.map_widget, 'add_home_marker'):
            self.map_widget.add_home_marker(home_lat, home_lon)
    
    def clear_mission_display(self):
        """Limpa os marcadores de missão do mapa."""
        if self.map_widget and hasattr(self.map_widget, 'clear_mission_markers'):
            self.map_widget.clear_mission_markers()

class InteractiveMapWidget(QWebEngineView):
    """
    Widget de mapa interativo que utiliza QWebEngineView para renderizar um mapa Leaflet.js.
    Esta classe é responsável por carregar o HTML do mapa e comunicar-se com o JavaScript
    para atualizar a posição do drone e controlar o zoom.
    """
    
    def __init__(self, parent=None, mapa_manager=None):
        """
        Inicializa o widget do mapa interativo.

        Args:
            parent (QWidget, optional): O widget pai na hierarquia Qt.
            mapa_manager (MapaManager, optional): Referência ao MapaManager para comunicação.
        """
        super().__init__(parent)
        
        # Configura settings do WebEngine para permitir acesso a URLs remotas de arquivos locais
        # Isso é necessário para carregar tiles do OpenStreetMap
        from PyQt6.QtWebEngineCore import QWebEngineSettings
        settings = self.settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        
        self.mapa_manager = mapa_manager
        
        # Coordenadas padrão para o centro do mapa e nível de zoom.
        # Estes valores podem ser sobrescritos por parâmetros de configuração.
        self.map_center_lat = -22.633890
        self.map_center_lon = -40.093330
        self.zoom_level = 15
        
        # Carrega os parâmetros do mapa de um arquivo de configuração.
        self.load_map_parameters()
        
        # Flag para indicar se o mapa está pronto para receber comandos JavaScript
        self.map_ready = False
        
        # Localiza o arquivo HTML do mapa.
        # Ordem de busca:
        # 1. Diretório share do pacote ROS2 (instalação via colcon)
        # 2. Diretório do arquivo fonte (desenvolvimento)
        # 3. Diretório de trabalho atual
        map_file_path = None
        
        # Tenta o diretório share do pacote ROS2 primeiro
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('drone_inspetor')
            share_path = os.path.join(package_share_dir, 'gui', 'map.html')
            if os.path.exists(share_path):
                map_file_path = share_path
        except Exception:
            pass
        
        # Fallback para o diretório do arquivo fonte (desenvolvimento)
        if map_file_path is None:
            source_path = os.path.join(os.path.dirname(__file__), "map.html")
            if os.path.exists(source_path):
                map_file_path = source_path
        
        # Fallback para o diretório de trabalho atual
        if map_file_path is None:
            cwd_path = os.path.join(os.getcwd(), "map.html")
            if os.path.exists(cwd_path):
                map_file_path = cwd_path
        
        # Carrega o arquivo HTML no QWebEngineView se ele for encontrado.
        if map_file_path and os.path.exists(map_file_path):
            abs_path = os.path.abspath(map_file_path)
            gui_log_info("MapaManager", f"Carregando mapa de: {abs_path}")
            
            # Conecta o sinal loadFinished para inicializar o mapa APENAS quando a página estiver pronta
            self.loadFinished.connect(self._on_load_finished)
            
            self.load(QUrl.fromLocalFile(abs_path))
        else:
            # Registra erro se o arquivo map.html não for encontrado
            gui_log_error("MapaManager", f"Arquivo map.html não encontrado!")
    
    def _on_load_finished(self, success: bool):
        """
        Callback chamado quando o carregamento da página HTML é concluído.
        
        Args:
            success (bool): True se o carregamento foi bem-sucedido.
        """
        if success:
            gui_log_info("MapaManager", "Página HTML carregada com sucesso, inicializando mapa...")
            # Pequeno delay para garantir que o JavaScript esteja pronto
            QTimer.singleShot(500, self.initialize_map)
        else:
            gui_log_error("MapaManager", "Falha ao carregar a página HTML do mapa!")
    
    def load_map_parameters(self):
        """
        Carrega parâmetros de configuração do mapa a partir de um arquivo YAML.
        Permite personalizar o centro do mapa e o zoom sem alterar o código.
        """
        try:
            # Constrói o caminho para o arquivo params.yaml.
            params_path = os.path.join(os.path.dirname(__file__), "..", "config", "params.yaml")
            if not os.path.exists(params_path):
                params_path = os.path.join(os.getcwd(), "config", "params.yaml")
            
            # Se o arquivo existir, carrega os parâmetros.
            if os.path.exists(params_path):
                with open(params_path, 'r') as file:
                    params = yaml.safe_load(file)
                
                # Extrai as configurações do mapa se elas estiverem presentes.
                if 'map_center' in params:
                    map_config = params['map_center']
                    self.map_center_lat = map_config.get('latitude', -22.633890)
                    self.map_center_lon = map_config.get('longitude', -40.093330)
                    self.zoom_level = map_config.get('zoom_level', 15)
                    
        except Exception as e:
            # Em caso de erro ao carregar os parâmetros, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao carregar parâmetros do mapa: {e}")
    
    def initialize_map(self):
        """
        Inicializa o mapa JavaScript após o carregamento completo da página HTML.
        Executa comandos JavaScript para definir o centro e o zoom inicial do mapa.
        """
        try:
            gui_log_info("MapaManager", f"Inicializando mapa com centro: ({self.map_center_lat}, {self.map_center_lon}), zoom: {self.zoom_level}")
            
            # Marca o mapa como pronto para receber comandos
            self.map_ready = True
            
            # Constrói o script JavaScript para definir o centro e o zoom do mapa.
            script = f"setMapCenter({self.map_center_lat}, {self.map_center_lon}, {self.zoom_level});"
            # Executa o script JavaScript na página do QWebEngineView.
            self.page().runJavaScript(script, self.on_js_finished)
            
            gui_log_info("MapaManager", "Mapa inicializado com sucesso!")
            
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao inicializar mapa: {e}")
    
    def update_drone_position(self, lat, lon, alt, heading, speed):
        """
        Atualiza a posição do drone no mapa via execução de JavaScript.

        Args:
            lat (float): Latitude do drone (graus decimais).
            lon (float): Longitude do drone (graus decimais).
            alt (float): Altitude do drone (metros).
            heading (float): Direção do drone em graus (0° = Norte, sentido horário).
            speed (float): Velocidade do drone (m/s).
        """
        # Não executa JavaScript se o mapa ainda não estiver pronto
        if not self.map_ready:
            return
            
        try:
            # Constrói o script JavaScript para atualizar a posição do drone.
            script = f"updateDronePosition({lat}, {lon}, {alt}, {heading}, {speed});"
            # Executa o script JavaScript na página do QWebEngineView.
            self.page().runJavaScript(script)
            
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao atualizar posição do drone: {e}")
    
    def zoom_in(self):
        """
        Aumenta o nível de zoom do mapa via JavaScript.
        """
        try:
            # Executa o script JavaScript para aumentar o zoom.
            self.page().runJavaScript("zoomIn();")
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao fazer zoom in: {e}")
    
    def zoom_out(self):
        """
        Diminui o nível de zoom do mapa via JavaScript.
        """
        try:
            # Executa o script JavaScript para diminuir o zoom.
            self.page().runJavaScript("zoomOut();")
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("MapaManager", f"Erro ao fazer zoom out: {e}")
    
    def on_js_finished(self, result):
        """
        Callback para quando a execução de JavaScript é concluída.
        Pode ser usado para depuração ou para lidar com resultados de scripts.

        Args:
            result: O resultado da execução do JavaScript.
        """
        # Este método é um placeholder para depuração e não realiza nenhuma ação visível.
        pass

    # ==================== MÉTODOS PARA MARCADORES DE MISSÃO ====================
    
    def add_home_marker(self, lat: float, lon: float):
        """
        Adiciona um marcador "H" para a posição home no mapa.
        Remove o marcador se as coordenadas forem inválidas (NaN ou zero).
        
        Args:
            lat (float): Latitude do home em graus.
            lon (float): Longitude do home em graus.
        """
        import math
        
        if not self.map_ready:
            return
        
        # Valida se os valores são números válidos (não NaN)
        if math.isnan(lat) or math.isnan(lon):
            # Remove marcador home se existir
            self._remove_home_marker()
            return
        
        # Remove marcador se as coordenadas são zero (não inicializado)
        if lat == 0.0 and lon == 0.0:
            self._remove_home_marker()
            return
        
        try:
            script = f"addHomeMarker({lat}, {lon});"
            self.page().runJavaScript(script)
        except Exception as e:
            gui_log_error("MapaManager", f"Erro ao adicionar marcador home: {e}")
    
    def _remove_home_marker(self):
        """Remove o marcador home do mapa."""
        if not self.map_ready:
            return
        try:
            self.page().runJavaScript("removeHomeMarker();")
        except Exception:
            pass
    
    def add_inspection_point(self, index: int, lat: float, lon: float, is_detection_point: bool = False):
        """
        Adiciona um ponto de inspeção numerado no mapa.
        
        Args:
            index (int): Índice/número do ponto de inspeção.
            lat (float): Latitude do ponto.
            lon (float): Longitude do ponto.
            is_detection_point (bool): True se é um ponto de detecção (cor vermelha).
        """
        if not self.map_ready:
            gui_log_error("MapaManager", "Mapa não pronto para adicionar ponto de inspeção")
            return
        
        try:
            is_detection_js = "true" if is_detection_point else "false"
            script = f"addInspectionPoint({index}, {lat}, {lon}, {is_detection_js});"
            self.page().runJavaScript(script)
        except Exception as e:
            gui_log_error("MapaManager", f"Erro ao adicionar ponto de inspeção: {e}")
    
    def display_mission_points(self, mission_data: dict):
        """
        Exibe todos os pontos de inspeção de uma missão no mapa.
        
        Args:
            mission_data (dict): Dados da missão contendo 'pontos_de_inspecao'.
        """
        if not mission_data or 'pontos_de_inspecao' not in mission_data:
            gui_log_error("MapaManager", "Dados de missão inválidos")
            return
        
        # Limpa marcadores anteriores
        self.clear_mission_markers()
        
        pontos = mission_data.get('pontos_de_inspecao', [])
        for i, ponto in enumerate(pontos, start=1):
            lat = ponto.get('lat')
            lon = ponto.get('lon')
            is_detection = ponto.get('ponto_de_deteccao', False)
            
            if lat is not None and lon is not None:
                self.add_inspection_point(i, lat, lon, is_detection)
        
        gui_log_info("MapaManager", f"Exibidos {len(pontos)} pontos de inspeção no mapa")
    
    def clear_mission_markers(self):
        """
        Remove todos os marcadores de missão (pontos de inspeção e home) do mapa.
        """
        if not self.map_ready:
            return
        
        try:
            self.page().runJavaScript("clearMissionMarkers();")
            gui_log_info("MapaManager", "Marcadores de missão limpos")
        except Exception as e:
            gui_log_error("MapaManager", f"Erro ao limpar marcadores de missão: {e}")

class ExpandedMapWindow(QMainWindow):
    """
    Janela expandida para exibir o mapa em tela cheia.
    """
    def __init__(self, mapa_manager=None):
        """
        Inicializa a janela expandida do mapa.

        Args:
            mapa_manager (MapaManager, optional): Referência ao MapaManager.
        """
        super().__init__()
        # Define o título e a geometria da janela.
        self.setWindowTitle("Mapa GPS Expandido")
        self.setGeometry(100, 100, 1200, 800)
        
        self.mapa_manager = mapa_manager
        
        # Configura o widget central e seu layout.
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Cria uma nova instância do InteractiveMapWidget para a janela expandida.
        self.map_widget = InteractiveMapWidget(parent=self, mapa_manager=mapa_manager)
        layout.addWidget(self.map_widget)
        
        # Conecta o sinal de carregamento do mapa ao callback.
        self.map_widget.loadFinished.connect(self._on_expanded_map_load_finished)

    def _on_expanded_map_load_finished(self, ok):
        """
        Callback chamado quando o mapa na janela expandida termina de carregar.

        Args:
            ok (bool): True se o carregamento foi bem-sucedido, False caso contrário.
        """
        if ok:
            gui_log_info("ExpandedMapWindow", "Mapa expandido carregado com sucesso")
            # Opcional: injetar JavaScript específico para a versão expandida, se necessário
        else:
            gui_log_error("ExpandedMapWindow", "Falha ao carregar mapa expandido")


