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

    def update_position_from_node(self, data):
        """
        Atualiza a latitude, longitude e altitude do drone com dados recebidos de um nó ROS2.
        Após a atualização, chama o método para refletir a nova posição no mapa.

        Args:
            data (dict): Dicionário contendo 'lat', 'lon' e 'alt'.
        """
        # Atualiza a latitude, longitude e altitude do drone com os dados recebidos.
        self.drone_lat = data.get("lat", self.drone_lat)
        self.drone_lon = data.get("lon", self.drone_lon)
        self.drone_alt = data.get("alt", self.drone_alt)
        
        # Chama o método para atualizar a posição do drone no mapa.
        self.update_map_drone_position()

    def update_attitude_from_node(self, data):
        """
        Atualiza a direção (heading) e velocidade do drone com dados recebidos de um nó ROS2.
        Após a atualização, chama o método para refletir a nova atitude no mapa.

        Args:
            data (dict): Dicionário contendo 'heading' e 'speed'.
        """
        # Atualiza a direção e velocidade do drone com os dados recebidos.
        self.drone_heading = data.get("heading", self.drone_heading)
        self.drone_speed = data.get("speed", self.drone_speed)
        
        # Chama o método para atualizar a posição do drone no mapa.
        self.update_map_drone_position()

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
        
        self.mapa_manager = mapa_manager
        
        # Coordenadas padrão para o centro do mapa e nível de zoom.
        # Estes valores podem ser sobrescritos por parâmetros de configuração.
        self.map_center_lat = -22.633890
        self.map_center_lon = -40.093330
        self.zoom_level = 15
        
        # Carrega os parâmetros do mapa de um arquivo de configuração.
        self.load_map_parameters()
        
        # Localiza o arquivo HTML do mapa. Primeiro tenta no diretório atual, depois no diretório de trabalho.
        map_file_path = os.path.join(os.path.dirname(__file__), "map.html")
        if not os.path.exists(map_file_path):
            map_file_path = os.path.join(os.getcwd(), "map.html")
        
        # Carrega o arquivo HTML no QWebEngineView se ele for encontrado.
        if os.path.exists(map_file_path):
            self.load(QUrl.fromLocalFile(os.path.abspath(map_file_path)))
        else:
            # Registra erro se o arquivo map.html não for encontrado
            gui_log_error("InteractiveMapWidget", f"Arquivo map.html não encontrado em: {map_file_path}")
        
        # Configura um timer para inicializar o mapa JavaScript após o carregamento da página.
        # Isso garante que o JavaScript seja executado apenas quando a página estiver pronta.
        self.init_timer = QTimer()
        self.init_timer.timeout.connect(self.initialize_map)
        self.init_timer.setSingleShot(True)
        self.init_timer.start(5000) # Atraso de 5 segundos para garantir o carregamento da página.
    
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
            gui_log_error("InteractiveMapWidget", f"Erro ao carregar parâmetros do mapa: {e}")
    
    def initialize_map(self):
        """
        Inicializa o mapa JavaScript após o carregamento completo da página HTML.
        Executa comandos JavaScript para definir o centro e o zoom inicial do mapa.
        """
        try:
            # Constrói o script JavaScript para definir o centro e o zoom do mapa.
            script = f"setMapCenter({self.map_center_lat}, {self.map_center_lon}, {self.zoom_level});"
            # Executa o script JavaScript na página do QWebEngineView.
            self.page().runJavaScript(script, self.on_js_finished)
            
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("InteractiveMapWidget", f"Erro ao inicializar mapa: {e}")
    
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
        try:
            # Constrói o script JavaScript para atualizar a posição do drone.
            script = f"updateDronePosition({lat}, {lon}, {alt}, {heading}, {speed});"
            # Executa o script JavaScript na página do QWebEngineView.
            self.page().runJavaScript(script)
            
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("InteractiveMapWidget", f"Erro ao atualizar posição do drone: {e}")
    
    def zoom_in(self):
        """
        Aumenta o nível de zoom do mapa via JavaScript.
        """
        try:
            # Executa o script JavaScript para aumentar o zoom.
            self.page().runJavaScript("zoomIn();")
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("InteractiveMapWidget", f"Erro ao fazer zoom in: {e}")
    
    def zoom_out(self):
        """
        Diminui o nível de zoom do mapa via JavaScript.
        """
        try:
            # Executa o script JavaScript para diminuir o zoom.
            self.page().runJavaScript("zoomOut();")
        except Exception as e:
            # Em caso de erro, registra mensagem de erro
            gui_log_error("InteractiveMapWidget", f"Erro ao fazer zoom out: {e}")
    
    def on_js_finished(self, result):
        """
        Callback para quando a execução de JavaScript é concluída.
        Pode ser usado para depuração ou para lidar com resultados de scripts.

        Args:
            result: O resultado da execução do JavaScript.
        """
        # Este método é um placeholder para depuração e não realiza nenhuma ação visível.
        pass

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


