"""
dashboard_signals.py
=================================================================================================
Sistema de sinais PyQt6 para comunicação assíncrona entre ROS2 e GUI.

Este módulo define todas as classes de sinais utilizadas para comunicação entre o DashboardNode
e o DashboardGUI. Os sinais permitem que dados sejam transmitidos de forma assíncrona entre
threads, mantendo a GUI responsiva enquanto processa mensagens ROS2.

ARQUITETURA:
- Sinais modulares por componente (Camera, CV, Depth, LiDAR, FSM, Drone, Mapa)
- Métodos de publicação de comandos incorporados aos signals
- Separação clara entre recepção de dados (sinais) e envio de comandos (métodos)
=================================================================================================
"""

from PyQt6.QtCore import pyqtSignal, QObject

class CameraSignals(QObject):
    """
    Sinais relacionados à câmera principal.
    
    Emite sinais quando novas imagens ou atualizações de status são recebidas.
    """
    image_received = pyqtSignal(object)       # Recebe a imagem da câmera
    status_updated = pyqtSignal(str)          # Atualizações de status da câmera
    recording_status_received = pyqtSignal(bool)  # Status de gravação (True = gravando)

class LidarSignals(QObject):
    """
    Sinais relacionados ao sensor LiDAR.
    
    Emite sinais quando novos dados de varredura laser, estatísticas ou detecções são recebidos.
    """
    lidar_data_received = pyqtSignal(dict)          # Dados consolidados do LiDAR (point_vector + ground_distance)
    point_vector_received = pyqtSignal(object)      # Recebe o vetor de pontos do LiDAR (legado)
    statistics_received = pyqtSignal(dict)          # Estatísticas do LiDAR
    obstacle_detections_received = pyqtSignal(list) # Detecções de obstáculos do LiDAR

class FSMSignals(QObject):
    """
    Sinais relacionados à Máquina de Estados Finita (FSM).
    
    Emite sinais quando o estado da FSM muda.
    O sinal emite um dict com todos os campos de FSMStateMSG.
    """
    fsm_state_updated = pyqtSignal(dict)    # Estado completo da FSM (dict)

class DroneSignals(QObject):
    """
    Sinais relacionados ao controle do drone.
    
    Emite sinais quando novos dados de estado do drone são recebidos.
    O sinal drone_state_updated emite um dict com todos os campos de DroneStateMSG.
    """
    drone_state_updated = pyqtSignal(dict)  # Estado completo do drone (dict)
    mission_command_sent = pyqtSignal(str)  # Comando de missão enviado
    
    def __init__(self):
        """
        Inicializa os sinais de controle.
        
        Os publishers serão configurados posteriormente pelo DashboardNode.
        """
        super().__init__()
        # Publishers serão atribuídos pelo DashboardNode após criação
        self._fsm_publisher = None
    
    def set_fsm_publisher(self, fsm_publisher):
        """
        Configura o publisher de comandos FSM.
        
        Args:
            fsm_publisher: Instância de DashboardFSMPublisher para publicação de comandos
        """
        self._fsm_publisher = fsm_publisher
    
    def send_mission_command(self, command_json):
        """
        Publica um comando de missão para a FSM.
        
        Formato esperado: JSON string contendo:
        {
            "command": "<nome_do_comando>",
            ... outras variáveis específicas do comando ...
        }
        
        Comandos suportados:
        - "start_inspection": Inicia uma missão de inspeção
            Variáveis opcionais:
                - "inspection_type" (str): Tipo de inspeção (ex: "Flare", "Tanques de GLP", etc.)
        - "stop_inspection": Pausa a missão atual
        - "cancel_inspection": Cancela completamente a missão
        - "return_to_base": Comanda o drone a retornar à base
        
        Args:
            command_json (str): String JSON contendo o comando de missão e suas variáveis
        """
        if self._fsm_publisher:
            self._fsm_publisher.send_mission_command(command_json)
            # Emite sinal para notificar que o comando foi enviado
            self.mission_command_sent.emit(command_json)

class CVSignals(QObject):
    """
    Sinais relacionados à Visão Computacional (CV).
    
    Emite sinais quando novas imagens processadas, dados de análise ou detecções são recebidos.
    """
    image_received = pyqtSignal(object)       # Imagem processada pelo CV
    analysis_data_received = pyqtSignal(dict) # Dados de análise CV
    detections_received = pyqtSignal(str)     # Detecções CV

class DepthSignals(QObject):
    """
    Sinais relacionados ao sensor de Profundidade.
    
    Emite sinais quando novas imagens de profundidade, estatísticas ou alertas são recebidos.
    """
    image_received = pyqtSignal(object)    # Imagem de profundidade
    statistics_received = pyqtSignal(dict) # Estatísticas de profundidade
    proximity_alert_received = pyqtSignal(list) # Alertas de proximidade

class MapaSignals(QObject):
    """
    Sinais relacionados ao Mapa GPS.
    
    Emite sinais quando a posição ou atitude do drone são atualizadas no mapa.
    """
    drone_state_updated = pyqtSignal(dict)    # Estado completo do drone
    home_position_updated = pyqtSignal(dict)  # Posição home atualizada (home_lat, home_lon)
    mission_selected = pyqtSignal(str)        # Nome da missão selecionada no dropdown
    mission_started = pyqtSignal(str)         # Nome da missão iniciada (para exibir pontos)
    mission_ended = pyqtSignal()              # Missão finalizada (limpar marcadores)

class DashboardSignals(QObject):
    """
    Classe principal que agrega todos os sinais modularizados para o Dashboard.
    
    Permite que o DashboardNode e o DashboardGUI acessem os sinais de forma organizada.
    Também gerencia a configuração dos publishers para métodos de publicação de comandos.
    """
    
    def __init__(self):
        """
        Inicializa todos os sinais modulares do dashboard.
        """
        super().__init__()
        self.camera = CameraSignals()
        self.lidar = LidarSignals()
        self.fsm = FSMSignals()
        self.control = DroneSignals()
        self.cv = CVSignals()
        self.depth = DepthSignals()
        self.mapa = MapaSignals()
    
    def configure_publishers(self, fsm_publisher):
        """
        Configura os publishers necessários para métodos de publicação de comandos.
        
        Este método deve ser chamado pelo DashboardNode após criar os publishers.
        
        Args:
            fsm_publisher: Instância de DashboardFSMPublisher para publicação de comandos FSM
        """
        self.control.set_fsm_publisher(fsm_publisher)
