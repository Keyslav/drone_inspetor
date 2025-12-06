"""
dashboard_node.py
=================================================================================================
Nó ROS2 principal do Dashboard do Drone Inspetor.

Este nó atua como orquestrador central entre a interface gráfica (GUI) e os demais nós do sistema.
É responsável por gerenciar a comunicação bidirecional entre a GUI PyQt6 e o ambiente ROS2,
coordenando subscribers e publishers modulares para diferentes componentes do sistema.

ARQUITETURA:
- Recebe dados de sensores através de subscribers modulares
- Publica comandos de controle através de publishers modulares
- Utiliza sinais PyQt6 para comunicação assíncrona com a GUI
- Executa em thread separada para não bloquear a interface gráfica
=================================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QThread
import sys

# Importações de sinais e GUI
from drone_inspetor.signals.dashboard_signals import DashboardSignals
from drone_inspetor.gui.dashboard_gui import DashboardGUI

# Importações de subscribers modulares
# Cada subscriber gerencia a assinatura de um tópico específico e emite sinais PyQt6
from drone_inspetor.subscribers.dashboard_camera_subscriber import DashboardCameraSubscriber
from drone_inspetor.subscribers.dashboard_cv_subscriber import DashboardCVSubscriber
from drone_inspetor.subscribers.dashboard_depth_subscriber import DashboardDepthSubscriber
from drone_inspetor.subscribers.dashboard_lidar_subscriber import DashboardLidarSubscriber
from drone_inspetor.subscribers.dashboard_drone_subscriber import DashboardDroneSubscriber
from drone_inspetor.subscribers.dashboard_fsm_subscriber import DashboardFSMSubscriber
from drone_inspetor.subscribers.dashboard_mapa_subscriber import DashboardMapaSubscriber

# Importações de publishers modulares
# Cada publisher gerencia a publicação de comandos para um nó específico
from drone_inspetor.publishers.dashboard_drone_publisher import DashboardDronePublisher
from drone_inspetor.publishers.dashboard_camera_publisher import DashboardCameraPublisher
from drone_inspetor.publishers.dashboard_cv_publisher import DashboardCVPublisher
from drone_inspetor.publishers.dashboard_depth_publisher import DashboardDepthPublisher
from drone_inspetor.publishers.dashboard_lidar_publisher import DashboardLidarPublisher
from drone_inspetor.publishers.dashboard_mapa_publisher import DashboardMapaPublisher
from drone_inspetor.publishers.dashboard_fsm_publisher import DashboardFSMPublisher


class DashboardNode(Node):
    """
    Nó ROS2 para o dashboard principal.
    
    Este nó coordena a comunicação entre a interface gráfica e os demais componentes do sistema.
    Utiliza uma arquitetura modular com subscribers e publishers dedicados para cada componente,
    facilitando a manutenção e extensibilidade do código.
    
    Responsabilidades:
    - Orquestrar subscribers modulares que recebem dados dos nós do sistema
    - Orquestrar publishers modulares que enviam comandos para os nós do sistema
    - Servir como ponto central de comunicação entre ROS2 e a GUI PyQt6
    - Expor métodos públicos para a GUI publicar comandos de forma simplificada
    """
    
    def __init__(self, signals: DashboardSignals):
        """
        Inicializa o DashboardNode.

        Args:
            signals (DashboardSignals): Instância da classe DashboardSignals que contém
                                       todos os sinais PyQt6 para comunicação com a GUI.
        """
        super().__init__("dashboard_node")
        self.get_logger().info("Dashboard Node started.")

        # Armazena a referência aos sinais PyQt6 para comunicação com a GUI
        self.signals = signals

        # ==================== INICIALIZAÇÃO DOS SUBSCRIBERS MODULARIZADOS ====================
        # Cada subscriber assina um tópico ROS2 específico e emite sinais PyQt6 quando recebe dados
        # Os sinais são conectados aos slots da GUI para atualização da interface
        
        # Subscriber para imagens da câmera principal
        self.camera_subscriber = DashboardCameraSubscriber(self, self.signals.camera)
        
        # Subscriber para imagens processadas por visão computacional
        self.cv_subscriber = DashboardCVSubscriber(self, self.signals.cv)
        
        # Subscriber para imagens e dados de profundidade
        self.depth_subscriber = DashboardDepthSubscriber(self, self.signals.depth)
        
        # Subscriber para dados do sensor LiDAR
        self.lidar_subscriber = DashboardLidarSubscriber(self, self.signals.lidar)
        
        # Subscriber para dados do drone (posição, atitude) e mapa
        self.control_subscriber = DashboardDroneSubscriber(self, self.signals.control, self.signals.mapa)
        
        # Subscriber para estado da máquina de estados finitos (FSM)
        self.fsm_subscriber = DashboardFSMSubscriber(self, self.signals.fsm)
        
        # Subscriber para dados específicos do mapa
        self.mapa_subscriber = DashboardMapaSubscriber(self, self.signals.mapa)

        # ==================== INICIALIZAÇÃO DOS PUBLISHERS MODULARIZADOS ====================
        # Cada publisher gerencia a publicação de comandos para um nó específico do sistema
        # Os comandos são enviados em formato JSON para padronização da comunicação
        
        # Publisher para comandos de controle do drone
        self.control_publisher = DashboardDronePublisher(self)
        
        # Publisher para comandos de controle da câmera
        self.camera_publisher = DashboardCameraPublisher(self)
        
        # Publisher para comandos de controle de visão computacional
        self.cv_publisher = DashboardCVPublisher(self)
        
        # Publisher para comandos de controle de profundidade
        self.depth_publisher = DashboardDepthPublisher(self)
        
        # Publisher para comandos de controle do LiDAR
        self.lidar_publisher = DashboardLidarPublisher(self)
        
        # Publisher para comandos de posição e atitude do mapa
        self.mapa_publisher = DashboardMapaPublisher(self)
        
        # Publisher para comandos de missão para a FSM
        self.fsm_publisher = DashboardFSMPublisher(self)
        
        # Configura os publishers nos signals para permitir publicação de comandos
        # Isso permite que a GUI publique comandos através dos signals
        self.signals.configure_publishers(self.fsm_publisher)

        self.get_logger().info("Nó do Dashboard inicializado com subscribers e publishers modulares.")



def main(args=None):
    """
    Função principal para iniciar o nó ROS2 e a aplicação GUI.
    
    Esta função configura e inicia todos os componentes necessários:
    1. Inicializa o ROS2
    2. Cria a aplicação PyQt6
    3. Cria os sinais compartilhados entre ROS2 e GUI
    4. Inicializa o executor ROS2 em thread separada
    5. Inicializa o nó do dashboard
    6. Inicializa e exibe a GUI
    7. Executa o loop de eventos até o encerramento
    8. Realiza limpeza adequada de recursos
    
    Args:
        args: Argumentos de linha de comando (opcional)
    """
    # Inicializa o ROS2
    rclpy.init(args=args)
    
    # Cria a aplicação PyQt6 para a interface gráfica
    app = QApplication(sys.argv)

    # Cria instância dos sinais PyQt6 que serão compartilhados entre o nó ROS2 e a GUI
    # Os sinais permitem comunicação assíncrona entre threads
    signals = DashboardSignals()

    # ==================== CRIAÇÃO DO EXECUTOR ROS2 ====================
    # Utiliza MultiThreadedExecutor para permitir processamento paralelo de callbacks
    executor = MultiThreadedExecutor()

    # ==================== INICIALIZAÇÃO DO NÓ ROS2 DO DASHBOARD ====================
    # Cria o nó do dashboard e adiciona ao executor
    dashboard_node = DashboardNode(signals=signals)
    executor.add_node(dashboard_node)

    # ==================== THREAD DO EXECUTOR ROS2 ====================
    # O executor ROS2 roda em uma thread separada para não bloquear a GUI
    # Isso permite que a interface gráfica permaneça responsiva enquanto processa mensagens ROS2
    executor_thread = QThread()
    executor_thread.run = lambda: executor.spin()
    executor_thread.start()

    # ==================== INICIALIZAÇÃO DA GUI ====================
    # A GUI recebe apenas os sinais PyQt6, que já contêm os métodos de publicação de comandos
    # Isso mantém a separação entre GUI e ROS2, facilitando manutenção e testes
    dashboard_gui = DashboardGUI(signals=signals)
    dashboard_gui.showMaximized()

    # ==================== LOOP DE EVENTOS PRINCIPAL ====================
    # Inicia o loop de eventos da aplicação PyQt6
    # Este loop processa eventos da GUI (cliques, redimensionamentos, etc.)
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        # Trata interrupção por teclado (Ctrl+C) de forma graciosa
        exit_code = 0

    # ==================== LIMPEZA E FINALIZAÇÃO ====================
    # Realiza limpeza adequada de todos os recursos antes de encerrar
    print("Iniciando limpeza...")
    
    # Para o executor ROS2 e aguarda a thread finalizar
    executor.shutdown()
    executor_thread.quit()
    executor_thread.wait(5000)  # Espera até 5 segundos para a thread finalizar
    
    # Destroi o nó ROS2
    dashboard_node.destroy_node()
    
    # Finaliza o ROS2
    rclpy.shutdown()
    
    # Encerra a aplicação com o código de saída apropriado
    sys.exit(exit_code)

if __name__ == '__main__':
    main()


