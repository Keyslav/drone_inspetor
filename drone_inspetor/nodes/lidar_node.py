"""
lidar_node.py
=================================================================================================
Nó ROS2 responsável pelo processamento de dados LiDAR.

Este nó processa dados de varredura laser (LaserScan) recebidos de sensores LiDAR.
Realiza análise de distâncias, detecta obstáculos em diferentes direções, e calcula
estatísticas para auxiliar na navegação e prevenção de colisões.

ARQUITETURA:
- Assina: /drone_inspetor/externo/lidar/scan (dados LaserScan do sensor)
- Assina: /drone_inspetor/dashboard/lidar/control (comandos de controle em JSON)
- Publica: /drone_inspetor/lidar_node/point_vector (vetor de pontos para visualização)
- Publica: /drone_inspetor/lidar_node/statistics (estatísticas em JSON)
- Publica: /drone_inspetor/lidar_node/obstacle_detections (detecções de obstáculos em JSON)

FUNCIONALIDADES:
- Processamento de mensagens LaserScan (varredura angular de distâncias)
- Detecção de obstáculos em setores (frente, esquerda, direita, trás)
- Cálculo de estatísticas (mínimo, máximo, média, desvio padrão)
- Geração de vetor de pontos para visualização no dashboard HTML
- Publicação periódica de dados a 2Hz (500ms)
=================================================================================================
"""

# ==================== IMPORTAÇÕES ====================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray

import numpy as np
import json
from datetime import datetime
import math

class LidarNode(Node):
    """
    Nó ROS2 para processamento de dados LiDAR.
    
    Este nó processa dados de varredura laser recebidos de sensores LiDAR. Analisa as
    distâncias medidas em diferentes ângulos, detecta obstáculos em setores específicos,
    e fornece estatísticas para auxiliar na navegação autônoma e prevenção de colisões.
    
    Responsabilidades:
    - Assinar tópico de LaserScan do sensor LiDAR externo
    - Processar dados de varredura laser e converter para formato utilizável
    - Detectar obstáculos em diferentes direções (frente, esquerda, direita, trás)
    - Calcular estatísticas de distância (mínimo, máximo, média, desvio padrão)
    - Gerar vetor de pontos para visualização no dashboard HTML
    - Publicar estatísticas e detecções em tópicos padronizados em formato JSON
    - Responder a comandos de controle do dashboard (ex: ajuste de threshold)
    """
    
    def __init__(self):
        """
        Inicializa o nó de LiDAR.
        """
        super().__init__("lidar_node")
        self.get_logger().info("Nó LidarNode iniciado.")
        
        # ==================== CONFIGURAÇÃO DE QoS ========================
        # QoS para dados de sensores: VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS para comandos: VOLATILE + RELIABLE (garantir entrega de comandos)
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==================== SUBSCRIBERS ====================
        # Assina tópico de varredura laser do sensor LiDAR externo
        # As mensagens LaserScan contêm arrays de distâncias medidas em diferentes ângulos
        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            "/drone_inspetor/externo/lidar/scan",
            self.laserscan_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado tópico: {self.laserscan_subscription.topic_name}")
        
        # Assina tópico de comandos de controle do dashboard
        # Permite ajustar parâmetros do LiDAR em tempo de execução (ex: threshold de obstáculos)
        self.lidar_control_subscription = self.create_subscription(
            String,
            "/drone_inspetor/dashboard/lidar/control",
            self.lidar_control_callback,
            qos_commands
        )
        self.get_logger().info(f"Assinado tópico: {self.lidar_control_subscription.topic_name}")
        
        # ==================== PUBLISHERS ====================
        # Publica vetor de pontos LiDAR para visualização no dashboard HTML
        # Formato: [distancia1, angulo1, distancia2, angulo2, ...]
        # Este formato é otimizado para renderização em JavaScript/HTML5 Canvas
        self.point_vector_publisher = self.create_publisher(
            Float32MultiArray,
            "/drone_inspetor/lidar_node/point_vector",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.point_vector_publisher.topic_name}")

        # Publica estatísticas de LiDAR em formato JSON
        # Contém: número de pontos válidos, distância mínima/máxima/média, desvio padrão
        self.lidar_stats_publisher = self.create_publisher(
            String,
            "/drone_inspetor/lidar_node/statistics",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.lidar_stats_publisher.topic_name}")
        
        # Publica detecções de obstáculos em formato JSON
        # Indica presença de obstáculos em diferentes setores (frente, esquerda, direita, trás)
        self.obstacle_publisher = self.create_publisher(
            String,
            "/drone_inspetor/lidar_node/obstacle_detections",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando no tópico: {self.obstacle_publisher.topic_name}")
        
        # ==================== TIMER PARA RELATÓRIOS ====================
        # Timer para publicar dados periodicamente a 2Hz (500ms)
        # Publica vetor de pontos, estatísticas e detecções de obstáculos
        # A publicação periódica garante que o dashboard sempre tenha dados atualizados
        self.stats_timer = self.create_timer(0.5, self.publish_lidar_data_and_stats)
        
        self.get_logger().info("LidarNode inicializado com sucesso.")
        
        # ==================== INICIALIZAÇÃO DE ATRIBUTOS ===============================================
        # Armazena estatísticas calculadas da última varredura processada
        self.lidar_statistics = {}
        
        # Armazena detecções de obstáculos da última varredura processada
        self.obstacle_detections = {}
        
        # Armazena a última mensagem LaserScan recebida
        self.current_laserscan = None
        
        # Armazena o vetor de pontos calculado da última varredura
        # Formato: [distancia1, angulo1, distancia2, angulo2, ...]
        self.current_point_vector_data = []
        
        # ==================== CONFIGURAÇÕES PADRÃO ======================================================
        # Range válido de distância: valores fora deste range são considerados inválidos
        self.min_range = 0.1   # metros (distância mínima válida)
        self.max_range = 10.0  # metros (distância máxima válida)
        
        # Threshold de obstáculo: distâncias menores que este valor indicam presença de obstáculo
        self.obstacle_threshold = 2.0  # metros

    def laserscan_callback(self, msg):
        """
        Callback para processar varredura laser LaserScan.
        Calcula vetor de pontos, estatísticas e obstáculos, armazenando para publicação periódica.
        
        Args:
            msg: Mensagem sensor_msgs/LaserScan
        """
        self.get_logger().debug("Processando varredura laser")
        
        try:
            self.current_laserscan = msg
            
            # Converte LaserScan para array numpy
            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

            # Gerar vetor de pontos no formato [distancia1, angulo1, distancia2, angulo2, ...]
            point_vector = []
            for i in range(len(ranges)):
                if ranges[i] >= msg.range_min and ranges[i] <= msg.range_max and np.isfinite(ranges[i]):
                    point_vector.append(float(ranges[i]))
                    point_vector.append(float(angles[i]))
            
            self.current_point_vector_data = point_vector
            
            # Calcular estatísticas e obstáculos
            self.lidar_statistics = self.calculate_statistics(ranges)
            self.obstacle_detections = self.calculate_obstacles(ranges, angles)
            
            # Registrar no log
            self.get_logger().debug(f"Estatísticas calculadas: {json.dumps(self.lidar_statistics)}")
            self.get_logger().debug(f"Obstáculos detectados: {json.dumps(self.obstacle_detections)}")
            self.get_logger().debug(f"Vetor de pontos calculado: {len(point_vector)//2} pontos")
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de LaserScan: {e}")

    def lidar_control_callback(self, msg):
        """
        Callback para comandos de controle do LiDAR.
        
        Args:
            msg: Mensagem std_msgs/String com comando
        """
        try:
            command_data = json.loads(msg.data)
            command = command_data.get("command", "").lower()
            
            if command == "set_threshold":
                threshold = command_data.get("threshold", self.obstacle_threshold)
                self.set_obstacle_threshold(threshold)
            else:
                self.get_logger().warn(f"Comando LiDAR desconhecido: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().warn(f"Comando inválido, esperado JSON: {msg.data}")
                
        except Exception as e:
            self.get_logger().error(f"Erro no comando LiDAR: {e}")

    def calculate_obstacles(self, ranges, angles):
        """
        Calcula a presença de obstáculos em diferentes direções.
        
        Args:
            ranges: Array numpy com distâncias
            angles: Array numpy com ângulos em radianos
        
        Returns:
            dict: Dicionário com presença de obstáculos (frente, lados, trás)
        """
        obstacles = {
            "front": False,
            "left": False,
            "right": False,
            "rear": False
        }
        
        # Definir setores angulares (em radianos)
        front_range = (-math.pi/4, math.pi/4)      # -45 a 45 graus
        left_range = (math.pi/4, 3*math.pi/4)      # 45 a 135 graus
        right_range = (-3*math.pi/4, -math.pi/4)   # -135 a -45 graus
        rear_range = (3*math.pi/4, 5*math.pi/4)    # 135 a 225 graus
        
        for i, (distance, angle) in enumerate(zip(ranges, angles)):
            if not (self.min_range <= distance <= self.max_range) or not np.isfinite(distance):
                continue
                
            if distance < self.obstacle_threshold:
                if front_range[0] <= angle <= front_range[1]:
                    obstacles["front"] = True
                
                if left_range[0] <= angle <= left_range[1]:
                    obstacles["left"] = True
                
                if right_range[0] <= angle <= right_range[1]:
                    obstacles["right"] = True
                
                if rear_range[0] <= angle <= rear_range[1]:
                    obstacles["rear"] = True
        
        return obstacles

    def calculate_statistics(self, ranges):
        """
        Calcula estatísticas relevantes para o LiDAR.
        
        Args:
            ranges: Array numpy com distâncias
        
        Returns:
            dict: Dicionário com estatísticas
        """
        valid_ranges = [r for r in ranges if self.min_range <= r <= self.max_range and np.isfinite(r)]
        
        if not valid_ranges:
            return {
                "timestamp": datetime.now().isoformat(),
                "valid_points": 0,
                "min_distance": None,
                "max_distance": None,
                "avg_distance": None,
                "std_distance": None
            }
        
        return {
            "timestamp": datetime.now().isoformat(),
            "valid_points": len(valid_ranges),
            "min_distance": float(np.min(valid_ranges)),
            "max_distance": float(np.max(valid_ranges)),
            "avg_distance": float(np.mean(valid_ranges)),
            "std_distance": float(np.std(valid_ranges))
        }

    def publish_point_vector(self, point_vector):
        """
        Publica o vetor de pontos LiDAR.
        
        Args:
            point_vector: Lista de floats [distancia1, angulo1, distancia2, angulo2, ...]
        """
        try:
            msg = Float32MultiArray()
            msg.data = point_vector
            self.point_vector_publisher.publish(msg)
            self.get_logger().debug(f"Vetor de pontos LiDAR publicado: {len(point_vector)//2} pontos")
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar vetor de pontos: {e}")

    def publish_obstacles(self, obstacles):
        """
        Publica detecções de obstáculos via ROS2 em formato JSON.
        
        Args:
            obstacles: Dicionário com detecções de obstáculos
        """
        try:
            obstacles_msg = String()
            obstacles_msg.data = json.dumps(obstacles)
            self.obstacle_publisher.publish(obstacles_msg)
            self.get_logger().debug("Obstáculos publicados")
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar obstáculos: {e}")
    
    def publish_lidar_statistics(self, statistics):
        """
        Publica estatísticas de LiDAR em formato JSON.
        
        Args:
            statistics: Dicionário com estatísticas
        """
        try:
            stats_msg = String()
            stats_msg.data = json.dumps(statistics)
            self.lidar_stats_publisher.publish(stats_msg)
            self.get_logger().debug("Estatísticas LiDAR publicadas")
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar estatísticas: {e}")

    def publish_lidar_data_and_stats(self):
        """
        Publica o vetor de pontos, estatísticas e obstáculos periodicamente (2Hz).
        """

        if not self.current_laserscan or not self.current_point_vector_data:
            self.get_logger().debug("Nenhum dado LiDAR disponível para publicação")
            return

        try:
            # Publicar vetor de pontos
            self.publish_point_vector(self.current_point_vector_data)

            # Publicar estatísticas
            self.publish_lidar_statistics(self.lidar_statistics)

            # Publicar obstáculos
            self.publish_obstacles(self.obstacle_detections)

        except Exception as e:
            self.get_logger().error(f"Erro ao publicar dados LiDAR: {e}")

    def set_obstacle_threshold(self, threshold):
        """
        Define threshold para detecção de obstáculos.
        
        Args:
            threshold: Threshold em metros
        """
        if threshold > 0:
            self.obstacle_threshold = threshold
            self.get_logger().info(f"Threshold de obstáculos alterado para: {threshold}m")
        else:
            self.get_logger().warn("Threshold deve ser positivo")

def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    lidar_node = LidarNode()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()