"""
lidar_node.py
=================================================================================================
Nó ROS2 para processamento de dados LiDAR.

Recebe dados de varredura laser, detecta obstáculos em setores específicos,
e fornece informações para navegação autônoma e prevenção de colisões.
=================================================================================================
"""

# ==================================================================================================
# IMPORTAÇÕES
# ==================================================================================================

# Bibliotecas ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from drone_inspetor_msgs.msg import LidarMSG, LidarObstaclesMSG

import numpy as np
import json
from datetime import datetime
import math
import time


# ==================== CLASSE LIDARDATA ====================
class LidarData:
    """
    Classe que encapsula os dados do sistema LiDAR para publicação.
    """
    
    def __init__(self):
        self.point_vector: list = []
        self.ground_distance: float = 0.0
    
    def to_msg(self) -> LidarMSG:
        msg = LidarMSG()
        msg.point_vector = self.point_vector
        msg.ground_distance = self.ground_distance
        return msg


class LidarNode(Node):
    """
    Nó ROS2 para processamento de dados LiDAR.
    
    Processa dados de varredura laser, detecta obstáculos em setores específicos,
    e fornece informações para navegação autônoma e prevenção de colisões.
    """
    
    COOLDOWN_SECONDS = 3.0  # Tempo mínimo antes de resetar uma flag
    
    def __init__(self):
        super().__init__("lidar_node")
        self.get_logger().info("Nó LidarNode iniciado.")
        
        # ==================== DECLARAÇÃO DE PARÂMETROS ROS2 ====================
        self.declare_parameter("lidar_data_publish_rate", 5.0)
        self.declare_parameter("obstacles_publish_rate", 5.0)
        
        # Obtém valores dos parâmetros
        self._lidar_data_rate = self.get_parameter("lidar_data_publish_rate").get_parameter_value().double_value
        self._obstacles_rate = self.get_parameter("obstacles_publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Parâmetros: lidar_data={self._lidar_data_rate}Hz, obstacles={self._obstacles_rate}Hz")
        
        # ==================== FLAGS DE OBSTÁCULOS (com cooldown) ====================
        self._obstacle_flags = {
            'have_obstacles_8m': False,
            'have_obstacles_5m': False,
            'have_obstacles_3m': False,
            'have_obstacles_2m': False,
            'have_obstacles_1m': False,
            'have_obstacles_front_90': False,
            'have_obstacles_right_90': False,
            'have_obstacles_back_90': False,
            'have_obstacles_left_90': False,
            'have_obstacles_down_1m': False,
            'have_obstacles_down_05m': False,
        }
        self._flag_set_times = {key: 0.0 for key in self._obstacle_flags}
        
        # ==================== CONFIGURAÇÃO DE QoS ====================
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ==================== SUBSCRIBERS ====================
        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            "/drone_inspetor/externo/lidar/scan",
            self.laserscan_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado: {self.laserscan_subscription.topic_name}")
        
        self.lidar_down_subscription = self.create_subscription(
            LaserScan,
            "/drone_inspetor/externo/lidar_down/scan",
            self.lidar_down_callback,
            qos_sensor_data
        )
        self.get_logger().info(f"Assinado: {self.lidar_down_subscription.topic_name}")
        
        # ==================== PUBLISHERS ====================
        # Publisher para dashboard (vetor de pontos)
        self.lidar_data_publisher = self.create_publisher(
            LidarMSG,
            "/drone_inspetor/lidar_node/lidar_data",
            qos_sensor_data
        )
        self.get_logger().info(f"Publicando: {self.lidar_data_publisher.topic_name}")
        
        # Publisher para drone_node (detecção de obstáculos)
        self.obstacles_publisher = self.create_publisher(
            LidarObstaclesMSG,
            "/drone_inspetor/interno/lidar_node/obstacle_detections",
            qos_reliable
        )
        self.get_logger().info(f"Publicando: {self.obstacles_publisher.topic_name}")
        
        # ==================== TIMERS ====================
        # Timer para publicar lidar_data
        lidar_period = 1.0 / self._lidar_data_rate
        self.lidar_data_timer = self.create_timer(lidar_period, self.publish_lidar_data)
        
        # Timer para publicar obstacle_detections
        obstacles_period = 1.0 / self._obstacles_rate
        self.obstacles_timer = self.create_timer(obstacles_period, self.publish_obstacles)
        
        # ==================== DADOS ====================
        self.lidar_data = LidarData()
        self.current_laserscan = None
        
        # Configurações de range válido
        self.min_range = 0.1
        self.max_range = 12.0
        
        self.get_logger().info("LidarNode inicializado com sucesso.")

    # ==================== MÉTODOS DE FLAGS COM COOLDOWN ====================
    def _set_flag(self, name: str, value: bool):
        """Seta uma flag com cooldown de 3 segundos."""
        current_time = time.time()
        
        if value:
            # Setando para True - atualiza timestamp
            self._obstacle_flags[name] = True
            self._flag_set_times[name] = current_time
        else:
            # Tentando resetar para False - só permite se passaram 3 segundos
            if current_time - self._flag_set_times[name] >= self.COOLDOWN_SECONDS:
                self._obstacle_flags[name] = False
    
    def _reset_all_flags(self):
        """Reseta todas as flags para False, respeitando cooldown."""
        for name in self._obstacle_flags:
            self._set_flag(name, False)
    
    def _to_obstacles_msg(self) -> LidarObstaclesMSG:
        """Converte flags para mensagem ROS."""
        msg = LidarObstaclesMSG()
        msg.have_obstacles_8m = self._obstacle_flags['have_obstacles_8m']
        msg.have_obstacles_5m = self._obstacle_flags['have_obstacles_5m']
        msg.have_obstacles_3m = self._obstacle_flags['have_obstacles_3m']
        msg.have_obstacles_2m = self._obstacle_flags['have_obstacles_2m']
        msg.have_obstacles_1m = self._obstacle_flags['have_obstacles_1m']
        msg.have_obstacles_front_90 = self._obstacle_flags['have_obstacles_front_90']
        msg.have_obstacles_right_90 = self._obstacle_flags['have_obstacles_right_90']
        msg.have_obstacles_back_90 = self._obstacle_flags['have_obstacles_back_90']
        msg.have_obstacles_left_90 = self._obstacle_flags['have_obstacles_left_90']
        msg.have_obstacles_down_1m = self._obstacle_flags['have_obstacles_down_1m']
        msg.have_obstacles_down_05m = self._obstacle_flags['have_obstacles_down_05m']
        return msg

    # ==================== CALLBACKS ====================
    def laserscan_callback(self, msg: LaserScan):
        """
        Callback para processar varredura laser LaserScan.
        Atualiza vetor de pontos e calcula obstáculos por setor.
        """
        try:
            self.current_laserscan = msg
            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
            # Gerar vetor de pontos [dist1, ang1, dist2, ang2, ...]
            point_vector = []
            for i in range(len(ranges)):
                if msg.range_min <= ranges[i] <= msg.range_max and np.isfinite(ranges[i]):
                    point_vector.append(float(ranges[i]))
                    point_vector.append(float(angles[i]))
            
            self.lidar_data.point_vector = point_vector
            
            # Calcular obstáculos por setor
            self._calculate_sector_obstacles(ranges, angles)
            
        except Exception as e:
            self.get_logger().error(f"Erro no processamento de LaserScan: {e}")

    def lidar_down_callback(self, msg: LaserScan):
        """
        Callback para receber dados do LiDAR 1D inferior.
        Atualiza distância ao solo e flags de obstáculo abaixo.
        """
        try:
            if msg.ranges and len(msg.ranges) > 0:
                valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
                if valid_ranges:
                    ground_dist = float(min(valid_ranges))
                    self.lidar_data.ground_distance = ground_dist
                    
                    # Atualiza flags de obstáculo abaixo
                    self._set_flag('have_obstacles_down_1m', ground_dist <= 1.0)
                    self._set_flag('have_obstacles_down_05m', ground_dist <= 0.5)
                    
        except Exception as e:
            self.get_logger().error(f"Erro no LiDAR down: {e}")

    def _calculate_sector_obstacles(self, ranges: np.ndarray, angles: np.ndarray):
        """
        Calcula a presença de obstáculos em cada quadrante (90°).
        
        Quadrantes (em graus, convenção: 0° = frente, positivo = direita):
        - front_90:  -45° a +45°
        - right_90:  +45° a +135°
        - back_90:   +135° a -135° (±180°)
        - left_90:   -45° a -135°
        """
        # Converter ângulos para graus
        angles_deg = np.degrees(angles)
        
        for dist, angle_deg in zip(ranges, angles_deg):
            if not (self.min_range <= dist <= self.max_range) or not np.isfinite(dist):
                continue
            
            # Flags de distância (qualquer ângulo)
            if dist <= 8.0:
                self._set_flag('have_obstacles_8m', True)
            if dist <= 5.0:
                self._set_flag('have_obstacles_5m', True)
            if dist <= 3.0:
                self._set_flag('have_obstacles_3m', True)
            if dist <= 2.0:
                self._set_flag('have_obstacles_2m', True)
            if dist <= 1.0:
                self._set_flag('have_obstacles_1m', True)
            
            # Quadrantes apenas para obstáculos a 1 metro
            if dist > 1.0:
                continue
            
            # Normaliza ângulo para -180 a 180 (usando módulo)
            angle_norm = ((angle_deg + 180) % 360) - 180
            
            # Frente: -45° a +45°
            if -45 <= angle_norm <= 45:
                self._set_flag('have_obstacles_front_90', True)
            
            # Direita: +45° a +135°
            elif 45 < angle_norm <= 135:
                self._set_flag('have_obstacles_right_90', True)
            
            # Esquerda: -45° a -135°
            elif -135 <= angle_norm < -45:
                self._set_flag('have_obstacles_left_90', True)
            
            # Trás: +135° a +180° ou -135° a -180°
            else:
                self._set_flag('have_obstacles_back_90', True)

    # ==================== PUBLISHERS ====================
    def publish_lidar_data(self):
        """Publica os dados consolidados do LiDAR para o dashboard."""
        try:
            msg = self.lidar_data.to_msg()
            self.lidar_data_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar lidar_data: {e}")

    def publish_obstacles(self):
        """Publica detecções de obstáculos para o drone_node."""
        try:
            msg = self._to_obstacles_msg()
            self.obstacles_publisher.publish(msg)
            self.get_logger().debug(
                f"Obstacles: front={msg.have_obstacles_front_90}, 1m={msg.have_obstacles_1m}"
            )
            
            # Reseta flags APÓS publicar para evitar race condition
            self._reset_all_flags()
            
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar obstacles: {e}")


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