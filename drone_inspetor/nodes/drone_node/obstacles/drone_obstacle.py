# =================================================================================================
# drone_obstacle.py
# =================================================================================================
# ARMAZENAMENTO CENTRAL DE FLAGS DE OBSTÁCULO DO DRONE
# =================================================================================================
# Classe principal do subsistema de obstáculos. Agrega flags de múltiplas fontes
# (LiDAR + câmera depth) usando OR — basta um sensor detectar para a flag ser ativada.
#
# Responsabilidades:
#   1. Manter instâncias de LidarObstacle e DepthObstacle.
#   2. Delegar o processamento de mensagens para cada fonte.
#   3. Expor properties agregadas (OR entre fontes) para consulta.
#
# NÃO contém lógica de cálculo de desvio — essa responsabilidade é do
# módulo obstacle_avoidance.py.
# =================================================================================================

from typing import TYPE_CHECKING

from drone_inspetor.nodes.drone_node.obstacles.lidar_obstacles import LidarObstacle
from drone_inspetor.nodes.drone_node.obstacles.depth_obstacles import DepthObstacle

if TYPE_CHECKING:
    from drone_inspetor_msgs.msg import LidarMSG, ObstaclesMSG


class DroneObstacle:
    """
    Armazenamento central de flags de obstáculo do drone.

    Agrega informações de múltiplas fontes (LiDAR 360° + câmera depth frontal).
    As properties públicas retornam OR entre as fontes — basta um sensor
    detectar para a flag ser considerada ativa.

    Flags armazenadas:
        - Distância:  have_obstacles_8m, 5m, 3m, 2m, 1m
        - Quadrante:  have_obstacles_front_90, right_90, back_90, left_90
        - Inferior:   have_obstacles_down_1m, down_05m
    """

    def __init__(self):
        # Fontes de detecção de obstáculos.
        self._lidar = LidarObstacle()
        self._depth = DepthObstacle()

    # =============================================================================================
    # Atualização por fonte
    # =============================================================================================

    def update_from_lidar(self, msg: 'LidarMSG') -> None:
        """
        Processa dados brutos do LiDAR e atualiza flags de obstáculo.

        O LidarObstacle converte o point_vector (pares distância/ângulo) e
        ground_distance em flags booleanas para todas as 11 categorias.

        Args:
            msg: LidarMSG com point_vector e ground_distance.
        """
        self._lidar.process(msg)

    def update_from_depth(self, msg: 'ObstaclesMSG') -> None:
        """
        Atualiza flags a partir de ObstaclesMSG da câmera de profundidade.

        O DepthObstacle extrai apenas as flags que a câmera frontal cobre
        (distância + quadrante frontal). Demais flags não são afetadas.

        Args:
            msg: ObstaclesMSG publicada pelo depth_node.
        """
        self._depth.update(msg)

    # =============================================================================================
    # Properties agregadas (OR entre fontes)
    # =============================================================================================

    @property
    def have_obstacles_8m(self) -> bool:
        """Obstáculo detectado em raio de 8 metros (qualquer fonte)."""
        return (self._lidar.flags.get('have_obstacles_8m', False)
                or self._depth.flags.get('have_obstacles_8m', False))

    @property
    def have_obstacles_5m(self) -> bool:
        """Obstáculo detectado em raio de 5 metros (qualquer fonte)."""
        return (self._lidar.flags.get('have_obstacles_5m', False)
                or self._depth.flags.get('have_obstacles_5m', False))

    @property
    def have_obstacles_3m(self) -> bool:
        """Obstáculo detectado em raio de 3 metros (qualquer fonte)."""
        return (self._lidar.flags.get('have_obstacles_3m', False)
                or self._depth.flags.get('have_obstacles_3m', False))

    @property
    def have_obstacles_2m(self) -> bool:
        """Obstáculo detectado em raio de 2 metros (qualquer fonte)."""
        return (self._lidar.flags.get('have_obstacles_2m', False)
                or self._depth.flags.get('have_obstacles_2m', False))

    @property
    def have_obstacles_1m(self) -> bool:
        """Obstáculo detectado em raio de 1 metro (qualquer fonte)."""
        return (self._lidar.flags.get('have_obstacles_1m', False)
                or self._depth.flags.get('have_obstacles_1m', False))

    @property
    def have_obstacles_front_90(self) -> bool:
        """Obstáculo no quadrante frontal (-45° a +45°), qualquer fonte."""
        return (self._lidar.flags.get('have_obstacles_front_90', False)
                or self._depth.flags.get('have_obstacles_front_90', False))

    @property
    def have_obstacles_right_90(self) -> bool:
        """Obstáculo no quadrante direito (+45° a +135°). Apenas LiDAR."""
        return self._lidar.flags.get('have_obstacles_right_90', False)

    @property
    def have_obstacles_back_90(self) -> bool:
        """Obstáculo no quadrante traseiro (±135° a ±180°). Apenas LiDAR."""
        return self._lidar.flags.get('have_obstacles_back_90', False)

    @property
    def have_obstacles_left_90(self) -> bool:
        """Obstáculo no quadrante esquerdo (-45° a -135°). Apenas LiDAR."""
        return self._lidar.flags.get('have_obstacles_left_90', False)

    @property
    def have_obstacles_down_1m(self) -> bool:
        """Obstáculo abaixo a até 1 metro. Apenas LiDAR inferior."""
        return self._lidar.flags.get('have_obstacles_down_1m', False)

    @property
    def have_obstacles_down_05m(self) -> bool:
        """Obstáculo abaixo a até 0.5 metro. Apenas LiDAR inferior."""
        return self._lidar.flags.get('have_obstacles_down_05m', False)
