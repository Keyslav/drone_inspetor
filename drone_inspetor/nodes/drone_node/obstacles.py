# obstacles.py
# =================================================================================================
# DETECÇÃO E EVITAÇÃO DE OBSTÁCULOS (DRONE)
# =================================================================================================
# Mescla flags de proximidade vindas de múltiplos sensores (lidar e depth) — cada fonte mantém
# seu próprio buffer e o consumo das flags faz OR entre elas. Calcula ajustes de trajetória
# para evitar colisões.
# =================================================================================================

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_inspetor_msgs.msg import ObstaclesMSG
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode


_FLAG_FIELDS = (
    'have_obstacles_8m',
    'have_obstacles_5m',
    'have_obstacles_3m',
    'have_obstacles_2m',
    'have_obstacles_1m',
    'have_obstacles_front_90',
    'have_obstacles_right_90',
    'have_obstacles_back_90',
    'have_obstacles_left_90',
    'have_obstacles_down_1m',
    'have_obstacles_down_05m',
)


class _ObstacleFlags:
    """Buffer simples de flags de obstáculo de uma única fonte."""

    __slots__ = _FLAG_FIELDS

    def __init__(self):
        for f in _FLAG_FIELDS:
            setattr(self, f, False)

    def update(self, msg: 'ObstaclesMSG'):
        for f in _FLAG_FIELDS:
            setattr(self, f, getattr(msg, f))


class DroneObstacles:
    """
    Gerencia detecção de obstáculos a partir de múltiplas fontes (lidar + depth).
    Cada fonte mantém suas próprias flags; as propriedades públicas retornam o
    OR entre as fontes — basta um sensor detectar para o desvio ser disparado.
    """

    def __init__(self, node: 'DroneNode'):
        """
        Inicializa o sistema de detecção de obstáculos.

        Args:
            node: Referência ao DroneNode para logging e acesso ao estado
        """
        self.node = node

        # Buffers por fonte
        self._lidar = _ObstacleFlags()
        self._depth = _ObstacleFlags()

    # ----- Atualização por fonte -----

    def update_from_lidar(self, msg: 'ObstaclesMSG'):
        """Atualiza buffer de flags vindas do lidar_node."""
        self._lidar.update(msg)

    def update_from_depth(self, msg: 'ObstaclesMSG'):
        """Atualiza buffer de flags vindas do depth_node."""
        self._depth.update(msg)

    # ----- Propriedades agregadas (OR entre fontes) -----

    @property
    def have_obstacles_8(self) -> bool:
        return self._lidar.have_obstacles_8m or self._depth.have_obstacles_8m

    @property
    def have_obstacles_5(self) -> bool:
        return self._lidar.have_obstacles_5m or self._depth.have_obstacles_5m

    @property
    def have_obstacles_3(self) -> bool:
        return self._lidar.have_obstacles_3m or self._depth.have_obstacles_3m

    @property
    def have_obstacles_2(self) -> bool:
        return self._lidar.have_obstacles_2m or self._depth.have_obstacles_2m

    @property
    def have_obstacles_1(self) -> bool:
        return self._lidar.have_obstacles_1m or self._depth.have_obstacles_1m

    @property
    def have_obstacles_front_90(self) -> bool:
        return self._lidar.have_obstacles_front_90 or self._depth.have_obstacles_front_90

    @property
    def have_obstacles_right_90(self) -> bool:
        return self._lidar.have_obstacles_right_90 or self._depth.have_obstacles_right_90

    @property
    def have_obstacles_back_90(self) -> bool:
        return self._lidar.have_obstacles_back_90 or self._depth.have_obstacles_back_90

    @property
    def have_obstacles_left_90(self) -> bool:
        return self._lidar.have_obstacles_left_90 or self._depth.have_obstacles_left_90

    @property
    def have_down_obstacles_1(self) -> bool:
        return self._lidar.have_obstacles_down_1m or self._depth.have_obstacles_down_1m

    @property
    def have_down_obstacles_05(self) -> bool:
        return self._lidar.have_obstacles_down_05m or self._depth.have_obstacles_down_05m

    def has_obstacle_in_sector(self, angle_rad: float) -> bool:
        """
        Verifica se há obstáculo no quadrante (90°) correspondente ao ângulo.

        Args:
            angle_rad: Ângulo em radianos

        Returns:
            bool: True se a flag do quadrante correspondente estiver ativa
        """
        # Converte para graus e normaliza para -180 a 180
        from drone_inspetor.common.math_utils import normalize_yaw_deg
        angle_deg = normalize_yaw_deg(math.degrees(angle_rad))

        # Frente: -45° a +45°
        if -45 <= angle_deg <= 45:
            return self.have_obstacles_front_90

        # Direita: +45° a +135°
        elif 45 < angle_deg <= 135:
            return self.have_obstacles_right_90

        # Esquerda: -45° a -135°
        elif -135 <= angle_deg < -45:
            return self.have_obstacles_left_90

        # Trás: +135° a +180° ou -135° a -180°
        else:
            return self.have_obstacles_back_90

    def get_velocity_cap_from_obstacles(self, vc: float) -> float:
        """
        Retorna o teto de velocidade (m/s) imposto pelos obstáculos atualmente detectados.

        Substitui a antiga `calculate_trajectory_magnitude_reduction`, que mexia diretamente
        em distância. Agora a redução vira um cap de velocidade-alvo consumido pelo
        TrajectoryProfile: quando v_des excede esse cap, o profile entra em OBSTACLE_BRAKE
        com aceleração `-ao` até alinhar.

        Cascata equivalente ao comportamento legado (cruzeiro proporcional à proximidade):
            obstáculo a 8m → vc capado em 3.0
            obstáculo a 5m → vc capado em 2.0
            obstáculo a 3m → vc capado em 1.0
            obstáculo a 2m → vc capado em 0.5
            obstáculo a 1m → para (0.0)

        Args:
            vc: Velocidade de cruzeiro nominal (m/s).

        Returns:
            float: Velocidade-alvo permitida (m/s). Sem obstáculo retorna `vc`.
        """
        if self.have_obstacles_1:
            return 0.0
        if self.have_obstacles_2:
            return min(vc, 0.5)
        if self.have_obstacles_3:
            return min(vc, 1.0)
        if self.have_obstacles_5:
            return min(vc, 2.0)
        if self.have_obstacles_8:
            return min(vc, 3.0)
        return vc

    def calculate_trajectory_adjustment(self, current_x: float, current_y: float, current_z: float,
                                         next_x: float, next_y: float, next_z: float) -> tuple:
        """
        Calcula o ajuste necessário da trajetória por conta de obstáculos a 1m ou 0.5m.
        Chamada APENAS quando há obstáculo próximo (verificado antes da chamada).

        NOTA: Este método agora é stateless — não armazena resultado no contexto.
        A persistência de desvios é gerenciada pela WaypointStack.

        Args:
            current_x, current_y, current_z: Posição atual
            next_x, next_y, next_z: Próxima posição desejada

        Returns:
            tuple: (adjusted_x, adjusted_y, adjusted_z)
        """
        dz = next_z - current_z
        is_descending = dz > 0  # NED: positivo = para baixo

        need_adjust_xy = self.have_obstacles_1
        need_block_z = (is_descending and self.have_down_obstacles_05)

        adjusted_next_x = next_x
        adjusted_next_y = next_y
        adjusted_next_z = next_z

        if not need_adjust_xy and not need_block_z:
            return (next_x, next_y, next_z)

        if need_block_z:
            adjusted_next_z = current_z

        if need_adjust_xy:
            adjusted_xy, has_obstacle_ahead, no_escape = self.adjust_trajectory_XY(
                (current_x, current_y),
                (next_x, next_y)
            )

            if has_obstacle_ahead:

                if no_escape:
                    adjusted_next_x = current_x
                    adjusted_next_y = current_y
                    if need_block_z:
                        adjusted_next_z = current_z - 50  # NED: negativo = sobe
                else:
                    adjusted_next_x = adjusted_xy[0]
                    adjusted_next_y = adjusted_xy[1]

        return (adjusted_next_x, adjusted_next_y, adjusted_next_z)

    def adjust_trajectory_XY(self, current_pos_XY: tuple, next_pos_XY: tuple) -> tuple:
        """
        Ajusta a trajetória com base em obstáculos detectados.
        Se obstáculos à frente, desvia lateralmente (esquerda/direita).

        Args:
            current_pos_XY: (current_x, current_y) posição atual
            next_pos_XY: (next_x, next_y) próxima posição calculada

        Returns:
            tuple: ((adjusted_x, adjusted_y) ou None, has_obstacle_ahead, no_escape)
        """
        current_x, current_y = current_pos_XY
        next_x, next_y = next_pos_XY

        # Calcula direção do movimento
        dx = next_x - current_x
        dy = next_y - current_y
        direction_rad = math.atan2(dy, dx)

        # Verifica obstáculos no arco frontal (direção ±50°)
        has_obstacle_ahead = self.has_obstacle_in_sector(direction_rad)
        has_obstacle_left = self.has_obstacle_in_sector(direction_rad + math.radians(90))
        has_obstacle_right = self.has_obstacle_in_sector(direction_rad - math.radians(90))
        no_escape = has_obstacle_ahead and has_obstacle_left and has_obstacle_right

        if not has_obstacle_ahead:
            return None, False, False

        if not has_obstacle_right:
            # Sem obstáculo à direita - pode desviar 1m para direita na direção do movimento
            right_direction = direction_rad - math.radians(90)
            next_x = current_x + math.cos(right_direction) * 1.0
            next_y = current_y + math.sin(right_direction) * 1.0
        elif not has_obstacle_left:
            # Sem obstáculo à esquerda - pode desviar 1m para esquerda na direção do movimento
            left_direction = direction_rad + math.radians(90)
            next_x = current_x + math.cos(left_direction) * 1.0
            next_y = current_y + math.sin(left_direction) * 1.0

        if no_escape:
            return None, True, True

        return (next_x, next_y), True, False
