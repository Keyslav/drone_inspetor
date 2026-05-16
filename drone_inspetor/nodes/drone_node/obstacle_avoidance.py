# =================================================================================================
# obstacle_avoidance.py — Helpers de evasão e consulta de obstáculos
# =================================================================================================
# Funções PURAS (sem dependência ROS) para consulta de obstáculos e cálculo de evasão.
# Usadas pelo estado DESLOCANDO da DeslocamentoFSM e pelo Trajectory (cap de velocidade).
#
# Decisões de design:
#   - Todas as funções recebem DroneObstacle como parâmetro (sem estado próprio).
#   - O cálculo do ponto de desvio é STATELESS. A persistência ("já tentei este lugar?")
#     fica em TargetStack._desvio_history, consultada via `target_stack.is_loop_candidate()`.
#   - O sentido lateral (esquerda/direita) é escolhido com base em quais quadrantes
#     do DroneObstacle estão LIVRES. Se ambos livres, prefere direita (regra da mão direita).
#   - Se nenhum lado livre, retorna `None` — caller deve transitar para hover/abortar.
# =================================================================================================

import math
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.obstacles.drone_obstacle import DroneObstacle


# Distância (m) que o ponto de desvio fica do drone, lateral à direção do movimento.
# Suficiente para sair da projeção do sensor de proximidade, sem ficar longe demais.
DEFAULT_DETOUR_DISTANCE: float = 3.0


# =================================================================================================
# Consultas de obstáculo (leitura das flags do DroneObstacle)
# =================================================================================================

def has_obstacle_in_sector(obstacles: 'DroneObstacle', angle_rad: float) -> bool:
    """
    Verifica se há obstáculo no quadrante (90°) correspondente ao ângulo.

    Quadrantes (em graus, convenção: 0° = frente):
        - Frente:   -45° a +45°
        - Direita:  +45° a +135°
        - Esquerda: -135° a -45°
        - Trás:     +135° a ±180° / -135° a -180°

    Args:
        obstacles: Instância de DroneObstacle com flags agregadas.
        angle_rad: Ângulo em radianos a verificar.

    Returns:
        True se a flag do quadrante correspondente estiver ativa.
    """
    from drone_inspetor.common.math_utils import normalize_yaw_deg
    angle_deg = normalize_yaw_deg(math.degrees(angle_rad))

    if -45 <= angle_deg <= 45:
        return obstacles.have_obstacles_front_90
    elif 45 < angle_deg <= 135:
        return obstacles.have_obstacles_right_90
    elif -135 <= angle_deg < -45:
        return obstacles.have_obstacles_left_90
    else:
        return obstacles.have_obstacles_back_90


def get_velocity_cap_from_obstacles(obstacles: 'DroneObstacle', vc: float) -> float:
    """
    Retorna o teto de velocidade (m/s) imposto pelos obstáculos detectados.

    Cascata de redução proporcional à proximidade:
        obstáculo a 8m → vc capado em 3.0
        obstáculo a 5m → vc capado em 2.0
        obstáculo a 3m → vc capado em 1.0
        obstáculo a 2m → vc capado em 0.5
        obstáculo a 1m → para (0.0)

    Args:
        obstacles: Instância de DroneObstacle com flags agregadas.
        vc: Velocidade de cruzeiro nominal (m/s).

    Returns:
        Velocidade-alvo permitida (m/s). Sem obstáculo retorna `vc`.
    """
    if obstacles.have_obstacles_1m:
        return 0.0
    if obstacles.have_obstacles_2m:
        return min(vc, 0.5)
    if obstacles.have_obstacles_3m:
        return min(vc, 1.0)
    if obstacles.have_obstacles_5m:
        return min(vc, 2.0)
    if obstacles.have_obstacles_8m:
        return min(vc, 3.0)
    return vc


def calculate_trajectory_adjustment(
    obstacles: 'DroneObstacle',
    current_x: float, current_y: float, current_z: float,
    next_x: float, next_y: float, next_z: float,
) -> tuple:
    """
    Calcula o ajuste necessário da trajetória por conta de obstáculos a 1m ou 0.5m.
    Chamada APENAS quando há obstáculo próximo (verificado antes da chamada).

    NOTA: Este método é stateless — não armazena resultado.
    A persistência de desvios é gerenciada pela WaypointStack.

    Args:
        obstacles: Instância de DroneObstacle com flags agregadas.
        current_x, current_y, current_z: Posição atual.
        next_x, next_y, next_z: Próxima posição desejada.

    Returns:
        (adjusted_x, adjusted_y, adjusted_z)
    """
    dz = next_z - current_z
    is_descending = dz > 0  # NED: positivo = para baixo

    need_adjust_xy = obstacles.have_obstacles_1m
    need_block_z = (is_descending and obstacles.have_obstacles_down_05m)

    adjusted_next_x = next_x
    adjusted_next_y = next_y
    adjusted_next_z = next_z

    if not need_adjust_xy and not need_block_z:
        return (next_x, next_y, next_z)

    if need_block_z:
        adjusted_next_z = current_z

    if need_adjust_xy:
        adjusted_xy, has_obstacle_ahead, no_escape = _adjust_trajectory_xy(
            obstacles,
            (current_x, current_y),
            (next_x, next_y),
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


def _adjust_trajectory_xy(
    obstacles: 'DroneObstacle',
    current_pos_xy: tuple,
    next_pos_xy: tuple,
) -> tuple:
    """
    Ajusta a trajetória XY com base em obstáculos detectados.
    Se obstáculos à frente, desvia lateralmente (esquerda/direita).

    Args:
        obstacles: Instância de DroneObstacle com flags agregadas.
        current_pos_xy: (current_x, current_y) posição atual.
        next_pos_xy: (next_x, next_y) próxima posição calculada.

    Returns:
        ((adjusted_x, adjusted_y) ou None, has_obstacle_ahead, no_escape)
    """
    current_x, current_y = current_pos_xy
    next_x, next_y = next_pos_xy

    # Calcula direção do movimento.
    dx = next_x - current_x
    dy = next_y - current_y
    direction_rad = math.atan2(dy, dx)

    # Verifica obstáculos no arco frontal e laterais.
    has_obstacle_ahead = has_obstacle_in_sector(obstacles, direction_rad)
    has_obstacle_left = has_obstacle_in_sector(obstacles, direction_rad + math.radians(90))
    has_obstacle_right = has_obstacle_in_sector(obstacles, direction_rad - math.radians(90))
    no_escape = has_obstacle_ahead and has_obstacle_left and has_obstacle_right

    if not has_obstacle_ahead:
        return None, False, False

    if not has_obstacle_right:
        # Sem obstáculo à direita — desvia 1m para direita.
        right_direction = direction_rad - math.radians(90)
        next_x = current_x + math.cos(right_direction) * 1.0
        next_y = current_y + math.sin(right_direction) * 1.0
    elif not has_obstacle_left:
        # Sem obstáculo à esquerda — desvia 1m para esquerda.
        left_direction = direction_rad + math.radians(90)
        next_x = current_x + math.cos(left_direction) * 1.0
        next_y = current_y + math.sin(left_direction) * 1.0

    if no_escape:
        return None, True, True

    return (next_x, next_y), True, False


# =================================================================================================
# Cálculo de pontos de desvio (funções originais)
# =================================================================================================

def calcular_yaw_para(from_pos: tuple, to_pos: tuple) -> float:
    """
    Calcula o yaw (em graus, intervalo [-180, 180]) que aponta de `from_pos` para `to_pos`.

    Args:
        from_pos: (x, y) ou (x, y, z) — origem.
        to_pos:   (x, y) ou (x, y, z) — destino.

    Returns:
        Yaw em graus normalizado, no intervalo [-180, 180].
    """
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    yaw_deg = math.degrees(math.atan2(dy, dx))
    # Normalização para [-180, 180].
    return ((yaw_deg + 180.0) % 360.0) - 180.0


def calcular_coordenada_desvio(
    current_pos: tuple,
    direction_yaw_rad: float,
    obstacles: 'DroneObstacle',
    distance: float = DEFAULT_DETOUR_DISTANCE,
) -> Optional[tuple]:
    """
    Calcula um ponto lateral para contornar um obstáculo detectado na direção de movimento.

    Algoritmo:
        1. Verifica quais lados (esquerda/direita do drone, relativo ao seu yaw atual)
           estão LIVRES de obstáculos.
        2. Se DIREITA livre: desvia 90° à direita por `distance` metros.
        3. Senão, se ESQUERDA livre: desvia 90° à esquerda por `distance` metros.
        4. Se ambos os lados estão bloqueados: retorna None (sem escape lateral).

    Args:
        current_pos:        (x, y, z) posição atual em NED.
        direction_yaw_rad:  Yaw atual de movimento do drone (rad). Usado como
                            referência para "esquerda"/"direita".
        obstacles:          Instância DroneObstacle (consulta sectoral).
        distance:           Distância em metros do ponto de desvio até `current_pos`.

    Returns:
        Tupla (x, y, z) do ponto de desvio, ou None se não houver lado livre.
        O z é mantido igual ao current_pos[2] — desvios são puramente horizontais.
    """
    # Direções absolutas de esquerda/direita relativas ao yaw atual.
    yaw_direita = direction_yaw_rad - math.radians(90)
    yaw_esquerda = direction_yaw_rad + math.radians(90)

    obstaculo_direita = has_obstacle_in_sector(obstacles, yaw_direita)
    obstaculo_esquerda = has_obstacle_in_sector(obstacles, yaw_esquerda)

    if not obstaculo_direita:
        # Prefere direita por convenção (mantém regra da mão direita).
        desvio_x = current_pos[0] + math.cos(yaw_direita) * distance
        desvio_y = current_pos[1] + math.sin(yaw_direita) * distance
        return (desvio_x, desvio_y, current_pos[2])

    if not obstaculo_esquerda:
        desvio_x = current_pos[0] + math.cos(yaw_esquerda) * distance
        desvio_y = current_pos[1] + math.sin(yaw_esquerda) * distance
        return (desvio_x, desvio_y, current_pos[2])

    # Cercado: sem escape lateral. Caller decide o que fazer (parar, subir, abortar).
    return None
