# _obstaculo_base.py
# =================================================================================================
# CLASSES BASE DOS ESTADOS DE DESVIO DE OBSTÁCULO
# =================================================================================================
# A sub-FSM de desvio tem 5 fases (sufixos):
#   _OBSTACULO              -> calcula coordenada de desvio, salva destino original
#   _OBSTACULO_GIRANDO_INICIO -> alinha yaw para a direção do desvio
#   _OBSTACULO_DESVIANDO    -> voa até o desvio (pode reentrar em _OBSTACULO se novo obstáculo)
#   _OBSTACULO_GIRANDO_FIM  -> alinha yaw para a direção do destino original
#   _OBSTACULO_DESVIADO     -> verifica se obstáculo sumiu, restaura destino original ou recalcula
#
# Cada fluxo (VOANDO_A_CAMINHO, VOANDO_A_CAMINHO_COM_FOCO, RETORNANDO_A_CAMINHO) tem seus 5
# estados concretos que herdam destas bases e configuram apenas:
#   NEXT_GIRANDO_INICIO_ID, NEXT_DESVIANDO_ID, NEXT_GIRANDO_FIM_ID, NEXT_DESVIADO_ID,
#   NEXT_OBSTACULO_ID, RETOMA_NORMAL_ID (estado de retomada após desvio concluído).
# =================================================================================================

import math

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


# ------------------------------------------------------------------
# Helper: calcula uma coordenada de desvio a partir do estado dos sensores
# de obstáculo. Reaproveita adjust_trajectory_XY/has_obstacle_in_sector.
# Retorna [x, y, z] ou None se não há desvio possível.
# ------------------------------------------------------------------
def calcular_coordenada_desvio(context) -> list | None:
    state_px4 = context.state_px4
    if state_px4.local_position is None or context.target_local_position is None:
        return None

    cur_x = state_px4.local_position.x
    cur_y = state_px4.local_position.y
    cur_z = state_px4.local_position.z
    tgt_x, tgt_y, tgt_z = context.target_local_position

    adjusted_xy, has_obstacle_ahead, no_escape = context.obstacles.adjust_trajectory_XY(
        (cur_x, cur_y),
        (tgt_x, tgt_y),
    )

    if no_escape:
        # Sem saída lateral: tenta subir 2m (Z negativo = sobe em NED)
        return [cur_x, cur_y, cur_z - 2.0]

    if has_obstacle_ahead and adjusted_xy is not None:
        return [adjusted_xy[0], adjusted_xy[1], cur_z]

    return None


# ------------------------------------------------------------------
# Helper: calcula yaw (graus, -180 a 180) que aponta de origem para destino.
# Retorna None se as posições estão muito próximas.
# ------------------------------------------------------------------
def calcular_yaw_para(origem_xy, destino_xy):
    dx = destino_xy[0] - origem_xy[0]
    dy = destino_xy[1] - origem_xy[1]
    if abs(dx) <= 0.1 and abs(dy) <= 0.1:
        return None
    yaw = math.degrees(math.atan2(dy, dx))
    if yaw > 180:
        yaw -= 360
    elif yaw < -180:
        yaw += 360
    return yaw


# ==================================================================
# Base 1: *_OBSTACULO
# ==================================================================
class BaseObstaculoState(State):
    """
    Drone parou por obstáculo na rota. Calcula novo desvio, valida loop,
    salva destino original (se primeira vez) e transita para GIRANDO_INICIO.
    Se loop, aborta para VOANDO_PRONTO via reset_trajectory_vars.
    """

    NEXT_GIRANDO_INICIO_ID = None  # subclasse define

    def on_enter(self) -> None:
        context = self.context
        context.store_static_position()
        context.save_original_target()
        context.register_obstacle_stop_point()

    def on_step(self):
        context = self.context

        candidate = calcular_coordenada_desvio(context)
        if candidate is None:
            self.node.get_logger().warn(
                "_OBSTACULO: Sem coordenada de desvio possível. Abortando para VOANDO_PRONTO."
            )
            context.reset_trajectory_vars()
            return DS.VOANDO_PRONTO

        if context.is_detour_loop(candidate):
            self.node.get_logger().error(
                f"_OBSTACULO: LOOP detectado para candidato "
                f"[{candidate[0]:.2f}, {candidate[1]:.2f}, {candidate[2]:.2f}]. "
                "Abortando para VOANDO_PRONTO."
            )
            context.reset_trajectory_vars()
            return DS.VOANDO_PRONTO

        context.register_detour_point(candidate)

        # Atualiza target_local_position para apontar ao desvio. Origem da
        # trajetória do desvio é a posição atual.
        if context.state_px4.local_position is not None:
            context.origin_local_position = [
                context.state_px4.local_position.x,
                context.state_px4.local_position.y,
                context.state_px4.local_position.z,
            ]
        context.target_local_position = list(candidate)

        # Yaw de direção apontando para o desvio
        if context.state_px4.local_position is not None:
            yaw_norm = calcular_yaw_para(
                (context.state_px4.local_position.x, context.state_px4.local_position.y),
                (candidate[0], candidate[1]),
            )
            if yaw_norm is not None:
                yaw_360 = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
                context.target_direction_yaw_deg_normalized = yaw_norm
                context.target_direction_yaw_deg = yaw_360
                context.target_direction_yaw_rad = math.radians(yaw_norm)
            else:
                context.target_direction_yaw_deg = None
                context.target_direction_yaw_deg_normalized = None
                context.target_direction_yaw_rad = None

        # Yaw final do desvio: não há (vai recalcular no GIRANDO_FIM
        # apontando ao destino original)
        context.target_final_yaw_deg = None
        context.target_final_yaw_deg_normalized = None
        context.target_final_yaw_rad = None

        self.node.get_logger().info(
            f"_OBSTACULO: Desvio calculado em "
            f"[{candidate[0]:.2f}, {candidate[1]:.2f}, {candidate[2]:.2f}]. "
            f"Total de paradas={len(context.obstacle_stop_points)}, "
            f"desvios calculados={len(context.detour_calculated_points)}."
        )
        return self.NEXT_GIRANDO_INICIO_ID


# ==================================================================
# Base 2: *_OBSTACULO_GIRANDO_INICIO
# ==================================================================
class BaseObstaculoGirandoInicioState(State):
    """Gira o drone para alinhar com a direção do desvio antes de partir."""

    NEXT_DESVIANDO_ID = None  # subclasse define

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context

        target_dir_yaw = context.target_direction_yaw_deg
        if target_dir_yaw is None:
            return self.NEXT_DESVIANDO_ID

        yaw_diff = context.yaw_diff_shortest(
            context.state_px4.current_yaw_deg_normalized, target_dir_yaw
        )

        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                return self.NEXT_DESVIANDO_ID
            return None

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()

        return None


# ==================================================================
# Base 3: *_OBSTACULO_DESVIANDO
# ==================================================================
class BaseObstaculoDesviandoState(State):
    """
    Drone voando até a coordenada de desvio. Se detectar novo obstáculo na
    direção do movimento, retorna a *_OBSTACULO para recalcular.
    Se chegar ao desvio (dentro de position_tolerance), avança para *_GIRANDO_FIM.
    """

    NEXT_OBSTACULO_ID = None     # se detectar obstáculo durante desvio
    NEXT_GIRANDO_FIM_ID = None   # quando chegar no desvio

    def on_step(self):
        context = self.context
        state_px4 = context.state_px4

        if state_px4.local_position is None or context.target_local_position is None:
            return None

        # Verifica novo obstáculo na direção do movimento
        if context.target_direction_yaw_rad is not None:
            if context.obstacles.has_obstacle_in_sector(context.target_direction_yaw_rad):
                self.node.get_logger().warn(
                    "_DESVIANDO: Novo obstáculo detectado durante desvio. Recalculando."
                )
                return self.NEXT_OBSTACULO_ID

        cur = state_px4.local_position
        tx, ty, tz = context.target_local_position
        dx, dy, dz = tx - cur.x, ty - cur.y, tz - cur.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= context.position_tolerance:
            context.store_static_position()
            return self.NEXT_GIRANDO_FIM_ID

        return None


# ==================================================================
# Base 4: *_OBSTACULO_GIRANDO_FIM
# ==================================================================
class BaseObstaculoGirandoFimState(State):
    """
    Drone parou no waypoint de desvio. Calcula yaw da posição atual ao
    destino ORIGINAL (saved_target_local_position) e gira para alinhar.
    Quando alinhado, transita para *_DESVIADO.
    """

    NEXT_DESVIADO_ID = None  # subclasse define

    def on_enter(self) -> None:
        context = self.context
        context.yaw_aligned_time = None

        # Calcula yaw para o destino ORIGINAL a partir da posição atual
        if (
            context.state_px4.local_position is None
            or context.saved_target_local_position is None
        ):
            context.target_direction_yaw_deg = None
            context.target_direction_yaw_deg_normalized = None
            context.target_direction_yaw_rad = None
            return

        yaw_norm = calcular_yaw_para(
            (context.state_px4.local_position.x, context.state_px4.local_position.y),
            (context.saved_target_local_position[0], context.saved_target_local_position[1]),
        )
        if yaw_norm is None:
            context.target_direction_yaw_deg = None
            context.target_direction_yaw_deg_normalized = None
            context.target_direction_yaw_rad = None
            return

        yaw_360 = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
        context.target_direction_yaw_deg_normalized = yaw_norm
        context.target_direction_yaw_deg = yaw_360
        context.target_direction_yaw_rad = math.radians(yaw_norm)

    def on_step(self):
        context = self.context

        target_dir_yaw = context.target_direction_yaw_deg
        if target_dir_yaw is None:
            return self.NEXT_DESVIADO_ID

        yaw_diff = context.yaw_diff_shortest(
            context.state_px4.current_yaw_deg_normalized, target_dir_yaw
        )

        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                return self.NEXT_DESVIADO_ID
            return None

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()

        return None


# ==================================================================
# Base 5: *_OBSTACULO_DESVIADO
# ==================================================================
class BaseObstaculoDesviadoState(State):
    """
    Drone alinhado com o destino original. Verifica se a frente está livre.
    Se livre: limpa pilhas e snapshot, restaura destino original, retorna ao
    estado normal de retomada (RETOMA_NORMAL_ID).
    Se obstáculo persistir: recalcula via *_OBSTACULO.
    """

    RETOMA_NORMAL_ID = None       # subclasse define
    NEXT_OBSTACULO_ID = None      # se obstáculo persistir

    def on_step(self):
        context = self.context

        if context.target_direction_yaw_rad is not None:
            if context.obstacles.has_obstacle_in_sector(context.target_direction_yaw_rad):
                self.node.get_logger().warn(
                    "_DESVIADO: Obstáculo persiste na direção do destino original. Recalculando."
                )
                return self.NEXT_OBSTACULO_ID

        # Caminho livre: restaura destino original e limpa estado de desvio.
        context.restore_original_target()
        context.clear_obstacle_avoidance_state()
        self.node.get_logger().info(
            "_DESVIADO: Caminho livre. Destino original restaurado, retomando rota."
        )
        return self.RETOMA_NORMAL_ID
