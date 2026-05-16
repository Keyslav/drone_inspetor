# =================================================================================================
# DeslocandoState — estado da DeslocamentoFSM
# =================================================================================================
# DeslocamentoFSMDescription.DESLOCANDO
#
# Significado: drone transladando até o target ativo (target_stack.current). A trajetória
# é gerada pelo TrajectoryProfile (mixin DroneTrajectoryMixin do nó); este estado apenas:
#     - Observa a distância restante ao target.
#     - Monitora obstáculos na direção do movimento e empilha desvios quando necessário.
#
# Transições:
#     distancia <= position_tolerance → GIRANDO_FIM.
#     Obstáculo na direção do movimento → empilha desvio na TargetStack e retorna
#                                           GIRANDO_INICIO (alinhamento com o novo topo).
#     Pilha esvaziou (cancelamento externo) → PLANANDO.
# =================================================================================================

import math

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS
from drone_inspetor.nodes.drone_node.obstacle_avoidance import (
    has_obstacle_in_sector,
    calcular_coordenada_desvio,
)


class DeslocandoState(BaseState):
    """Transladando até o target. Monitora obstáculos e empilha desvios em rota."""

    def on_enter(self) -> None:
        ctx = self.context
        ctx.trajectory_start_time = ctx.now()
        # Calcula a distância inicial até o target (usada para feedback de progresso).
        target = ctx.target_stack.current
        if target is not None and ctx.state_px4.local_position is not None:
            cur = ctx.state_px4.local_position
            tx, ty, tz = target.local_position
            ctx.initial_distance_to_target = math.sqrt(
                (tx - cur.x) ** 2 + (ty - cur.y) ** 2 + (tz - cur.z) ** 2
            )
            self.node.get_logger().info(
                f"DeslocamentoFSM: DESLOCANDO → distância inicial {ctx.initial_distance_to_target:.2f}m."
            )

    def on_step(self):
        ctx = self.context
        target = ctx.target_stack.current

        # Pilha esvaziou (cancelamento externo): volta ao idle.
        if target is None:
            return TS.PLANANDO

        # Sem posição local ainda: aguarda.
        if ctx.state_px4.local_position is None:
            return None

        cur = ctx.state_px4.local_position

        # ---- (1) Detecção de obstáculo na direção do movimento ----
        # Se há obstáculo, calcula ponto de desvio lateral e empilha sobre o target atual.
        # A DeslocamentoFSM volta para GIRANDO_INICIO para alinhar com o novo topo (o desvio).
        obstacles = self.node.obstacles
        if (
            target.direction_yaw_rad is not None
            and has_obstacle_in_sector(obstacles, target.direction_yaw_rad)
        ):
            self.node.get_logger().warn(
                "DeslocamentoFSM: obstáculo na direção do movimento. Calculando desvio."
            )

            desvio = calcular_coordenada_desvio(
                current_pos=(cur.x, cur.y, cur.z),
                direction_yaw_rad=target.direction_yaw_rad,
                obstacles=obstacles,
            )

            if desvio is None:
                # Sem lado livre: hover na posição atual; recua o objetivo até obstáculo sumir.
                self.node.get_logger().error(
                    "DeslocamentoFSM: obstáculo sem escape lateral. Mantendo hover."
                )
                ctx.store_static_position()
                return TS.PLANANDO

            if ctx.target_stack.is_loop_candidate(list(desvio)):
                self.node.get_logger().error(
                    "DeslocamentoFSM: desvio caracteriza loop. Cancelando navegação."
                )
                ctx.reset_trajectory_vars()
                return TS.PLANANDO

            # Empilha o desvio sobre o target atual: DeslocamentoFSM passa a operar nele.
            ctx.target_stack.push_desvio(local_pos=list(desvio))
            return TS.GIRANDO_INICIO

        # ---- (2) Checagem de chegada no target ----
        tx, ty, tz = target.local_position
        distancia = math.sqrt(
            (tx - cur.x) ** 2 + (ty - cur.y) ** 2 + (tz - cur.z) ** 2
        )

        if distancia <= ctx.position_tolerance:
            yaw_info = (
                f" (yaw final: {target.final_yaw_deg:.1f}°)"
                if target.final_yaw_deg is not None
                else " (sem yaw final)"
            )
            self.node.get_logger().info(
                f"DeslocamentoFSM: target alcançado{yaw_info}. → GIRANDO_FIM."
            )
            return TS.GIRANDO_FIM

        return None
