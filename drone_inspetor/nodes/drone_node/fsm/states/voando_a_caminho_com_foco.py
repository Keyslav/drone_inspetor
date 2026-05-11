# voando_a_caminho_com_foco.py
# Estado: VOANDO_A_CAMINHO_COM_FOCO
# Drone voando para o target enquanto aponta para o ponto de foco.

import math

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoACaminhoComFocoState(State):
    """
    Drone em trânsito para o target com câmera apontando para o foco.

    Ao chegar dentro da tolerância:
    - Se yaw_aligned_time ainda não definido → inicia contagem de estabilização.
    - Se yaw_aligned_time definido e elapsed >= delay → reseta e vai para VOANDO_PRONTO.
    Detecção de obstáculo na direção do movimento → VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO.
    """

    def on_step(self):
        context = self.context
        state_px4 = context.state_px4

        if state_px4.local_position is None or context.target_local_position is None:
            return None

        # Detecção de obstáculo na direção do movimento → entra na sub-FSM de desvio.
        # Usa o yaw de direção do movimento (não o yaw do foco) p/ checar o setor à frente.
        cur = state_px4.local_position
        _dx = context.target_local_position[0] - cur.x
        _dy = context.target_local_position[1] - cur.y
        if abs(_dx) > 0.1 or abs(_dy) > 0.1:
            _direction_rad = math.atan2(_dy, _dx)
            if context.obstacles.has_obstacle_in_sector(_direction_rad):
                self.node.get_logger().warn(
                    "VOANDO_A_CAMINHO_COM_FOCO: Obstáculo detectado na direção do movimento."
                )
                return DS.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO

        tx, ty, tz = context.target_local_position

        dx = tx - cur.x
        dy = ty - cur.y
        dz = tz - cur.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= context.position_tolerance:
            context.waypoint_stack.complete_current()

            # Em período de estabilização
            if context.yaw_aligned_time is not None:
                elapsed = context.now() - context.yaw_aligned_time
                if elapsed >= context.yaw_stabilization_delay:
                    context.yaw_aligned_time = None
                    self.node.get_logger().info(
                        "GOTO+focus: Estabilização completa. Trajetória finalizada."
                    )
                    context.reset_trajectory_vars()
                    return DS.VOANDO_PRONTO
                else:
                    self.node.get_logger().info(
                        "GOTO+focus: Chegou ao destino. Aguardando estabilização...",
                        throttle_duration_sec=1.0,
                    )
                return None

            # Chegou ao destino — inicia estabilização
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                "GOTO+focus: Posição alvo alcançada. Aguardando estabilização..."
            )

        return None
