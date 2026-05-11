# voando_girando_com_foco.py
# Estado: VOANDO_GIRANDO_COM_FOCO
# Drone girando para apontar para o ponto de foco antes de iniciar o voo GOTO+focus.

import math

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoGirandoComFocoState(State):
    """
    Drone girando para apontar ao ponto de foco antes de partir.

    Calcula dinamicamente o yaw necessário para apontar ao foco a partir
    da posição atual. Quando alinhado e estabilizado, transiciona para
    VOANDO_A_CAMINHO_COM_FOCO.
    """

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context
        state_px4 = context.state_px4

        if state_px4.local_position is None or context.focus_local_position is None:
            return None

        # Calcula yaw dinâmico para o foco
        focus_dx = context.focus_local_position[0] - state_px4.local_position.x
        focus_dy = context.focus_local_position[1] - state_px4.local_position.y
        focus_yaw = math.degrees(math.atan2(focus_dy, focus_dx))
        if focus_yaw > 180:
            focus_yaw -= 360
        elif focus_yaw < -180:
            focus_yaw += 360

        # Em período de estabilização
        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                self.node.get_logger().info(
                    "GOTO+focus: Estabilização completa. Iniciando movimento."
                )
                return DS.VOANDO_A_CAMINHO_COM_FOCO
            else:
                self.node.get_logger().info(
                    "GOTO+focus: Aguardando estabilização...",
                    throttle_duration_sec=1.0,
                )
            return None

        yaw_diff = context.yaw_diff_shortest(state_px4.current_yaw_deg_normalized, focus_yaw)

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                f"GOTO+focus: Yaw para focus alcançado ({focus_yaw:.1f}°). Aguardando estabilização..."
            )

        return None
