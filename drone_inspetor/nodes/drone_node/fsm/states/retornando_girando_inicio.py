# retornando_girando_inicio.py
# Estado: RETORNANDO_GIRANDO_INICIO
# Drone girando para apontar em direção ao HOME antes de retornar (RTL).

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class RetornandoGirandoInicioState(State):
    """
    Drone girando para a direção do HOME no início do RTL.

    Fluxo:
    1. Se target_direction_yaw_deg é None → permanece aguardando (dados ainda não disponíveis).
    2. Se yaw_aligned_time está definido → verifica tempo de estabilização.
    3. Se |yaw_diff| <= tolerância → define yaw_aligned_time.
    """

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context

        if context.target_direction_yaw_deg is None:
            return None

        # Em período de estabilização
        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                self.node.get_logger().info(
                    "RTL: Estabilização completa. Iniciando voo para HOME."
                )
                return DS.RETORNANDO_A_CAMINHO
            else:
                self.node.get_logger().info(
                    "RTL: Aguardando estabilização...",
                    throttle_duration_sec=1.0,
                )
            return None

        yaw_diff = context.yaw_diff_shortest(
            context.state_px4.current_yaw_deg_normalized, context.target_direction_yaw_deg
        )

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                f"RTL: Yaw de direção alcançado ({context.target_direction_yaw_deg:.1f}°). "
                "Aguardando estabilização..."
            )

        return None
