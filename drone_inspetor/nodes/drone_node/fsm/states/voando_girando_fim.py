# voando_girando_fim.py
# Estado: VOANDO_GIRANDO_FIM
# Drone girando para o yaw final após chegar ao target.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoGirandoFimState(State):
    """
    Drone girando para o yaw final no destino.

    Fluxo:
    1. Se target_final_yaw_deg é None → reseta variáveis e vai para VOANDO_PRONTO.
    2. Se yaw_aligned_time está definido → verifica tempo de estabilização.
    3. Se |yaw_diff| <= tolerância → define yaw_aligned_time.
    """

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context

        if context.target_final_yaw_deg is None:
            context.reset_trajectory_vars()
            return DS.VOANDO_PRONTO

        # Em período de estabilização
        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                self.node.get_logger().info("Estabilização completa. Trajetória finalizada.")
                context.reset_trajectory_vars()
                return DS.VOANDO_PRONTO
            else:
                self.node.get_logger().info(
                    "Aguardando estabilização...",
                    throttle_duration_sec=1.0,
                )
            return None

        yaw_diff = context.yaw_diff_shortest(
            context.state_px4.current_yaw_deg_normalized, context.target_final_yaw_deg
        )

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                f"Yaw alvo alcançado ({context.target_final_yaw_deg:.1f}°). Aguardando estabilização..."
            )

        return None
