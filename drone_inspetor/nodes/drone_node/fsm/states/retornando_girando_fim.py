# retornando_girando_fim.py
# Estado: RETORNANDO_GIRANDO_FIM
# Drone girando para o yaw original de home antes de pousar no RTL.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class RetornandoGirandoFimState(State):
    """
    Drone girando para o yaw HOME antes do pouso final (RTL).

    Fluxo:
    1. Se home_yaw_deg é None → reseta variáveis, comanda pouso e vai para POUSANDO.
    2. Se yaw_aligned_time está definido → verifica tempo de estabilização.
    3. Se |yaw_diff| <= tolerância → define yaw_aligned_time.
    """

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context
        state_px4 = context.state_px4

        # Sem yaw final definido → pousa imediatamente
        if state_px4.home_yaw_deg is None:
            context.reset_trajectory_vars()
            context.command_land_requested = False
            self.node.get_logger().info("RTL: Iniciando pouso (sem yaw final)...")
            self.node.land()
            return DS.POUSANDO

        # Em período de estabilização
        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                self.node.get_logger().info("RTL: Estabilização completa. Iniciando pouso...")
                context.reset_trajectory_vars()
                context.command_land_requested = False
                self.node.land()
                return DS.POUSANDO
            else:
                self.node.get_logger().info(
                    "RTL: Aguardando estabilização...",
                    throttle_duration_sec=1.0,
                )
            return None

        yaw_diff = context.yaw_diff_shortest(
            state_px4.current_yaw_deg_normalized, state_px4.home_yaw_deg
        )

        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                f"RTL: Yaw HOME alcançado ({state_px4.home_yaw_deg:.1f}°). Aguardando estabilização..."
            )

        return None
