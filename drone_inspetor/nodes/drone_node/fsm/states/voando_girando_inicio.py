# voando_girando_inicio.py
# Estado: VOANDO_GIRANDO_INICIO
# Drone girando para apontar na direção do destino antes de iniciar o voo.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoGirandoInicioState(State):
    """
    Drone girando para a direção do target antes de partir.

    Fluxo:
    1. Se target_direction_yaw_deg é None → avança imediatamente para A_CAMINHO.
    2. Se yaw_aligned_time está definido → verifica tempo de estabilização.
    3. Se |yaw_diff| <= tolerância → define yaw_aligned_time.
    """

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None

    def on_step(self):
        context = self.context

        target_dir_yaw = context.target_direction_yaw_deg
        if target_dir_yaw is None:
            self.node.get_logger().info(
                "GIRANDO_INICIO: Sem yaw de direção definido, avançando para VOANDO_A_CAMINHO."
            )
            return DS.VOANDO_A_CAMINHO

        yaw_diff = context.yaw_diff_shortest(
            context.state_px4.current_yaw_deg_normalized, target_dir_yaw
        )

        # Em período de estabilização
        if context.yaw_aligned_time is not None:
            elapsed = context.now() - context.yaw_aligned_time
            if elapsed >= context.yaw_stabilization_delay:
                context.yaw_aligned_time = None
                self.node.get_logger().info("Estabilização completa. Iniciando movimento.")
                return DS.VOANDO_A_CAMINHO
            else:
                self.node.get_logger().info(
                    "Aguardando estabilização...",
                    throttle_duration_sec=1.0,
                )
            return None

        # Verifica alinhamento de yaw
        if abs(yaw_diff) <= context.yaw_tolerance_deg:
            context.yaw_aligned_time = context.now()
            self.node.get_logger().info(
                f"Yaw de direção alcançado ({target_dir_yaw:.1f}°). Aguardando estabilização..."
            )

        return None
