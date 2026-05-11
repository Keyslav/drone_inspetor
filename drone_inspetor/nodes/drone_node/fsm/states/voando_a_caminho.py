# voando_a_caminho.py
# Estado: VOANDO_A_CAMINHO
# Drone voando em direção ao target.

import math

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoACaminhoState(State):
    """
    Drone em trânsito para o target.

    Calcula distância 3D ao ponto alvo. Quando dentro da tolerância,
    completa o waypoint atual na pilha e avança para VOANDO_GIRANDO_FIM.
    Detecção de obstáculo na direção do movimento → VOANDO_A_CAMINHO_OBSTACULO.
    """

    def on_step(self):
        context = self.context
        state_px4 = context.state_px4

        if state_px4.local_position is None or context.target_local_position is None:
            return None

        # Detecção de obstáculo na direção do movimento → entra na sub-FSM de desvio.
        if context.target_direction_yaw_rad is not None:
            if context.obstacles.has_obstacle_in_sector(context.target_direction_yaw_rad):
                self.node.get_logger().warn(
                    "VOANDO_A_CAMINHO: Obstáculo detectado na direção do movimento."
                )
                return DS.VOANDO_A_CAMINHO_OBSTACULO

        cur = state_px4.local_position
        tx, ty, tz = context.target_local_position

        dx = tx - cur.x
        dy = ty - cur.y
        dz = tz - cur.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= context.position_tolerance:
            context.store_static_position()
            context.waypoint_stack.complete_current()

            yaw_info = (
                f" (yaw alvo: {context.target_final_yaw_deg:.1f}°)"
                if context.target_final_yaw_deg is not None
                else " (sem yaw final)"
            )
            self.node.get_logger().info(
                f"Posição alvo alcançada. Avançando para GIRANDO_FIM{yaw_info}."
            )
            return DS.VOANDO_GIRANDO_FIM

        return None
