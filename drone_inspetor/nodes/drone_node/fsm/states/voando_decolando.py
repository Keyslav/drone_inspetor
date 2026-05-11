# voando_decolando.py
# Estado: VOANDO_DECOLANDO
# Drone subindo até a altitude de decolagem.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class VoandoDecolandoState(State):
    """
    Drone em decolagem.

    Monitora altitude atual versus altitude alvo. Quando a diferença
    estiver dentro da tolerância de posição, transiciona para VOANDO_PRONTO.
    """

    def on_step(self):
        state_px4 = self.context.state_px4

        if state_px4.local_position is None or self.context.target_local_position is None:
            return None

        current_alt = -state_px4.local_position.z
        target_alt = -self.context.target_local_position[2]

        if abs(current_alt - target_alt) <= self.context.position_tolerance:
            self.node.get_logger().info(
                f"Decolagem completa! Altitude atual: {current_alt:.2f}m"
            )
            return DS.VOANDO_PRONTO

        return None
