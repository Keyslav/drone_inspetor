# pousado_armado.py
# Estado: POUSADO_ARMADO
# Drone no chão, armado e aguardando comando de decolagem.

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class PousadoArmadoState(State):
    """
    Drone pousado e armado.

    - Se pending_command == "TAKEOFF": inicializa variáveis de trajetória e
      transiciona para VOANDO_DECOLANDO.
    - Se drone não está mais pousado (levantou sem comando): reseta variáveis
      e transiciona para VOANDO_PRONTO.
    """

    def on_step(self):
        # Comando de decolagem pendente
        if self.context.pending_command == "TAKEOFF":
            self.context.on_trajectory = True
            self.context.trajectory_start_time = self.context.now()
            self.context.pending_command = None
            return DS.VOANDO_DECOLANDO

        # Drone levantou sem comando explícito (ex.: empurrado)
        if not self.context.state_px4.is_landed:
            self.node.get_logger().info(
                "Drone em POUSADO_ARMADO mas não está pousado. "
                "Transicionando para VOANDO_PRONTO."
            )
            self.context.reset_trajectory_vars()
            return DS.VOANDO_PRONTO

        return None
