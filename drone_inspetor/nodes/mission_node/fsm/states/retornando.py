# retornando.py
# =================================================================================================
# ESTADO: RETORNANDO
# =================================================================================================
# Envia comando RTL e aguarda o drone pousar e se desarmar.
# Transiciona para PRONTO após pouso e desarmamento bem-sucedidos.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import (
    MissionStateDescription as MS,
    DroneStateDescription as DS,
    DRONE_STATES_RTL,
    DRONE_STATES_POUSANDO,
)


class RetornandoState(State):
    """
    Retorno ao ponto de origem via RTL. Aguarda pouso e desarmamento para ir a PRONTO.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        # Aguarda action de RTL em andamento
        if self.node._action_in_progress:
            self.node.get_logger().info(
                "Aguardando conclusão do RTL...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone em estados de RTL (voando de volta)
        if drone_state in DRONE_STATES_RTL:
            self.node.get_logger().info(
                f"Drone retornando ao home ({drone_state.name})...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone pousou e desarmou — missão encerrada com sucesso
        if drone_state == DS.POUSADO_DESARMADO:
            self.node.get_logger().info(
                "Drone pousado e desarmado. Retorno concluído com sucesso!"
            )
            self.context.reset()
            return MS.PRONTO

        # Drone pousado mas ainda armado — aguarda desarmamento automático
        if drone_state == DS.POUSADO_ARMADO:
            self.node.get_logger().info(
                "Drone pousado e armado. Aguardando desarmamento automático...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone em processo de pouso
        if drone_state in DRONE_STATES_POUSANDO:
            self.node.get_logger().info(
                "Drone pousando...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone está voando estável → inicia RTL
        if drone_state == DS.VOANDO_PRONTO:
            self.node.get_logger().info("Drone voando. Enviando comando RTL.")
            self.node.send_drone_action({"command": "RTL"})
            return None

        self.node.get_logger().warn(
            f"Estado inesperado durante retorno: {drone_state.name}."
        )
        return None
