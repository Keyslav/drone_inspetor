# executando_armando.py
# =================================================================================================
# ESTADO: EXECUTANDO_ARMANDO
# =================================================================================================
# Envia comando ARM para o drone e aguarda confirmação.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class ExecutandoArmandoState(State):
    """
    Arma os motores do drone. Transiciona para DECOLANDO quando armado com sucesso.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        # Aguarda action em andamento
        if self.node._action_in_progress:
            if self.node.check_action_timeout():
                self.node.get_logger().error("Timeout ao armar. Abortando missão.")
                self.context.reset()
                return MS.DESATIVADO
            self.node.get_logger().info(
                "Aguardando confirmação de armamento...",
                throttle_duration_sec=3.0,
            )
            return None

        if drone_state == DS.POUSADO_ARMADO:
            self.node.get_logger().info("Drone armado! Iniciando decolagem.")
            return MS.EXECUTANDO_DECOLANDO

        if drone_state == DS.POUSADO_DESARMADO:
            self.node.get_logger().info("Enviando comando ARM para o drone.")
            self.node.send_drone_action({"command": "ARM"})
            return None

        self.node.get_logger().warn(
            f"Estado inesperado durante armamento: {drone_state.name}. Abortando."
        )
        self.context.reset()
        return MS.DESATIVADO
