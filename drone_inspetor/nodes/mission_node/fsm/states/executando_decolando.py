# executando_decolando.py
# =================================================================================================
# ESTADO: EXECUTANDO_DECOLANDO
# =================================================================================================
# Envia comando TAKEOFF e aguarda o drone atingir altitude de cruzeiro.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class ExecutandoDecolandoState(State):
    """
    Decola o drone até a altitude configurada. Transiciona para INSPECIONANDO ao ficar voando estável.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        # Aguarda action em andamento
        if self.node._action_in_progress:
            if self.node.check_action_timeout():
                self.node.get_logger().error("Timeout na decolagem. Abortando missão.")
                self.context.reset()
                return MS.DESATIVADO
            self.node.get_logger().info(
                "Aguardando conclusão da decolagem...",
                throttle_duration_sec=3.0,
            )
            return None

        if drone_state == DS.VOANDO_PRONTO:
            self.node.get_logger().info("Decolagem concluída! Iniciando inspeção.")
            return MS.EXECUTANDO_INSPECIONANDO

        if drone_state == DS.POUSADO_ARMADO:
            self.node.get_logger().info(
                f"Enviando comando TAKEOFF (altitude: {self.context.takeoff_altitude}m)."
            )
            self.node.send_drone_action({
                "command": "TAKEOFF",
                "altitude": self.context.takeoff_altitude,
            })
            return None

        if drone_state == DS.VOANDO_DECOLANDO:
            self.node.get_logger().info(
                "Drone decolando...",
                throttle_duration_sec=3.0,
            )
            return None

        self.node.get_logger().warn(
            f"Estado inesperado durante decolagem: {drone_state.name}."
        )
        return None
