# pronto.py
# =================================================================================================
# ESTADO: PRONTO
# =================================================================================================
# Drone pousado e desarmado, aguardando comando de missão do dashboard.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class ProntoState(State):
    """
    Sistema pronto. Aguarda flag on_mission para iniciar ou detecta falha de drone.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        if drone_state != DS.POUSADO_DESARMADO:
            self.node.get_logger().warn(
                f"Drone saiu do estado POUSADO_DESARMADO em PRONTO (estado: {drone_state.name}). Resetando..."
            )
            self.context.reset()
            return MS.DESATIVADO

        if self.context.on_mission:
            mission_name = self.context.mission.get("nome", "desconhecida") if self.context.mission else "desconhecida"
            self.node.get_logger().info(f"Iniciando missão: '{mission_name}'")
            return MS.EXECUTANDO_ARMANDO

        self.node.get_logger().info(
            "Aguardando Missão...",
            throttle_duration_sec=5.0,
        )
        return None
