# =================================================================================================
# EmergenciaState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.EMERGENCIA
#
# Significado: failsafe ativo. O DroneNode delegou o controle ao autopilot via RTL
# nativo do PX4 (bateria crítica, perda de offboard catastrófica, etc.). Nenhum
# setpoint offboard é enviado enquanto neste estado.
#
# Transições:
#     Drone foi desarmado (PX4 concluiu o RTL/LAND emergencial) → POUSADO_DESARMADO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class EmergenciaState(BaseState):
    """Failsafe ativo: controle delegado ao autopilot."""

    def on_enter(self) -> None:
        self.node.get_logger().error("EMERGÊNCIA: controle delegado ao autopilot (RTL nativo).")
        # Limpa qualquer estado de trajetória pendente — não vamos voltar a controlar
        # offboard até o drone aterrissar e ser desarmado.
        self.node.deslocamento_fsm_context.reset()
        # Dispara o RTL nativo do PX4 (entra em AUTO_RTL).
        self.node.emergency_rtl()

    def on_step(self):
        px4 = self.node.state_px4

        # PX4 concluiu o failsafe: drone pousou e desarmou.
        if not px4.is_armed and px4.is_landed:
            self.node.get_logger().info("Failsafe concluído. Drone pousou e desarmou.")
            self.context.emergency_active = False
            return DS.POUSADO_DESARMADO

        return None
