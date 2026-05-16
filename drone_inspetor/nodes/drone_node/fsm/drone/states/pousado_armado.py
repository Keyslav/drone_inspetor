# =================================================================================================
# PousadoArmadoState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.POUSADO_ARMADO
#
# Significado: drone no solo, motores armados (rotores girando). Aguarda comando TAKEOFF.
#
# Transições:
#     pending_command == "TAKEOFF" → inicia decolagem e vai para DECOLANDO.
#     Drone foi desarmado (auto-desarme do PX4) → POUSADO_DESARMADO.
#     Drone saiu do solo sem comando explícito  → EM_VOO (fallback raro).
#     PX4 saiu do modo OFFBOARD → OFFBOARD_DESATIVADO.
# =================================================================================================

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class PousadoArmadoState(BaseState):
    """Drone no solo, armado. Aguarda comando TAKEOFF."""

    def on_enter(self) -> None:
        # Garante setpoints de hover na posição atual enquanto aguarda TAKEOFF
        # (evita drift quando o autopilot fica em offboard com armed e sem setpoint).
        self.node.deslocamento_fsm_context.store_static_position()
        self.node.get_logger().info("Drone POUSADO_ARMADO. Aguardando comando TAKEOFF.")

    def on_step(self):
        ctx = self.context
        px4 = self.node.state_px4

        # Saiu do OFFBOARD.
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO

        # Foi desarmado (manual ou auto-desarme).
        if not px4.is_armed:
            return DS.POUSADO_DESARMADO

        # Comando TAKEOFF pendente: dispara a manobra de decolagem.
        # O helper takeoff() do node trata o setpoint inicial; a fase subsequente
        # (subida vertical até a altitude alvo) é responsabilidade do estado DECOLANDO.
        if ctx.pending_command == "TAKEOFF":
            self.node.takeoff()
            ctx.pending_command = None
            return DS.DECOLANDO

        # Drone saiu do solo sem comando (ex.: empurrão físico ou erro de leitura is_landed).
        # Caso raro mas defensivo: já vai para EM_VOO e deixa a DeslocamentoFSM cuidar do hover.
        if not px4.is_landed:
            self.node.get_logger().warn(
                "Drone POUSADO_ARMADO mas detectado no ar. Transicionando para EM_VOO."
            )
            return DS.EM_VOO

        return None
