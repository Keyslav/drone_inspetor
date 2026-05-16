# =================================================================================================
# EmVooState — estado da DroneFSM
# =================================================================================================
# DroneFSMDescription.EM_VOO
#
# Significado: drone no ar, executando o que for que estiver na TargetStack. Toda a
# lógica de manobra (hover, girar, deslocar) é delegada à DeslocamentoFSM paralela,
# que SÓ é tickada enquanto este estado está ativo.
#
# Este estado lifecycle observa apenas eventos físicos de alto nível:
#     - Drone pousou (LAND ou RTL concluiu) → POUSADO_ARMADO.
#     - Drone desarmou (auto-desarme após pouso) → POUSADO_DESARMADO.
#     - PX4 saiu do OFFBOARD → OFFBOARD_DESATIVADO.
#
# Comandos GOTO/LAND/RTL recebidos durante este estado NÃO transicionam o lifecycle —
# eles apenas alteram o conteúdo da TargetStack (ou seja, o que a DeslocamentoFSM
# deve executar). O DroneNode processa esses comandos no action_server.
# =================================================================================================

from px4_msgs.msg import VehicleStatus

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class EmVooState(BaseState):
    """Drone no ar — DeslocamentoFSM cuida das manobras."""

    def on_enter(self) -> None:
        self.node.get_logger().info("Drone EM_VOO. DeslocamentoFSM no comando das manobras.")
        # Captura a posição atual como referência de hover inicial.
        # A DeslocamentoFSM começa em PLANANDO (hover) e usa esta referência até que
        # um novo target seja empilhado.
        self.node.deslocamento_fsm_context.store_static_position()

    def on_step(self):
        px4 = self.node.state_px4

        # Saiu do modo OFFBOARD em pleno voo: situação grave, aborta gracioso.
        if px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return DS.OFFBOARD_DESATIVADO

        # Pousou (concluiu LAND/RTL ou aterrissagem forçada do PX4).
        if px4.is_landed:
            if px4.is_armed:
                return DS.POUSADO_ARMADO
            return DS.POUSADO_DESARMADO

        # Desarmou ainda no ar? Não deveria acontecer; defensivo.
        if not px4.is_armed:
            self.node.get_logger().error("Drone desarmou em pleno voo!")
            return DS.POUSADO_DESARMADO

        # Em voo normal: nada a transicionar daqui. DeslocamentoFSM cuida do resto.
        return None
