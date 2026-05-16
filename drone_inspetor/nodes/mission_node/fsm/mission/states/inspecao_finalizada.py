# inspecao_finalizada.py
# =================================================================================================
# ESTADO: INSPECAO_FINALIZADA
# =================================================================================================
# Todos os pontos foram inspecionados. Transiciona imediatamente para RETORNANDO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class InspecaoFinalizadaState(BaseState):
    """
    Inspeção concluída. Dispara imediatamente o retorno ao ponto de origem.
    """

    def on_step(self):
        self.node.get_logger().info("Inspeção finalizada. Iniciando retorno...")
        return MS.RETORNANDO
