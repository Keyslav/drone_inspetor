# executando_inspecionando_falha.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO_FALHA
# =================================================================================================
# Estado reservado para tratamento de falhas críticas durante a inspeção.
# Atualmente é um placeholder para implementação futura.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class ExecutandoFalhaState(BaseState):
    """
    Estado de falha durante inspeção. Reservado para tratamento futuro de falhas críticas.
    """

    def on_step(self):
        # Reservado para implementação futura de lógica de recuperação de falhas.
        pass
        return None
