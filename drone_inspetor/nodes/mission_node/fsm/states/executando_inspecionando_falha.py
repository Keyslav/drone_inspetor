# executando_inspecionando_falha.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO_FALHA
# =================================================================================================
# Estado reservado para tratamento de falhas críticas durante a inspeção.
# Atualmente é um placeholder para implementação futura.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class ExecutandoFalhaState(State):
    """
    Estado de falha durante inspeção. Reservado para tratamento futuro de falhas críticas.
    """

    def on_step(self):
        # Reservado para implementação futura de lógica de recuperação de falhas.
        pass
        return None
