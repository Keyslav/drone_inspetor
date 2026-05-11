# inspecao_finalizada.py
# =================================================================================================
# ESTADO: INSPECAO_FINALIZADA
# =================================================================================================
# Todos os pontos foram inspecionados. Transiciona imediatamente para RETORNANDO.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS


class InspecaoFinalizadaState(State):
    """
    Inspeção concluída. Dispara imediatamente o retorno ao ponto de origem.
    """

    def on_step(self):
        self.node.get_logger().info("Inspeção finalizada. Iniciando retorno...")
        return MS.RETORNANDO
