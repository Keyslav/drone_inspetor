# voando_a_caminho_obstaculo_girando_fim.py
# Estado: VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM
# Gira para alinhar yaw com o destino original.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoGirandoFimState


class VoandoACaminhoObstaculoGirandoFimState(BaseObstaculoGirandoFimState):
    NEXT_DESVIADO_ID = DS.VOANDO_A_CAMINHO_OBSTACULO_DESVIADO
