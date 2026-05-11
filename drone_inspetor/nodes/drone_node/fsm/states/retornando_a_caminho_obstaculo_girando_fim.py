# retornando_a_caminho_obstaculo_girando_fim.py
# Estado: RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoGirandoFimState


class RetornandoACaminhoObstaculoGirandoFimState(BaseObstaculoGirandoFimState):
    NEXT_DESVIADO_ID = DS.RETORNANDO_A_CAMINHO_OBSTACULO_DESVIADO
