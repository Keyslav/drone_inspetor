# retornando_a_caminho_obstaculo_desviando.py
# Estado: RETORNANDO_A_CAMINHO_OBSTACULO_DESVIANDO

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviandoState


class RetornandoACaminhoObstaculoDesviandoState(BaseObstaculoDesviandoState):
    NEXT_OBSTACULO_ID = DS.RETORNANDO_A_CAMINHO_OBSTACULO
    NEXT_GIRANDO_FIM_ID = DS.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM
