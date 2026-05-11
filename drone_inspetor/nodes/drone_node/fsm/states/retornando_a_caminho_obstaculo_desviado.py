# retornando_a_caminho_obstaculo_desviado.py
# Estado: RETORNANDO_A_CAMINHO_OBSTACULO_DESVIADO

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviadoState


class RetornandoACaminhoObstaculoDesviadoState(BaseObstaculoDesviadoState):
    RETOMA_NORMAL_ID = DS.RETORNANDO_GIRANDO_INICIO
    NEXT_OBSTACULO_ID = DS.RETORNANDO_A_CAMINHO_OBSTACULO
