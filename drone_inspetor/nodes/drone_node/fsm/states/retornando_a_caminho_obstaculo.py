# retornando_a_caminho_obstaculo.py
# Estado: RETORNANDO_A_CAMINHO_OBSTACULO

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoState


class RetornandoACaminhoObstaculoState(BaseObstaculoState):
    NEXT_GIRANDO_INICIO_ID = DS.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO
