# voando_a_caminho_obstaculo_desviando.py
# Estado: VOANDO_A_CAMINHO_OBSTACULO_DESVIANDO
# Voa até a coordenada de desvio. Se novo obstáculo, recalcula.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviandoState


class VoandoACaminhoObstaculoDesviandoState(BaseObstaculoDesviandoState):
    NEXT_OBSTACULO_ID = DS.VOANDO_A_CAMINHO_OBSTACULO
    NEXT_GIRANDO_FIM_ID = DS.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM
