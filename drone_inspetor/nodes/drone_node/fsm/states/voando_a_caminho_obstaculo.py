# voando_a_caminho_obstaculo.py
# Estado: VOANDO_A_CAMINHO_OBSTACULO
# Drone parou por obstáculo durante VOANDO_A_CAMINHO. Calcula desvio.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoState


class VoandoACaminhoObstaculoState(BaseObstaculoState):
    NEXT_GIRANDO_INICIO_ID = DS.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO
