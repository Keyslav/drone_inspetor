# voando_a_caminho_obstaculo_desviado.py
# Estado: VOANDO_A_CAMINHO_OBSTACULO_DESVIADO
# Verifica se obstáculo sumiu e retoma rota original.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviadoState


class VoandoACaminhoObstaculoDesviadoState(BaseObstaculoDesviadoState):
    RETOMA_NORMAL_ID = DS.VOANDO_GIRANDO_INICIO
    NEXT_OBSTACULO_ID = DS.VOANDO_A_CAMINHO_OBSTACULO
