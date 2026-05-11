# voando_a_caminho_obstaculo_girando_inicio.py
# Estado: VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO
# Gira para alinhar yaw com a direção do desvio.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoGirandoInicioState


class VoandoACaminhoObstaculoGirandoInicioState(BaseObstaculoGirandoInicioState):
    NEXT_DESVIANDO_ID = DS.VOANDO_A_CAMINHO_OBSTACULO_DESVIANDO
