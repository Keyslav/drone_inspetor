# voando_a_caminho_com_foco_obstaculo.py
# Estado: VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoState


class VoandoACaminhoComFocoObstaculoState(BaseObstaculoState):
    NEXT_GIRANDO_INICIO_ID = DS.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_INICIO
