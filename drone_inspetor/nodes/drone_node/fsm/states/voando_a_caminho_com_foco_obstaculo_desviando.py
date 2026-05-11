# voando_a_caminho_com_foco_obstaculo_desviando.py
# Estado: VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIANDO

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviandoState


class VoandoACaminhoComFocoObstaculoDesviandoState(BaseObstaculoDesviandoState):
    NEXT_OBSTACULO_ID = DS.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO
    NEXT_GIRANDO_FIM_ID = DS.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_FIM
