# voando_a_caminho_com_foco_obstaculo_desviado.py
# Estado: VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIADO
# Retorna para VOANDO_GIRANDO_COM_FOCO ao confirmar caminho livre, retomando
# o yaw apontando ao foco original.

from drone_inspetor.common.enums import DroneStateDescription as DS
from drone_inspetor.nodes.drone_node.fsm.states._obstaculo_base import BaseObstaculoDesviadoState


class VoandoACaminhoComFocoObstaculoDesviadoState(BaseObstaculoDesviadoState):
    RETOMA_NORMAL_ID = DS.VOANDO_GIRANDO_COM_FOCO
    NEXT_OBSTACULO_ID = DS.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO
