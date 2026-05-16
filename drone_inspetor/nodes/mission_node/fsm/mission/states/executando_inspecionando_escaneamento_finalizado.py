# executando_inspecionando_escaneamento_finalizado.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO
# =================================================================================================
# Para a gravação e desabilita detecção de anomalias. Limpa bbox center.
# Avança o índice do ponto atual e retorna para INSPECIONANDO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor_msgs.srv import RecordDetectionsSRV, EnableAnomalyDetectionSRV


class ExecutandoEscaneamentoFinalizadoState(BaseState):
    """
    Encerra gravação e detecção de anomalias após o escaneamento. Avança para o próximo ponto.
    """

    def on_step(self):
        # Para gravação
        if self.node._cv_record_client.service_is_ready():
            rec_req = RecordDetectionsSRV.Request()
            rec_req.start_recording = False
            self.node._cv_record_client.call_async(rec_req)
        else:
            self.node.get_logger().warn("Service de gravação CV não disponível ao finalizar escaneamento.")

        # Desabilita detecção de anomalias
        if self.node._cv_anomaly_detection_client.service_is_ready():
            anom_req = EnableAnomalyDetectionSRV.Request()
            anom_req.enable = False
            self.node._cv_anomaly_detection_client.call_async(anom_req)
        else:
            self.node.get_logger().warn("Service de anomalia CV não disponível ao finalizar escaneamento.")

        # Limpa bbox center
        self.node._detection_bbox_center = None

        # Avança para o próximo ponto
        ponto_idx = self.context.ponto_de_inspecao_indice_atual
        self.context.ponto_de_inspecao_indice_atual += 1
        self.context.ponto_de_inspecao_tempo_de_chegada = 0.0

        self.node.get_logger().info(
            f"Ponto {ponto_idx} escaneado com sucesso. "
            f"Avançando para ponto {self.context.ponto_de_inspecao_indice_atual}."
        )

        return MS.EXECUTANDO_INSPECIONANDO
