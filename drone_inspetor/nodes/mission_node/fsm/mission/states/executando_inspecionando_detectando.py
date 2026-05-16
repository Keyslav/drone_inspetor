# executando_inspecionando_detectando.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO_DETECTANDO
# =================================================================================================
# Chama o service de detecção CV para localizar o objeto alvo no ponto atual.
# Fase 1: inicia detecção assíncrona.
# Fase 2: aguarda resultado (bbox_center ou timeout de 10s).
# Fase 3: se detectado → ESCANEANDO; caso contrário → avança ponto e volta a INSPECIONANDO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor_msgs.srv import CVDetectionSRV, RecordDetectionsSRV

_DETECTION_TIMEOUT = 10.0   # segundos máximos aguardando resultado de detecção
_DETECTION_SRV_TIMEOUT = 2.0  # timeout passado para o service de detecção


class ExecutandoDetectandoState(BaseState):
    """
    Solicita detecção do objeto alvo via service CV.
    Em caso de sucesso avança para escaneamento; em falha, pula o ponto.
    """

    def on_enter(self):
        self.context._detection_started = False
        self.node._detection_bbox_center = None

    def on_step(self):
        # Valida missão e ponto atual
        if not self.context.mission:
            self.node.get_logger().warn("Missão não carregada em DETECTANDO. Retornando a INSPECIONANDO.")
            return MS.EXECUTANDO_INSPECIONANDO

        ponto_atual = self.context.get_ponto_atual()
        if ponto_atual is None:
            self.node.get_logger().warn("Ponto atual inválido em DETECTANDO. Retornando a INSPECIONANDO.")
            return MS.EXECUTANDO_INSPECIONANDO

        objeto_alvo = ponto_atual.get("objeto_alvo", "")
        tipos_anomalia = ponto_atual.get("tipos_anomalia", [])

        # Fase 1: Iniciar detecção
        if not self.context._detection_started:
            self.context._detection_started = True
            self.context._detection_start_time = self.context.now()
            self.node.get_logger().info(
                f"Iniciando detecção de '{objeto_alvo}' (anomalias: {tipos_anomalia})."
            )

            if not self.node._cv_detection_client.service_is_ready():
                self.node.get_logger().warn("Service de detecção CV não disponível. Pulando ponto.")
                self._skip_point()
                return MS.EXECUTANDO_INSPECIONANDO

            request = CVDetectionSRV.Request()
            request.objeto_alvo = objeto_alvo
            request.tipos_anomalia = tipos_anomalia
            request.timeout = _DETECTION_SRV_TIMEOUT

            future = self.node._cv_detection_client.call_async(request)
            future.add_done_callback(self.node._detection_response_callback)
            return None

        # Fase 2: Aguardando resultado
        elapsed = self.context.now() - self.context._detection_start_time
        if elapsed < _DETECTION_TIMEOUT and self.node._detection_bbox_center is None:
            self.node.get_logger().info(
                f"Aguardando detecção... ({elapsed:.1f}s / {_DETECTION_TIMEOUT:.1f}s)",
                throttle_duration_sec=2.0,
            )
            return None

        # Fase 3: Avaliar resultado
        if self.node._detection_bbox_center is not None:
            self.node.get_logger().info(
                f"Objeto '{objeto_alvo}' detectado em {self.node._detection_bbox_center}. "
                "Iniciando escaneamento."
            )
            self.node._approach_arrival_time = 0.0
            self.context._detection_started = False
            return MS.EXECUTANDO_INSPECIONANDO_ESCANEANDO
        else:
            self.node.get_logger().warn(
                f"Objeto '{objeto_alvo}' não detectado após {elapsed:.1f}s. Pulando ponto."
            )
            self._stop_recording()
            self._skip_point()
            return MS.EXECUTANDO_INSPECIONANDO

    def _skip_point(self):
        """Avança para o próximo ponto de inspeção."""
        self.context.ponto_de_inspecao_indice_atual += 1
        self.context.ponto_de_inspecao_tempo_de_chegada = 0.0
        self.context._detection_started = False
        self.node._detection_bbox_center = None

    def _stop_recording(self):
        """Para gravação via service cv_record_client, sem bloquear."""
        if not self.node._cv_record_client.service_is_ready():
            return
        request = RecordDetectionsSRV.Request()
        request.start_recording = False
        self.node._cv_record_client.call_async(request)
