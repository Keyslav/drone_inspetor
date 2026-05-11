# executando_inspecionando_escaneando.py
# =================================================================================================
# ESTADO: EXECUTANDO_INSPECIONANDO_ESCANEANDO
# =================================================================================================
# Inicia gravação e detecção de anomalias por um período (tempo_de_permanencia).
# Fase 1: dispara gravação e habilita detecção de anomalias.
# Fase 2: aguarda o tempo de escaneamento.
# Fase 3: transiciona para ESCANEAMENTO_FINALIZADO.
# =================================================================================================

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import MissionStateDescription as MS, DroneStateDescription as DS
from drone_inspetor_msgs.srv import RecordDetectionsSRV, EnableAnomalyDetectionSRV

_SCAN_DURATION = 5.0   # segundos de escaneamento (pode ser substituído por context.tempo_de_permanencia)


class ExecutandoEscaneandoState(State):
    """
    Realiza o escaneamento do ponto atual: grava vídeo e detecta anomalias pelo tempo configurado.
    """

    def on_enter(self):
        self.context._scanning_started = False

    def on_step(self):
        # Fase 1: iniciar gravação e detecção de anomalias
        if not self.context._scanning_started:
            self.context._scanning_started = True
            self.context._scanning_start_time = self.context.now()
            self.node.get_logger().info("Iniciando escaneamento: gravação e detecção de anomalias.")

            # Inicia gravação
            if self.node._cv_record_client.service_is_ready():
                rec_req = RecordDetectionsSRV.Request()
                rec_req.start_recording = True
                rec_req.folder_path = self.context.mission_folder_path
                self.node._cv_record_client.call_async(rec_req)
            else:
                self.node.get_logger().warn("Service de gravação CV não disponível. Escaneamento sem gravação.")

            # Habilita detecção de anomalias
            if self.node._cv_anomaly_detection_client.service_is_ready():
                anom_req = EnableAnomalyDetectionSRV.Request()
                anom_req.enable = True
                self.node._cv_anomaly_detection_client.call_async(anom_req)
            else:
                self.node.get_logger().warn("Service de anomalia CV não disponível. Escaneamento sem anomalia.")

            return None

        # Fase 2: aguardar duração do escaneamento
        elapsed = self.context.now() - self.context._scanning_start_time
        scan_duration = self.context.tempo_de_permanencia if self.context.tempo_de_permanencia > 0 else _SCAN_DURATION
        remaining = scan_duration - elapsed

        if remaining > 0:
            self.node.get_logger().info(
                f"Escaneando... {remaining:.1f}s restantes.",
                throttle_duration_sec=1.0,
            )
            return None

        # Fase 3: escaneamento concluído
        self.node.get_logger().info("Escaneamento concluído!")
        self.context._scanning_started = False
        return MS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO
