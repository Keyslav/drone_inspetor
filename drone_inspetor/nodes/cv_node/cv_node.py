"""Adaptador ROS para inferência, consultas de detecção e mídia de inspeção.

Imagem, estado de missão e controles pertencem ao mesmo grupo de callbacks:
nenhum frame combina configuração de missões ou modelos diferentes. A consulta
bloqueante de detecção tem grupo próprio e aguarda snapshots com prazo monotônico.
"""

from datetime import datetime
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from drone_inspetor.common.param_utils import load_param
from drone_inspetor.media.photos import filename_component, save_photo
from drone_inspetor.media.recorder import VideoRecorder
from drone_inspetor.nodes.cv_node.detection_buffer import DetectionBuffer
from drone_inspetor.nodes.cv_node.inference import DetectionPipeline, InferenceOptions
from drone_inspetor.nodes.cv_node.model_registry import (
    ModelManager, ModelRegistry, resolve_models_directory,
)
from drone_inspetor.ros_interfaces import (
    Topics, create_publisher_from, create_service_from, create_subscription_from,
)
from drone_inspetor_msgs.msg import CVDetectionMSG, CVDetectionItemMSG


FOCUSED_STATES = frozenset((
    'EXECUTANDO_INSPECIONANDO_DETECTANDO',
    'EXECUTANDO_INSPECIONANDO_ESCANEANDO',
    'EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO',
))


class CVNode(Node):
    """Converte contratos ROS em operações dos componentes de percepção."""

    def __init__(self, **node_options):
        super().__init__('cv_node', **node_options)
        self._is_shutting_down = False
        self.camera_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.bridge = CvBridge()
        self._current_mission_state = ''
        self._mission_folder = ''
        self._on_mission = False
        self._last_annotated_image = None
        self._last_frame_time = float('-inf')
        self._last_source_stamp = None
        self._photo_counter = 0
        self._ponto_indice_atual = 0
        self._objeto_alvo = ''
        self._photos_folder = None
        self._videos_folder = None
        self._anomaly_detection_enabled = False
        self._last_anomaly_photo_time = float('-inf')
        self._anomaly_photo_interval = load_param(self, 'anomaly_photo_interval_seconds', 4.0)
        self._object_min_confidence = load_param(self, 'object_detection_min_confidence', 0.5)
        self._anomaly_min_confidence = load_param(self, 'anomaly_detection_min_confidence', 0.5)
        self._photo_format = load_param(self, 'photo_format', 'jpg')
        self._photo_quality = load_param(self, 'photo_quality', 95)
        self._max_frame_age = load_param(self, 'detection_max_frame_age_seconds', 1.0)
        self._detections = DetectionBuffer(self._max_frame_age)
        self._recorder = VideoRecorder(
            load_param(self, 'video_codec', 'mp4v'), load_param(self, 'video_fps', 15),
        )
        if not math.isfinite(self._anomaly_photo_interval) or self._anomaly_photo_interval < 0:
            raise ValueError('Intervalo de fotos deve ser finito e não negativo')
        if any(not 0 <= value <= 1 for value in (
            self._object_min_confidence, self._anomaly_min_confidence,
        )):
            raise ValueError('Confiança mínima deve estar entre 0 e 1')

        self._registry = None
        self._models = None
        self._pipeline = None
        self._initialize_models()
        self._device = self._select_device(load_param(self, 'inference_device', 'auto'))

        self.compressed_image_subscription = create_subscription_from(
            self, Topics.Externo.CAMERA_COMPRESSED, self.image_callback,
            callback_group=self.camera_cb_group,
        )
        self.processed_image_publisher = create_publisher_from(self, Topics.Interno.CV_COMPRESSED)
        self.detection_publisher = create_publisher_from(self, Topics.Interno.CV_OBJECT_DETECTIONS)
        self.mission_state_sub = create_subscription_from(
            self, Topics.Interno.MISSION_STATE, self.mission_state_callback,
            callback_group=self.camera_cb_group,
        )
        self.cv_control_sub = create_subscription_from(
            self, Topics.Dashboard.CV_CONTROL, self._cv_control_callback,
            callback_group=self.camera_cb_group,
        )
        self.detection_service = create_service_from(
            self, Topics.Service.CV_DETECTION, self.detection_service_callback,
            callback_group=self.service_cb_group,
        )
        # Controles curtos continuam disponíveis enquanto a consulta espera imagens.
        self.record_service = create_service_from(
            self, Topics.Service.CV_RECORD_DETECTIONS, self.record_service_callback,
            callback_group=self.camera_cb_group,
        )
        self.anomaly_detection_service = create_service_from(
            self, Topics.Service.CV_ENABLE_ANOMALY, self.enable_anomaly_detection_callback,
            callback_group=self.camera_cb_group,
        )
        self.cv_models_service = create_service_from(
            self, Topics.Service.CV_LIST_MODELS, self.cv_models_service_callback,
            callback_group=self.camera_cb_group,
        )
        self.get_logger().info(f'CV iniciado; inferência em {self._device}')

    @staticmethod
    def _select_device(configured):
        """Nunca anuncia CPU enquanto envia device=0 para o preditor."""
        if str(configured).lower() == 'auto':
            import torch
            return 0 if torch.cuda.is_available() else 'cpu'
        return configured

    def _initialize_models(self):
        """Carrega cada categoria; indisponibilidade de anomalias não impede objetos."""
        try:
            from ultralytics import YOLO
            directory = resolve_models_directory(
                load_param(self, 'models_directory', ''),
                get_package_share_directory('drone_inspetor'))
            self._registry = ModelRegistry(directory)
            self._models = ModelManager(self._registry, YOLO)
            self._pipeline = DetectionPipeline(self._models)
            for kind in ('equipment', 'anomaly'):
                filename = self._registry.first(kind)
                if not filename:
                    continue
                try:
                    args = (filename, '') if kind == 'equipment' else ('', filename)
                    self._models.replace(*args)
                except Exception as exc:
                    self.get_logger().error(f'Falha ao carregar {filename}: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Percepção indisponível: {exc}')

    def _cv_control_callback(self, msg):
        """Troca o par inteiro; uma falha preserva ambos os modelos anteriores."""
        if self._models is None:
            self.get_logger().error('Registro de modelos indisponível')
            return
        try:
            self._models.replace(msg.object_detection_model, msg.anomaly_detection_model)
            self._detections.invalidate()
            self.get_logger().info(f'Modelos ativos: {self._models.filenames}')
        except Exception as exc:
            self.get_logger().error(f'Troca de modelos recusada: {exc}')

    def image_callback(self, msg):
        """Processa um frame com a configuração do início do callback."""
        if self._is_shutting_down:
            return
        received_at = self._frame_acquisition_time(msg)
        if received_at is None:
            return
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            annotated, detections = self.detect_objects(frame)
            if self._is_shutting_down:
                return
            self._last_annotated_image = annotated
            self._last_frame_time = received_at
            # A inferência não renova a idade do frame. Um resultado que levou
            # tempo demais pode aparecer na GUI, mas não confirma uma inspeção.
            self._detections.publish(detections, received_at)
            processed = self.bridge.cv2_to_compressed_imgmsg(annotated, dst_format='jpeg')
            processed.header = msg.header
            self.processed_image_publisher.publish(processed)
            self._publish_detections(detections)
            self._recorder.write(annotated)
        except Exception as exc:
            self.get_logger().error(f'Falha no processamento de imagem: {exc}')

    def _frame_acquisition_time(self, msg):
        """Não renova um frame repetido durante pausa do relógio simulado."""
        received = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp == 0:
            return received  # Driver sem timestamp: somente idade desde a recepção.
        now_ros = self.get_clock().now().nanoseconds / 1e9
        if self._last_source_stamp is not None and now_ros < self._last_source_stamp:
            self._last_source_stamp = None
            self._detections.invalidate()
        age = now_ros - stamp
        if (not 0 <= age <= self._max_frame_age or
                (self._last_source_stamp is not None and stamp <= self._last_source_stamp)):
            return None
        self._last_source_stamp = stamp
        return received - age

    def detect_objects(self, image):
        """Delega inferência e salva capturas conforme o intervalo da missão."""
        if self._pipeline is None:
            return image, []
        options = InferenceOptions(
            object_confidence=self._object_min_confidence,
            anomaly_confidence=self._anomaly_min_confidence,
            target=self._objeto_alvo,
            filter_target=self._current_mission_state in FOCUSED_STATES,
            enable_anomalies=self._anomaly_detection_enabled,
            device=self._device,
        )
        annotated, detections, captures = self._pipeline.process(image, options)
        for capture in captures:
            if not self._is_shutting_down:
                self._capture_anomaly_photos(capture)
        return annotated, detections

    def _publish_detections(self, detections):
        message = CVDetectionMSG()
        message.timestamp = datetime.now().isoformat()
        message.count = len(detections)
        for detection in detections:
            item = CVDetectionItemMSG()
            item.object_type = detection['object_type']
            item.class_name = detection['class']
            item.confidence = detection['confidence']
            item.bbox = detection['bbox']
            item.bbox_center = detection['bbox_center']
            message.detections.append(item)
        self.detection_publisher.publish(message)

    def mission_state_callback(self, msg):
        """Associa mídia à missão e encerra gravações antes de abandonar a pasta."""
        if self._is_shutting_down:
            return
        folder_changed = msg.mission_folder_path != self._mission_folder
        if msg.on_mission and (not self._on_mission or folder_changed):
            # Uma nova sessão pode usar o mesmo nome de missão; a pasta é que
            # separa os artefatos e impede continuar um vídeo da sessão anterior.
            self._recorder.close()
            self._detections.invalidate()
            self._photo_counter = 0
            self._last_anomaly_photo_time = float('-inf')
            self._mission_folder = msg.mission_folder_path
            folder = Path(self._mission_folder) if self._mission_folder else None
            self._photos_folder = folder / 'fotos_cv' if folder else None
            self._videos_folder = folder / 'videos_cv' if folder else None
        if msg.on_mission:
            if (msg.objeto_alvo != self._objeto_alvo or
                    msg.ponto_de_inspecao_indice_atual != self._ponto_indice_atual):
                self._detections.invalidate()
            self._ponto_indice_atual = msg.ponto_de_inspecao_indice_atual
            self._objeto_alvo = msg.objeto_alvo
        elif self._on_mission:
            self._recorder.close()
            self._detections.invalidate()
            self._photos_folder = None
            self._videos_folder = None
            self._mission_folder = ''
            self._anomaly_detection_enabled = False
            self._objeto_alvo = ''
        if (msg.on_mission and msg.state_name == 'EXECUTANDO_INSPECIONANDO_DETECTANDO'
                and self._current_mission_state == 'EXECUTANDO_INSPECIONANDO'):
            self._capture_photo(msg.state_name)
        self._on_mission = msg.on_mission
        self._current_mission_state = msg.state_name

    def _capture_photo(self, state_name):
        if (self._last_annotated_image is None or self._photos_folder is None or
                time.monotonic() - self._last_frame_time > self._max_frame_age):
            return
        self._photo_counter += 1
        state = state_name.lower().replace('executando_inspecionando_', '')
        timestamp = datetime.now().strftime('%H%M%S_%f')
        object_name = filename_component(self._objeto_alvo)
        filename = (f'{self._photo_counter:03d}_P{self._ponto_indice_atual + 1:02d}_'
                    f'{state}_{object_name}_{timestamp}.{self._photo_format}')
        try:
            save_photo(self._photos_folder / filename, self._last_annotated_image,
                       self._photo_quality)
        except (OSError, ValueError, cv2.error) as exc:
            self.get_logger().error(f'Falha ao salvar foto de inspeção: {exc}')

    def _capture_anomaly_photos(self, capture):
        now = time.monotonic()
        if (self._photos_folder is None or
                now - self._last_anomaly_photo_time < self._anomaly_photo_interval):
            return
        self._last_anomaly_photo_time = now
        self._photo_counter += 1
        timestamp = datetime.now().strftime('%H%M%S_%f')
        prefix = (f'P{self._ponto_indice_atual + 1:02d}_'
                  f'{filename_component(capture.object_name.lower())}_{self._photo_counter}')
        size = (capture.original.shape[1], capture.original.shape[0])
        images = (
            ('original', capture.original), ('objeto', capture.object_image),
            ('anomalias', capture.annotated),
            ('crop', cv2.resize(capture.crop, size, interpolation=cv2.INTER_LANCZOS4)),
            ('crop_anomalias', cv2.resize(capture.annotated_crop, size,
                                        interpolation=cv2.INTER_LANCZOS4)),
        )
        try:
            for index, (label, image) in enumerate(images, start=1):
                name = f'{prefix}_{index}_{label}_{timestamp}.{self._photo_format}'
                save_photo(self._photos_folder / name, image, self._photo_quality)
        except (OSError, ValueError, cv2.error) as exc:
            self.get_logger().error(f'Falha ao salvar fotos de anomalias: {exc}')

    def detection_service_callback(self, request, response):
        """Consulta só frames novos, mesmo quando o relógio simulado está pausado."""
        response.success = False
        response.confidence = 0.0
        response.bbox = []
        response.bbox_center = []
        timeout = request.timeout_seconds if request.timeout_seconds != 0 else 2.0
        try:
            detection = self._detections.wait_for(request.object_name, timeout)
        except ValueError as exc:
            response.message = str(exc)
            return response
        if detection is None:
            response.message = ('Percepção encerrada' if self._is_shutting_down else
                                f'Objeto {request.object_name!r} sem observação nova em {timeout}s')
        else:
            response.success = True
            response.confidence = detection['confidence']
            response.bbox = detection['bbox']
            response.bbox_center = detection['bbox_center']
            response.message = f'Objeto {request.object_name!r} detectado'
        return response

    def record_service_callback(self, request, response):
        """O gravador concentra abertura, redimensionamento e liberação do recurso."""
        response.success = False
        response.video_path = ''
        if self._is_shutting_down:
            response.message = 'Percepção encerrada'
            return response
        try:
            if request.start_recording:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                path = (self._videos_folder or Path('/tmp')) / (
                    f'detection_{timestamp}.{self._recorder.extension}')
                response.video_path = self._recorder.start(path, frame_size=(1280, 720))
                response.message = 'Gravação iniciada'
            else:
                if not self._recorder.active:
                    raise RuntimeError('Nenhuma gravação em andamento')
                response.video_path = self._recorder.close()
                response.message = 'Gravação finalizada'
            response.success = True
        except Exception as exc:
            response.message = f'Falha na gravação: {exc}'
            self.get_logger().error(response.message)
        return response

    def enable_anomaly_detection_callback(self, request, response):
        """Habilita o segundo estágio sem bloquear a espera de detecção."""
        self._anomaly_detection_enabled = request.enable
        response.success = True
        response.message = 'Detecção de anomalias ' + ('habilitada' if request.enable else 'desabilitada')
        return response

    def cv_models_service_callback(self, request, response):
        """Expõe apenas modelos efetivamente carregados como seleção atual."""
        response.models_data_json = json.dumps(self._registry.entries if self._registry else [])
        current = self._models.filenames if self._models else ('', '')
        response.current_object_model, response.current_anomaly_model = current
        return response

    def request_shutdown(self):
        """Libera consultas antes que o executor espere pelos callbacks pendentes."""
        self._is_shutting_down = True
        self._detections.close()

    def destroy_node(self):
        """O mesmo fechamento idempotente atende término normal e erro de execução."""
        self.request_shutdown()
        self._recorder.close()
        return super().destroy_node()


def main(args=None):
    """Mantém o handler de sinais do rclpy e aguarda término dos callbacks."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = CVNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.request_shutdown()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
