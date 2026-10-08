"""Fronteira ROS do gateway: recebe dados e delega operações aos nós existentes."""

import json
import time
from datetime import datetime
from pathlib import Path
from threading import Event, RLock, Thread

from .api import GatewayError


class ROSAdapter:
    """O executor ROS permanece separado das requisições HTTP limitadas."""

    def __init__(self, state, *, media_dir, ros_args=None, video_fps=None):
        """Conecta as interfaces ROS existentes e inicia um executor separado."""
        import rclpy
        from cv_bridge import CvBridge
        from rcl_interfaces.msg import Log
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from drone_inspetor.media.recorder import VideoRecorder
        from drone_inspetor.ros_interfaces import (
            Topics, create_client_from, create_publisher_from, create_subscription_from,
        )
        from drone_inspetor.subscribers.dashboard_monitor_subscriber import (
            DashboardMonitorSubscriber, _message_values,
        )
        self.rclpy, self.state = rclpy, state
        rclpy.init(args=ros_args)
        self.node = Node('mobile_gateway')
        self._values = _message_values
        self.bridge = CvBridge()
        self.media_dir = Path(media_dir).expanduser()
        self.recorder = VideoRecorder('MJPG', 15)
        self._frame_at = {}
        # O JPEG HTTP conserva sua cadência econômica. WebRTC precisa amostrar
        # mais quadros, mas a câmera nunca espera a codificação ou um cliente.
        self._frame_interval = 1 / video_fps if video_fps else 0.3
        self._depth_interval = 1 / video_fps if video_fps else 0.5
        self._record_at = 0.0
        self._model_cache = (0.0, None)
        self._model_lock = RLock()
        self.monitor = DashboardMonitorSubscriber(self.node, state.monitor)
        self.subscriptions = []
        subscriptions = (
            (Topics.PX4.VEHICLE_GLOBAL_POSITION, lambda msg: state.update_extra(
                'global', self._values(msg), label='Posição global',
                topic=Topics.PX4.VEHICLE_GLOBAL_POSITION.name)),
            (Topics.Interno.LIDAR_DATA, self._radar),
            (Topics.Interno.CAMERA_COMPRESSED, lambda msg: self._compressed('camera', msg)),
            (Topics.Interno.CV_COMPRESSED, lambda msg: self._compressed('cv', msg)),
            (Topics.Interno.DEPTH_IMAGE_PROCESSED, self._depth),
            (Topics.Interno.CAMERA_RECORDING, lambda msg: state.update_extra(
                'camera_recording', {'recording': msg.data}, label='Gravação da missão')),
            (Topics.Interno.CV_OBJECT_DETECTIONS, lambda msg: state.update_extra(
                'detections', self._values(msg), label='Detecções CV')),
            (Topics.Interno.CV_ANALYSIS_REPORT, lambda msg: state.event(msg.data)),
        )
        for spec, callback in subscriptions:
            self.subscriptions.append(create_subscription_from(self.node, spec, callback))
        self.subscriptions.append(self.node.create_subscription(Log, '/rosout', self._log, 50))
        self.mission_pub = create_publisher_from(self.node, Topics.Dashboard.MISSION_COMMANDS)
        self.models_pub = create_publisher_from(self.node, Topics.Dashboard.CV_CONTROL)
        self.model_client = create_client_from(self.node, Topics.Service.CV_LIST_MODELS)
        self.record_client = create_client_from(self.node, Topics.Service.CV_RECORD_DETECTIONS)
        self.anomaly_client = create_client_from(self.node, Topics.Service.CV_ENABLE_ANOMALY)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def _log(self, message):
        if message.name in ('drone_node', 'mission_node', 'cv_node'):
            self.state.event(f'{message.name}: {message.msg}',
                             'warning' if message.level >= 30 else 'info')

    def _radar(self, message):
        data = message.point_vector
        self.state.update_radar(zip(data[0::2], data[1::2]))

    def _compressed(self, key, message):
        import cv2
        now = time.monotonic()
        recording = key == 'camera' and self.recorder.active and now - self._record_at >= 1 / 15
        publish = now - self._frame_at.get(key, 0) >= self._frame_interval
        if not publish and not recording:
            return
        try:
            data = bytes(message.data)
            image = None
            if recording or not data.startswith(b'\xff\xd8'):
                image = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding='bgr8')
            if recording:
                self.recorder.write(image)
                self._record_at = now
            if publish:
                if not data.startswith(b'\xff\xd8'):
                    success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not success:
                        return
                    data = encoded.tobytes()
                self.state.put_frame(key, data)
                self._frame_at[key] = now
        except Exception as exc:
            self.node.get_logger().warning(f'Frame móvel inválido: {exc}', throttle_duration_sec=5)

    def _depth(self, message):
        import cv2
        now = time.monotonic()
        if now - self._frame_at.get('depth', 0) < self._depth_interval:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            if str(image.dtype) != 'uint8':
                image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if success:
                self.state.put_frame('depth', encoded.tobytes())
                self._frame_at['depth'] = now
        except Exception as exc:
            self.node.get_logger().warning(
                f'Profundidade móvel inválida: {exc}', throttle_duration_sec=5)

    @staticmethod
    def _call(client, request):
        if not client.service_is_ready():
            raise GatewayError('Serviço ROS indisponível.', 503)
        done = Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(3.0):
            # Um pedido entregue pode concluir tarde. O gateway não o repete.
            raise TimeoutError('Serviço sem resposta no prazo')
        return future.result()

    def models(self):
        """Consulta os modelos disponíveis com cache de dois segundos."""
        from drone_inspetor_msgs.srv import CVModelsSRV
        with self._model_lock:
            at, data = self._model_cache
            if data is not None and time.monotonic() - at < 2:
                return data
            response = self._call(self.model_client, CVModelsSRV.Request())
            entries = json.loads(response.models_data_json)
            if not isinstance(entries, list):
                raise GatewayError('Catálogo CV incompatível.', 502)
            data = {'models': entries, 'current_object_model': response.current_object_model,
                    'current_anomaly_model': response.current_anomaly_model}
            self._model_cache = (time.monotonic(), data)
            return data

    def execute(self, command, args):
        """Encaminha comandos validados sem assumir aceitação dos pedidos por tópico."""
        from drone_inspetor_msgs.msg import CVControlMSG, DashboardMissionCommandMSG
        from drone_inspetor_msgs.srv import EnableAnomalyDetectionSRV, RecordDetectionsSRV
        if command.startswith('mission.'):
            if self.mission_pub.get_subscription_count() == 0:
                raise GatewayError('MissionNode sem assinatura do comando.', 503)
            message = DashboardMissionCommandMSG()
            message.command = 1 if command == 'mission.start' else 2
            message.mission = args.get('mission', '')
            self.mission_pub.publish(message)
            return {'status': 'submitted',
                    'message': 'Pedido enviado à missão; acompanhe o estado.'}
        if command == 'cv.models':
            if self.models_pub.get_subscription_count() == 0:
                raise GatewayError('CVNode sem assinatura de modelos.', 503)
            message = CVControlMSG()
            message.object_detection_model = args['object_model']
            message.anomaly_detection_model = args['anomaly_model']
            self.models_pub.publish(message)
            self._model_cache = (0.0, None)
            return {'status': 'submitted',
                    'message': 'Troca solicitada; confirme os modelos ativos no catálogo.'}
        if command == 'cv.record':
            request = RecordDetectionsSRV.Request()
            request.start_recording = args['enabled']
            response = self._call(self.record_client, request)
            return {'status': 'completed' if response.success else 'rejected',
                    'message': response.message}
        if command == 'cv.anomaly':
            request = EnableAnomalyDetectionSRV.Request()
            request.enable = args['enabled']
            response = self._call(self.anomaly_client, request)
            return {'status': 'completed' if response.success else 'rejected',
                    'message': response.message}
        if command == 'camera.record' and not args['enabled']:
            return self._stop_recording()
        frame = self.state.frame('camera')
        if frame is None:
            raise GatewayError('Câmera sem imagem recente.', 409)
        self.media_dir.mkdir(parents=True, exist_ok=True)
        basename = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        if command == 'camera.capture':
            path = self.media_dir / f'foto_{basename}.jpg'
            with path.open('xb') as stream:
                stream.write(frame)
            return {'status': 'completed', 'message': f'Foto salva no gateway: {path.name}'}
        if command == 'camera.record':
            if self.recorder.active:
                raise GatewayError('Gravação manual já ativa.', 409)
            path = self.media_dir / f'video_{basename}.{self.recorder.extension}'
            self.recorder.start(path)
            return {'status': 'completed',
                    'message': 'Gravador manual preparado para os próximos frames.'}
        raise GatewayError('Comando não implementado.')

    def _stop_recording(self):
        if not self.recorder.active:
            raise GatewayError('Nenhuma gravação manual ativa.', 409)
        opened = self.recorder.opened
        path = Path(self.recorder.close())
        if not opened or not path.is_file() or path.stat().st_size == 0:
            return {'status': 'completed',
                    'message': 'Gravação encerrada sem vídeo disponível; confira a câmera e os logs.'}
        return {'status': 'completed', 'message': f'Vídeo manual salvo no gateway: {path}'}

    def close(self):
        """Encerra o executor, fecha vídeo pendente e libera o contexto ROS."""
        self.executor.shutdown(timeout_sec=5)
        self.recorder.close()
        self.node.destroy_node()
        self.rclpy.try_shutdown()
        self.thread.join(timeout=5)
