"""Adaptador da câmera: publicação ROS e mídia associada à missão atual."""

from datetime import datetime
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_inspetor.common.param_utils import load_param
from drone_inspetor.media.photos import filename_component, save_photo
from drone_inspetor.media.recorder import VideoRecorder
from drone_inspetor.ros_interfaces import (
    Topics, create_publisher_from, create_subscription_from,
)


class CameraNode(Node):
    """Callbacks serializados; o gravador é o único proprietário do VideoWriter."""

    def __init__(self):
        super().__init__('camera_node')
        self._photo_format = load_param(self, 'photo_format', 'jpg')
        self._photo_quality = load_param(self, 'photo_quality', 95)
        self._video_enabled = load_param(self, 'video_enabled', True)
        self._recorder = VideoRecorder(
            load_param(self, 'video_codec', 'MJPG'), load_param(self, 'video_fps', 15),
        )
        self.bridge = CvBridge()
        self._current_mission_state = ''
        self._on_mission = False
        self._is_shutting_down = False
        self._last_image = None
        self._photo_counter = 0
        self._ponto_indice_atual = 0
        self._objeto_alvo = ''
        self._mission_folder_path = ''
        self._photos_folder = None
        self._videos_folder = None
        self._recording_status = False
        self.camera_raw_sub = create_subscription_from(
            self, Topics.Externo.CAMERA_COMPRESSED, self.camera_raw_callback,
        )
        self.mission_state_sub = create_subscription_from(
            self, Topics.Interno.MISSION_STATE, self.mission_state_callback,
        )
        self.dashboard_camera_image_pub = create_publisher_from(self, Topics.Interno.CAMERA_COMPRESSED)
        self.recording_status_pub = create_publisher_from(self, Topics.Interno.CAMERA_RECORDING)

    def _start_video_recording(self):
        """Prepara a sessão; o status só indica gravação após abrir o primeiro frame."""
        if not self._video_enabled or self._videos_folder is None or self._recorder.active:
            return
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = self._videos_folder / f'mission_{timestamp}.{self._recorder.extension}'
        try:
            self._recorder.start(path)
        except (OSError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f'Falha ao preparar vídeo: {exc}')

    def _write_video_frame(self, image):
        try:
            if self._recorder.write(image) and not self._recording_status:
                self._publish_recording_status(True)
        except Exception as exc:
            self._stop_video_recording()
            self.get_logger().error(f'Falha ao gravar frame: {exc}')

    def _stop_video_recording(self):
        self._recorder.close()
        if self._recording_status and not self._is_shutting_down:
            self._publish_recording_status(False)
        self._recording_status = False

    def _publish_recording_status(self, recording):
        message = Bool()
        message.data = recording
        self.recording_status_pub.publish(message)
        self._recording_status = recording

    def _capture_photo(self, state_name):
        if self._last_image is None or self._photos_folder is None:
            return
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(self._last_image, desired_encoding='bgr8')
            self._photo_counter += 1
            timestamp = datetime.now().strftime('%H%M%S_%f')
            state = state_name.lower().replace('executando_inspecionando_', '')
            filename = (f'{self._photo_counter:03d}_P{self._ponto_indice_atual + 1:02d}_'
                        f'{state}_{filename_component(self._objeto_alvo)}_{timestamp}.'
                        f'{self._photo_format}')
            save_photo(self._photos_folder / filename, image, self._photo_quality)
        except (OSError, ValueError, cv2.error) as exc:
            self.get_logger().error(f'Falha ao capturar fotografia: {exc}')

    def camera_raw_callback(self, msg):
        """Republica o payload original e decodifica somente durante uma gravação."""
        if self._is_shutting_down:
            return
        self._last_image = msg
        self.dashboard_camera_image_pub.publish(msg)
        if self._recorder.active:
            try:
                image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self._write_video_frame(image)
            except Exception as exc:
                self.get_logger().error(f'Imagem inválida para vídeo: {exc}', throttle_duration_sec=5)

    def mission_state_callback(self, msg):
        """Abre em decolagem e mantém vídeo durante retorno até o término da missão."""
        if self._is_shutting_down:
            return
        if msg.on_mission:
            changed = msg.mission_folder_path != self._mission_folder_path
            if not self._on_mission or changed:
                was_recording = self._recorder.active
                self._stop_video_recording()
                self._mission_folder_path = msg.mission_folder_path
                folder = Path(msg.mission_folder_path) if msg.mission_folder_path else None
                self._photos_folder = folder / 'fotos' if folder else None
                self._videos_folder = folder / 'videos' if folder else None
                self._photo_counter = 0
                if was_recording:
                    self._start_video_recording()
            self._ponto_indice_atual = msg.ponto_de_inspecao_indice_atual
            self._objeto_alvo = msg.objeto_alvo
            if msg.state_name == 'EXECUTANDO_DECOLANDO':
                self._start_video_recording()
        if not msg.on_mission or msg.state_name in ('DESATIVADO', 'PRONTO'):
            self._stop_video_recording()
        if not msg.on_mission:
            self._mission_folder_path = ''
            self._photos_folder = None
            self._videos_folder = None
        if (msg.on_mission and msg.state_name == 'EXECUTANDO_INSPECIONANDO_DETECTANDO'
                and self._current_mission_state == 'EXECUTANDO_INSPECIONANDO'):
            self._capture_photo(msg.state_name)
        self._on_mission = msg.on_mission
        self._current_mission_state = msg.state_name

    def destroy_node(self):
        """Fecha o vídeo inclusive quando a missão não enviou estado final."""
        self._is_shutting_down = True
        self._recorder.close()
        return super().destroy_node()


def main(args=None):
    """Preserva sinais de shutdown do rclpy; erros inesperados permanecem visíveis."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
