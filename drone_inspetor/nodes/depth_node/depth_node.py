"""Adaptador ROS da profundidade: imagem/JSON para GUI e scan métrico para navegação.

A navegação só recebe distâncias quando a calibração está explícita. O scan é
publicado no callback da imagem original, com o mesmo timestamp; nenhum timer
republica leituras antigas como se fossem novas. Flags booleanas são legado da GUI.
"""

from dataclasses import replace
from datetime import datetime
import json
import math

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from drone_inspetor.common.param_utils import load_param
from drone_inspetor.nodes.depth_node.processing import (
    depth_statistics, filtered_depth, proximity_alerts, render_depth,
)
from drone_inspetor.nodes.depth_node.projection import (
    DepthCalibration, depth_in_meters, project_depth_scan,
)
from drone_inspetor.nodes.lidar_node.processing import sector_flags
from drone_inspetor.ros_interfaces import Topics, create_publisher_from, create_subscription_from
from drone_inspetor_msgs.msg import ObstaclesMSG


class DepthNode(Node):
    """Separa parsing, geometria, análise e transporte em componentes explícitos."""

    def __init__(self):
        super().__init__('depth_node')
        self.bridge = CvBridge()
        self.proximity_threshold = load_param(self, 'proximity_threshold', 1.0)
        self.min_depth = load_param(self, 'min_depth', 0.1)
        self.max_depth = load_param(self, 'max_depth', 10.0)
        self.visualization_mode = load_param(self, 'visualization_mode', 'grayscale')
        self._scan_enabled = load_param(self, 'scan_enabled', True)
        self._scan_frame_id = load_param(self, 'scan_frame_id', 'depth_scan_flu')
        self._calibration = DepthCalibration(
            horizontal_fov_deg=load_param(self, 'scan_horizontal_fov_deg', 0.0),
            fx=load_param(self, 'scan_fx', 0.0), fy=load_param(self, 'scan_fy', 0.0),
            cx=load_param(self, 'scan_cx', -1.0), cy=load_param(self, 'scan_cy', -1.0),
            width=load_param(self, 'scan_calibration_width', 0),
            height=load_param(self, 'scan_calibration_height', 0),
            mount_yaw_deg=load_param(self, 'scan_mount_yaw_deg', 0.0),
            camera_height_m=load_param(self, 'scan_camera_height_m', 0.0),
            band_half_height_m=load_param(self, 'scan_band_half_height_m', 0.35),
            bins=load_param(self, 'scan_bins', 181),
            min_depth=self.min_depth, max_depth=self.max_depth,
        )
        if not math.isfinite(self.proximity_threshold) or self.proximity_threshold <= 0:
            raise ValueError('Threshold de proximidade deve ser positivo')
        if self.visualization_mode not in ('grayscale', 'colormap'):
            raise ValueError('Modo de visualização deve ser grayscale ou colormap')
        if not self._scan_frame_id.strip():
            raise ValueError('Frame FLU do scan de profundidade deve estar definido')
        if self._scan_enabled and not self._calibration.calibrated:
            self.get_logger().warn('Depth sem calibração: GUI ativa, scan de navegação indisponível')
        self.depth_statistics = {}
        self.proximity_alerts = []
        self._obstacle_flags = sector_flags([], [])
        self.depth_subscription = create_subscription_from(
            self, Topics.Externo.DEPTH_CAMERA_IMAGE_RAW, self.depth_image_callback,
        )
        self.processed_depth_publisher = create_publisher_from(self, Topics.Interno.DEPTH_IMAGE_PROCESSED)
        self.depth_stats_publisher = create_publisher_from(self, Topics.Interno.DEPTH_STATISTICS)
        self.proximity_alerts_publisher = create_publisher_from(self, Topics.Interno.DEPTH_PROXIMITY_ALERTS)
        self.obstacles_publisher = create_publisher_from(self, Topics.Interno.DEPTH_OBSTACLE_DETECTIONS)
        self.scan_publisher = create_publisher_from(self, Topics.Interno.DEPTH_SCAN)

    def depth_image_callback(self, msg):
        """O header da imagem e o stamp do scan preservam a aquisição da câmera."""
        try:
            pixels = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = depth_in_meters(pixels, msg.encoding)
        except Exception as exc:
            self.get_logger().error(f'Imagem depth inválida: {exc}')
            return
        # Publicação métrica não depende do sucesso do renderer do dashboard.
        if self._scan_enabled and self._calibration.calibrated:
            try:
                scan = project_depth_scan(depth, self._calibration)
                message = LaserScan()
                message.header.stamp = msg.header.stamp
                message.header.frame_id = self._scan_frame_id
                message.angle_min = scan.angle_min
                message.angle_max = scan.angle_max
                message.angle_increment = scan.angle_increment
                message.range_min = scan.range_min
                message.range_max = scan.range_max
                message.ranges = list(scan.ranges)
                self.scan_publisher.publish(message)
                angles = [scan.angle_min + index * scan.angle_increment
                          for index in range(len(scan.ranges))]
                self._obstacle_flags = sector_flags(scan.ranges, angles)
            except ValueError as exc:
                self._obstacle_flags = sector_flags([], [])
                self.get_logger().error(f'Falha na projeção depth: {exc}')
        else:
            self._obstacle_flags = sector_flags([], [])
        self.publish_obstacles()
        try:
            image, statistics, alerts = self.process_depth_image(depth)
            self.depth_statistics = statistics
            self.proximity_alerts = alerts
            processed = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            processed.header = msg.header
            self.processed_depth_publisher.publish(processed)
            self.publish_depth_statistics()
            self.publish_proximity_alerts(alerts)
        except Exception as exc:
            self.get_logger().error(f'Falha na visualização depth: {exc}')

    def process_depth_image(self, depth):
        """Análise pura usa a imagem inteira; navegação seleciona a faixa vertical."""
        filtered = filtered_depth(depth, self.min_depth, self.max_depth)
        timestamp = datetime.now().strftime('%H:%M:%S')
        statistics = depth_statistics(filtered, timestamp)
        alerts = proximity_alerts(filtered, self.proximity_threshold, timestamp)
        image = render_depth(filtered, statistics, alerts, self.visualization_mode)
        return image, statistics, alerts

    def publish_obstacles(self):
        """Snapshot legado; desconhecido não tem representação na mensagem booleana."""
        message = ObstaclesMSG()
        for name, value in self._obstacle_flags.items():
            setattr(message, name, value)
        self.obstacles_publisher.publish(message)

    def publish_proximity_alerts(self, alerts):
        message = String()
        message.data = json.dumps(alerts, allow_nan=False)
        self.proximity_alerts_publisher.publish(message)

    def publish_depth_statistics(self):
        message = String()
        message.data = json.dumps(self.depth_statistics, allow_nan=False)
        self.depth_stats_publisher.publish(message)

    def set_proximity_threshold(self, threshold):
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError('Threshold deve ser positivo')
        self.proximity_threshold = threshold

    def set_depth_range(self, minimum, maximum):
        calibration = replace(self._calibration, min_depth=minimum, max_depth=maximum)
        self.min_depth, self.max_depth = minimum, maximum
        self._calibration = calibration

    def set_visualization_mode(self, mode):
        if mode not in ('grayscale', 'colormap'):
            raise ValueError('Modo de visualização desconhecido')
        self.visualization_mode = mode

    def toggle_visualization_mode(self):
        self.set_visualization_mode('colormap' if self.visualization_mode == 'grayscale' else 'grayscale')

    def reset_depth_data(self):
        self.depth_statistics = {}
        self.proximity_alerts = []
        self._obstacle_flags = sector_flags([], [])


def main(args=None):
    """Finaliza recursos e preserva erros operacionais para diagnóstico."""
    rclpy.init(args=args)
    node = None
    try:
        node = DepthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
