"""Abre apenas o monitor Qt e assinaturas de telemetria, sem comandos de voo."""

import signal
import sys
from threading import Thread

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from drone_inspetor.gui.monitor_screen import MonitorWindow
from drone_inspetor.gui.presentation.telemetry import MonitorStore
from drone_inspetor.subscribers.dashboard_monitor_subscriber import DashboardMonitorSubscriber


def main(args=None):
    """Executa com ``ros2 run drone_inspetor monitor_node`` e remaps ROS usuais."""
    rclpy.init(args=args)
    app = QApplication(remove_ros_args(args=sys.argv if args is None else args))
    node = Node('monitor_node')
    store = MonitorStore()
    subscriber = DashboardMonitorSubscriber(node, store)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = Thread(target=spin, name='monitor-ros', daemon=True)
    window = MonitorWindow(store)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: app.quit() if not rclpy.ok() else None)
    timer.start(200)
    try:
        thread.start()
        window.show()
        return app.exec()
    finally:
        window.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.try_shutdown()
        del subscriber


if __name__ == '__main__':
    sys.exit(main())
