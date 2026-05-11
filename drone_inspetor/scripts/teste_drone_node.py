#!/usr/bin/env python3
# =================================================================================================
# teste_drone_node.py
# =================================================================================================
# Script interativo para testar o drone_node sem depender do mission_node.
#
# Sequência (cada etapa avança ao pressionar Enter):
#   1. ARM
#   2. TAKEOFF (altitude default 5 m)
#   3. GOTO longe   (~50 m ao norte, mesma altitude)
#   4. GOTO próximo (~5 m ao norte do ponto anterior)
#   5. LAND
#
# USO:
#   ros2 run drone_inspetor teste_drone_node
#
# Pré-requisitos:
#   - drone_node em execução
#   - PX4 conectado e em modo OFFBOARD (após receber o primeiro setpoint)
# =================================================================================================

import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor_msgs.msg import DroneStateMSG

from drone_inspetor.ros_interfaces import Topics, create_subscription_from


# Aproximação local: 1 grau de latitude ≈ 111_111 m. Suficiente para offsets pequenos
# em qualquer latitude. Para longitude, multiplicar por cos(lat).
_METERS_PER_DEG_LAT = 111_111.0


class DroneNodeTester(Node):
    """Cliente interativo do action server do drone_node."""

    def __init__(self):
        super().__init__("drone_node_tester")

        spec = Topics.Action.DRONE_COMMAND
        self._action_client = ActionClient(self, spec.action_type, spec.name)

        # Cache da última telemetria recebida — usada para calcular destinos relativos
        self._last_state: DroneStateMSG | None = None
        create_subscription_from(self, Topics.Interno.DRONE_STATE, self._on_drone_state)

    # ------------------------------------------------------------------
    # Telemetria
    # ------------------------------------------------------------------

    def _on_drone_state(self, msg: DroneStateMSG) -> None:
        self._last_state = msg

    def wait_for_telemetry(self, timeout_s: float = 10.0) -> bool:
        """Bloqueia até receber a primeira DroneStateMSG (com lat/lon válidos)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            s = self._last_state
            if s is not None and not math.isnan(s.current_latitude) \
                    and not math.isnan(s.current_longitude):
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------------
    # Envio de comando (síncrono): bloqueia até resultado/cancelamento
    # ------------------------------------------------------------------

    def send_command(self, goal: DroneCommand.Goal, label: str) -> bool:
        self.get_logger().info(f">>> Enviando: {label}")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server não disponível.")
            return False

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback,
        )
        # Espera ativa do goal handle sem bloquear o executor
        while not send_future.done():
            time.sleep(0.05)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal '{label}' rejeitado pelo drone_node.")
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result().result
        status_msg = "OK" if result.success else "FALHOU"
        self.get_logger().info(
            f"<<< {label}: {status_msg} — {result.message} (final_state={result.final_state})"
        )
        return bool(result.success)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  [feedback] estado={fb.state_name} "
            f"dist={fb.distance_to_target:.2f}m progresso={fb.progress_percent:.1f}%"
        )

    # ------------------------------------------------------------------
    # Construtores de Goal
    # ------------------------------------------------------------------

    @staticmethod
    def goal_arm() -> DroneCommand.Goal:
        g = DroneCommand.Goal()
        g.command = "ARM"
        return g

    @staticmethod
    def goal_takeoff(altitude_m: float) -> DroneCommand.Goal:
        g = DroneCommand.Goal()
        g.command = "TAKEOFF"
        g.altitude = float(altitude_m)
        return g

    @staticmethod
    def goal_goto(lat: float, lon: float, alt: float, yaw_deg: float = float("nan")) -> DroneCommand.Goal:
        g = DroneCommand.Goal()
        g.command = "GOTO"
        g.lat = float(lat)
        g.lon = float(lon)
        g.alt = float(alt)
        g.yaw = float(yaw_deg)
        g.use_focus = False
        return g

    @staticmethod
    def goal_land() -> DroneCommand.Goal:
        g = DroneCommand.Goal()
        g.command = "LAND"
        return g

    # ------------------------------------------------------------------
    # Cálculo de offsets relativos
    # ------------------------------------------------------------------

    def offset_north(self, lat: float, lon: float, alt: float, meters: float) -> tuple:
        """Aplica offset (em metros) ao norte da coordenada base."""
        d_lat = meters / _METERS_PER_DEG_LAT
        return (lat + d_lat, lon, alt)


# ----------------------------------------------------------------------
# Helper de UI: aguarda Enter (ou 'q' para abortar)
# ----------------------------------------------------------------------

def wait_enter(prompt: str) -> bool:
    """Mostra o prompt e aguarda Enter. Retorna False se o usuário digitar 'q'."""
    try:
        resp = input(f"\n[Enter] {prompt}  (ou 'q' para abortar) > ")
    except EOFError:
        return False
    return resp.strip().lower() != "q"


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = DroneNodeTester()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.get_logger().info("Aguardando primeira telemetria do drone_node...")
        if not node.wait_for_telemetry(timeout_s=15.0):
            node.get_logger().error(
                "Sem telemetria após 15s. Verifique se drone_node está rodando e se PX4 publica posição."
            )
            return

        s = node._last_state
        base_lat, base_lon, base_alt = s.current_latitude, s.current_longitude, s.current_altitude
        node.get_logger().info(
            f"Posição inicial: lat={base_lat:.6f} lon={base_lon:.6f} alt={base_alt:.2f}m"
        )

        takeoff_alt_m = 5.0
        takeoff_alt_amsl = base_alt + takeoff_alt_m  # alt absoluta após decolar 5 m

        # Etapa 1: ARM
        if not wait_enter("ARMAR o drone"):
            return
        if not node.send_command(node.goal_arm(), "ARM"):
            return

        # Etapa 2: TAKEOFF
        if not wait_enter(f"DECOLAR para {takeoff_alt_m:.1f} m"):
            return
        if not node.send_command(node.goal_takeoff(takeoff_alt_m), "TAKEOFF"):
            return

        # Etapa 3: GOTO longe (~20 m ao norte)
        far_lat, far_lon, far_alt = node.offset_north(base_lat, base_lon, takeoff_alt_amsl, 20.0)
        if not wait_enter(f"GOTO LONGE: ~20 m ao norte (lat={far_lat:.6f})"):
            return
        if not node.send_command(node.goal_goto(far_lat, far_lon, far_alt), "GOTO_LONGE"):
            return

        # Etapa 4: GOTO próximo (~5 m ao norte do ponto anterior)
        near_lat, near_lon, near_alt = node.offset_north(far_lat, far_lon, far_alt, 5.0)
        if not wait_enter(f"GOTO PRÓXIMO: ~5 m ao norte (lat={near_lat:.6f})"):
            return
        if not node.send_command(node.goal_goto(near_lat, near_lon, near_alt), "GOTO_PROXIMO"):
            return

        # Etapa 5: LAND
        if not wait_enter("POUSAR"):
            return
        node.send_command(node.goal_land(), "LAND")

        node.get_logger().info("Sequência de teste concluída.")

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
