# context.py
# =================================================================================================
# CONTEXTO COMPARTILHADO DA FSM INTERNA DO DRONE
# =================================================================================================
# Contém todos os dados compartilhados entre os estados: telemetria PX4, obstáculos,
# parâmetros de navegação, pilha de waypoints e variáveis de trajetória.
# Cada State recebe uma referência a este contexto via self.context.
# =================================================================================================

import math
from typing import TYPE_CHECKING

from px4_msgs.msg import VehicleStatus

from drone_inspetor.common.enums import DroneStateDescription
from drone_inspetor.nodes.drone_node.px4_state import DroneStatePX4
from drone_inspetor.nodes.drone_node.obstacles import DroneObstacles
from drone_inspetor.nodes.drone_node.waypoint_stack import WaypointStack

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode


# Mapeamento de comandos para estados permitidos
VALID_COMMANDS = {
    "ARM": [DroneStateDescription.POUSADO_DESARMADO],
    "TAKEOFF": [DroneStateDescription.POUSADO_ARMADO],
    "GOTO": [DroneStateDescription.VOANDO_PRONTO],
    "LAND": [DroneStateDescription.VOANDO_PRONTO],
    "RTL": [DroneStateDescription.VOANDO_PRONTO],
}


class DroneFSMContext:
    """
    Dados compartilhados entre todos os estados da FSM interna do drone.
    Encapsula telemetria PX4, obstáculos, parâmetros de navegação e pilha de waypoints.
    """

    def __init__(self, node: 'DroneNode'):
        self.node = node

        # Subsistemas
        self.state_px4 = DroneStatePX4()
        self.obstacles = DroneObstacles(node)

        # Estado atual (gerenciado pela StateMachine, espelhado aqui para acesso rápido)
        self.state = DroneStateDescription.OFFBOARD_DESATIVADO
        self.state_entry_time = self.node.get_clock().now().nanoseconds / 1e9

        # Trajetória
        self.on_trajectory = False
        self.trajectory_start_time = None
        self.yaw_step_deg = 15.0

        # Posições de navegação
        self.origin_local_position = None
        self.target_local_position = None
        self.target_latitude = None
        self.target_longitude = None
        self.target_altitude = None
        self.origin_latitude = None
        self.origin_longitude = None
        self.origin_altitude = None
        self.target_final_yaw_deg = None
        self.target_final_yaw_deg_normalized = None
        self.target_final_yaw_rad = None
        self.position_tolerance = 0.15
        self.yaw_tolerance_deg = 2.0

        # Direção e yaw
        self.target_direction_yaw_deg = None
        self.target_direction_yaw_deg_normalized = None
        self.target_direction_yaw_rad = None
        self.takeoff_altitude = 2.5
        self.rtl_altitude = 30.0
        self.yaw_aligned_time = None
        self.yaw_stabilization_delay = 3.0
        self.initial_distance_to_target = None

        # Última posição estática (hover)
        self.last_static_position = None
        self.last_static_yaw_deg = None
        self.last_static_yaw_deg_normalized = None
        self.last_static_yaw_rad = None

        # Pilha de coordenadas para navegação e desvio de obstáculos
        self.waypoint_stack = WaypointStack()

        # Comando pendente (substitui flags command_*_requested)
        # Valores possíveis: None, "ARM", "TAKEOFF", "GOTO", "LAND", "RTL"
        self.pending_command: str | None = None

        # Flag de foco para o GOTO pendente: quando True, mantém yaw apontando ao foco
        self.pending_use_focus: bool = False

        # Variáveis de foco (GOTO com use_focus=True)
        self.focus_latitude = None
        self.focus_longitude = None
        self.focus_local_position = None
        self.focus_yaw_deg = None
        self.focus_yaw_deg_normalized = None
        self.focus_yaw_rad = None

        # Flag de emergência: True após acionar RTL nativo do PX4. Evita reenvio
        # do comando a cada tick e marca que o controle foi delegado ao autopilot.
        self.emergency_active = False

        # ----- Sistema de desvio de obstáculo (estados *_OBSTACULO_*) -----
        # Distância mínima (m) p/ considerar uma coordenada candidata como
        # "ponto já visitado" durante a checagem de loop nas pilhas abaixo.
        self.obstacle_loop_threshold = 1.0

        # Pilha 1: pontos onde o drone parou por causa de obstáculo
        # Cada elemento: (x, y, z, yaw_deg). Inclui o ponto inicial e cada
        # nova parada caso um novo obstáculo seja detectado em pleno desvio.
        self.obstacle_stop_points: list = []

        # Pilha 2: coordenadas calculadas como desvio
        # Cada elemento: (x, y, z). Inclui todos os desvios já tentados,
        # mesmo que o drone tenha tido que recalcular outro durante a recursão.
        self.detour_calculated_points: list = []

        # Snapshot do destino original (preservado durante o desvio).
        # Esses campos guardam o que estava em target_local_position e nos
        # campos de yaw final ANTES do drone iniciar o desvio. Restaurados
        # no estado *_OBSTACULO_DESVIADO ao retomar a rota.
        self.saved_target_local_position: list | None = None
        self.saved_target_latitude: float | None = None
        self.saved_target_longitude: float | None = None
        self.saved_target_altitude: float | None = None
        self.saved_target_final_yaw_deg: float | None = None
        self.saved_target_final_yaw_deg_normalized: float | None = None
        self.saved_target_final_yaw_rad: float | None = None
        self.saved_focus_local_position: list | None = None
        self.saved_focus_latitude: float | None = None
        self.saved_focus_longitude: float | None = None

    # ------------------------------------------------------------------
    # Utilitários
    # ------------------------------------------------------------------

    def now(self) -> float:
        """Timestamp atual em segundos."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def store_static_position(self):
        """Armazena posição e yaw atuais como referência estática para hover."""
        if self.state_px4.local_position is not None:
            self.last_static_position = [
                self.state_px4.local_position.x,
                self.state_px4.local_position.y,
                self.state_px4.local_position.z,
            ]
            self.last_static_yaw_deg_normalized = self.state_px4.current_yaw_deg_normalized
            self.last_static_yaw_deg = self.state_px4.current_yaw_deg_normalized
            if self.last_static_yaw_deg < 0:
                self.last_static_yaw_deg += 360
            self.last_static_yaw_rad = self.state_px4.current_yaw_rad

    def reset_trajectory_vars(self):
        """Limpa variáveis de trajetória, pilha de waypoints e armazena posição estática para hover."""
        self.store_static_position()
        self.on_trajectory = False
        self.target_latitude = None
        self.target_longitude = None
        self.target_altitude = None
        self.origin_latitude = None
        self.origin_longitude = None
        self.origin_altitude = None
        self.origin_local_position = None
        self.target_local_position = None
        self.target_final_yaw_deg = None
        self.target_final_yaw_deg_normalized = None
        self.target_final_yaw_rad = None
        self.target_direction_yaw_deg = None
        self.target_direction_yaw_deg_normalized = None
        self.target_direction_yaw_rad = None
        self.trajectory_start_time = None
        self.focus_latitude = None
        self.focus_longitude = None
        self.focus_local_position = None
        self.focus_yaw_deg = None
        self.focus_yaw_deg_normalized = None
        self.focus_yaw_rad = None
        self.waypoint_stack.clear()
        self.pending_command = None
        self.pending_use_focus = False
        # Sistema de desvio de obstáculo também é descartado em qualquer reset
        # de trajetória (STOP, perda de offboard, emergência, etc.)
        self.obstacle_stop_points.clear()
        self.detour_calculated_points.clear()
        self.saved_target_local_position = None
        self.saved_target_latitude = None
        self.saved_target_longitude = None
        self.saved_target_altitude = None
        self.saved_target_final_yaw_deg = None
        self.saved_target_final_yaw_deg_normalized = None
        self.saved_target_final_yaw_rad = None
        self.saved_focus_local_position = None
        self.saved_focus_latitude = None
        self.saved_focus_longitude = None

    def apply_waypoint(self, wp) -> None:
        """
        Carrega as variáveis de navegação a partir de um Waypoint da pilha.
        Atualiza target_local_position, yaw de direção, yaw final e foco.

        Args:
            wp: Waypoint da WaypointStack
        """
        self.target_local_position = list(wp.local_position)
        self.target_latitude = wp.latitude
        self.target_longitude = wp.longitude
        self.target_altitude = wp.altitude

        # Yaw de direção: recalcula a partir da posição atual
        if self.state_px4.local_position is not None:
            dx = wp.local_position[0] - self.state_px4.local_position.x
            dy = wp.local_position[1] - self.state_px4.local_position.y
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                direction_yaw_deg = math.degrees(math.atan2(dy, dx))
                if direction_yaw_deg > 180:
                    direction_yaw_deg -= 360
                elif direction_yaw_deg < -180:
                    direction_yaw_deg += 360
                self.target_direction_yaw_deg_normalized = direction_yaw_deg
                self.target_direction_yaw_deg = direction_yaw_deg if direction_yaw_deg >= 0 else direction_yaw_deg + 360
                self.target_direction_yaw_rad = math.radians(direction_yaw_deg)
            else:
                self.target_direction_yaw_deg = None
                self.target_direction_yaw_deg_normalized = None
                self.target_direction_yaw_rad = None
        elif wp.direction_yaw_deg is not None:
            self.target_direction_yaw_deg = wp.direction_yaw_deg
            self.target_direction_yaw_deg_normalized = wp.direction_yaw_deg_normalized
            self.target_direction_yaw_rad = wp.direction_yaw_rad

        # Yaw final
        if wp.final_yaw_deg is not None:
            self.target_final_yaw_deg = wp.final_yaw_deg
            self.target_final_yaw_deg_normalized = wp.final_yaw_deg_normalized
            self.target_final_yaw_rad = wp.final_yaw_rad
        else:
            self.target_final_yaw_deg = None
            self.target_final_yaw_deg_normalized = None
            self.target_final_yaw_rad = None

        # Foco (GOTO com use_focus=True)
        if wp.focus_local_position is not None:
            self.focus_local_position = list(wp.focus_local_position)
            self.focus_latitude = wp.focus_latitude
            self.focus_longitude = wp.focus_longitude

        # Distância inicial (para cálculo de progresso)
        if self.state_px4.local_position is not None:
            dx = wp.local_position[0] - self.state_px4.local_position.x
            dy = wp.local_position[1] - self.state_px4.local_position.y
            dz = wp.local_position[2] - self.state_px4.local_position.z
            self.initial_distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

        # Origem
        if self.state_px4.local_position is not None:
            self.origin_local_position = [
                self.state_px4.local_position.x,
                self.state_px4.local_position.y,
                self.state_px4.local_position.z,
            ]

    def yaw_diff_shortest(self, current_deg: float, target_deg: float) -> float:
        """Diferença angular pelo caminho mais curto (-180 a 180). Delega para math_utils."""
        from drone_inspetor.common.math_utils import yaw_diff_shortest
        return yaw_diff_shortest(current_deg, target_deg)

    # ------------------------------------------------------------------
    # Helpers do sistema de desvio de obstáculo
    # ------------------------------------------------------------------

    def save_original_target(self) -> None:
        """
        Tira um snapshot do destino original antes de iniciar um desvio.
        Só salva se ainda não houver snapshot (ou seja, na primeira detecção
        de obstáculo da sequência — recursões durante o desvio NÃO sobrescrevem).
        """
        if self.saved_target_local_position is not None:
            return
        if self.target_local_position is not None:
            self.saved_target_local_position = list(self.target_local_position)
        self.saved_target_latitude = self.target_latitude
        self.saved_target_longitude = self.target_longitude
        self.saved_target_altitude = self.target_altitude
        self.saved_target_final_yaw_deg = self.target_final_yaw_deg
        self.saved_target_final_yaw_deg_normalized = self.target_final_yaw_deg_normalized
        self.saved_target_final_yaw_rad = self.target_final_yaw_rad
        if self.focus_local_position is not None:
            self.saved_focus_local_position = list(self.focus_local_position)
        self.saved_focus_latitude = self.focus_latitude
        self.saved_focus_longitude = self.focus_longitude

    def restore_original_target(self) -> None:
        """
        Restaura o destino original a partir do snapshot. Chamado no
        *_OBSTACULO_DESVIADO ao confirmar que o caminho está livre.
        """
        if self.saved_target_local_position is not None:
            self.target_local_position = list(self.saved_target_local_position)
        self.target_latitude = self.saved_target_latitude
        self.target_longitude = self.saved_target_longitude
        self.target_altitude = self.saved_target_altitude
        self.target_final_yaw_deg = self.saved_target_final_yaw_deg
        self.target_final_yaw_deg_normalized = self.saved_target_final_yaw_deg_normalized
        self.target_final_yaw_rad = self.saved_target_final_yaw_rad
        if self.saved_focus_local_position is not None:
            self.focus_local_position = list(self.saved_focus_local_position)
        self.focus_latitude = self.saved_focus_latitude
        self.focus_longitude = self.saved_focus_longitude

    def register_obstacle_stop_point(self) -> None:
        """Empilha posição+yaw atuais na lista de pontos de parada por obstáculo."""
        if self.state_px4.local_position is None:
            return
        self.obstacle_stop_points.append((
            self.state_px4.local_position.x,
            self.state_px4.local_position.y,
            self.state_px4.local_position.z,
            self.state_px4.current_yaw_deg_normalized,
        ))

    def is_detour_loop(self, candidate_pos) -> bool:
        """
        Verifica se uma coordenada candidata a desvio caracteriza loop.
        Critério: candidato a menos de obstacle_loop_threshold (m) de qualquer
        ponto de parada por obstáculo OU de qualquer desvio já calculado.
        """
        cx, cy, cz = candidate_pos[0], candidate_pos[1], candidate_pos[2]
        threshold = self.obstacle_loop_threshold

        for (px, py, pz, _yaw) in self.obstacle_stop_points:
            dx, dy, dz = cx - px, cy - py, cz - pz
            if math.sqrt(dx * dx + dy * dy + dz * dz) < threshold:
                return True

        for (px, py, pz) in self.detour_calculated_points:
            dx, dy, dz = cx - px, cy - py, cz - pz
            if math.sqrt(dx * dx + dy * dy + dz * dz) < threshold:
                return True

        return False

    def register_detour_point(self, pos) -> None:
        """Empilha uma coordenada calculada como desvio (após validação anti-loop)."""
        self.detour_calculated_points.append((pos[0], pos[1], pos[2]))

    def clear_obstacle_avoidance_state(self) -> None:
        """
        Limpa todo o estado do sistema de desvio: as duas pilhas e o snapshot
        do destino original. Chamado em *_OBSTACULO_DESVIADO quando o caminho
        está livre, em STOP, ou em reset_trajectory_vars().
        """
        self.obstacle_stop_points.clear()
        self.detour_calculated_points.clear()
        self.saved_target_local_position = None
        self.saved_target_latitude = None
        self.saved_target_longitude = None
        self.saved_target_altitude = None
        self.saved_target_final_yaw_deg = None
        self.saved_target_final_yaw_deg_normalized = None
        self.saved_target_final_yaw_rad = None
        self.saved_focus_local_position = None
        self.saved_focus_latitude = None
        self.saved_focus_longitude = None

    def verifica_condicao_de_emergencia(self) -> bool:
        """
        Verifica condições de emergência (bateria crítica, etc.).
        Retorna True apenas na PRIMEIRA detecção; depois disso a flag
        emergency_active suprime novas detecções para evitar reenvio do RTL.
        """
        if self.emergency_active:
            return False
        if self.state_px4.battery_status and self.state_px4.battery_status.remaining < 0.10:
            self.node.get_logger().warn("EMERGÊNCIA: Bateria crítica!")
            self.emergency_active = True
            return True
        return False

    def verifica_validade_do_comando(self, command: str) -> tuple:
        """
        Verifica se um comando pode ser executado no estado atual.

        Returns:
            (pode_executar, mensagem_erro)
        """
        if self.state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, "Drone NÃO está em modo OFFBOARD."
        if self.state_px4.local_position is None:
            return False, "Posição local desconhecida."
        allowed = VALID_COMMANDS.get(command, [])
        if not allowed:
            return True, ""
        if self.state in allowed:
            return True, ""
        allowed_names = [s.name for s in allowed]
        return False, (
            f"Comando '{command}' não permitido no estado {self.state.name}. "
            f"Estados permitidos: {allowed_names}"
        )
