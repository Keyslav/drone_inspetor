# =================================================================================================
# action_server.py
# =================================================================================================
# MIXIN: ActionServer callbacks do DroneNode (interface com mission_node)
# =================================================================================================
# Agrupa os callbacks do ActionServer DroneCommand: aceitação, cancelamento, execução
# e feedback. Implementado como Mixin herdado por DroneNode.
#
# Comandos suportados (DroneCommand Action):
#     ARM, TAKEOFF, GOTO (com ou sem foco), LAND, RTL.
#
# Critério de conclusão (`_is_command_complete`) — em termos do NOVO modelo de FSMs:
#     ARM     → lifecycle == POUSADO_ARMADO
#     TAKEOFF → lifecycle == EM_VOO
#     GOTO    → lifecycle == EM_VOO  +  target_stack vazia  +  deslocamento_state == PLANANDO
#     LAND    → lifecycle ∈ {POUSADO_ARMADO, POUSADO_DESARMADO}
#     RTL     → lifecycle == POUSADO_DESARMADO
# =================================================================================================

import math
import time
from typing import TYPE_CHECKING

from rclpy.action import CancelResponse, GoalResponse

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription
from drone_inspetor.common.log_colors import LogPrefix


class DroneActionServerMixin:
    """Mixin com os callbacks do ActionServer DroneCommand."""

    # Handle do goal em execução (None quando idle). Compartilhado pelos callbacks.
    _current_goal_handle = None

    # =============================================================================================
    # Aceitação / cancelamento de goals
    # =============================================================================================

    def goal_callback(self: 'DroneNode', goal_request):
        """
        Aceita ou rejeita um goal com base na validade do comando no estado atual
        (consulta `context.verifica_validade_do_comando`, que conhece VALID_COMMANDS).
        """
        command = goal_request.command
        self.get_logger().info(LogPrefix.mission_rx(f"(DroneCommand Action): {command}"))

        pode, erro = self.drone_fsm_context.verifica_validade_do_comando(command)
        if pode:
            self.get_logger().info(f"Comando do mission_node aceito: {command}")
            return GoalResponse.ACCEPT
        self.get_logger().warn(f"Comando do mission_node rejeitado: {erro}")
        return GoalResponse.REJECT

    def cancel_callback(self: 'DroneNode', goal_handle):
        """Aceita o cancelamento e executa STOP para parar o drone em hover."""
        self.get_logger().info("Cancelamento de action solicitado. Executando STOP...")
        self.stop()
        return CancelResponse.ACCEPT

    # =============================================================================================
    # Execução do goal (loop de feedback)
    # =============================================================================================

    def execute_drone_command_callback(self: 'DroneNode', goal_handle):
        """
        Executa o comando recebido, publicando feedback periódico até a conclusão
        ou ocorrência de timeout/cancelamento.
        """
        self._current_goal_handle = goal_handle
        request = goal_handle.request
        command = request.command

        if not self._execute_command(request):
            result = DroneCommand.Result()
            result.success = False
            result.message = f"Falha ao iniciar comando: {command}"
            result.final_state = int(self.drone_fsm_context.state)
            goal_handle.abort()
            self._current_goal_handle = None
            return result

        # Timeouts por comando (segundos).
        command_timeouts = {
            "ARM": 10.0,
            "TAKEOFF": 60.0,
            "LAND": 120.0,
            "GOTO": 120.0,
            "RTL": 300.0,
        }
        timeout = command_timeouts.get(command, 60.0)
        start_time = self.get_clock().now().nanoseconds / 1e9

        feedback_msg = DroneCommand.Feedback()

        while not self._is_command_complete(command):
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - start_time
            if elapsed > timeout:
                self.get_logger().warn(f"Timeout ({timeout}s) aguardando conclusão de {command}.")
                result = DroneCommand.Result()
                result.success = False
                result.message = f"Timeout ({timeout}s) aguardando conclusão de {command}."
                result.final_state = int(self.drone_fsm_context.state)
                goal_handle.abort()
                self._current_goal_handle = None
                return result

            if goal_handle.is_cancel_requested:
                self.get_logger().info(f"Action cancelada durante execução de {command}.")
                result = DroneCommand.Result()
                result.success = False
                result.message = "Ação cancelada."
                result.final_state = int(self.drone_fsm_context.state)
                goal_handle.canceled()
                self._current_goal_handle = None
                return result

            if self._is_drone_node_shutting_down:
                return DroneCommand.Result()

            try:
                lifecycle = self.drone_fsm_context.state
                feedback_msg.current_state = int(lifecycle)
                feedback_msg.state_name = lifecycle.name
                feedback_msg.distance_to_target = self._calculate_distance_to_target()
                feedback_msg.progress_percent = self._calculate_progress_percent(command)
                goal_handle.publish_feedback(feedback_msg)
            except Exception as e:
                if not self._is_drone_node_shutting_down:
                    self.get_logger().warn(f"Erro ao publicar feedback: {e}")

            time.sleep(0.1)

        result = DroneCommand.Result()
        result.success = True
        result.message = f"Comando {command} completado com sucesso."
        result.final_state = int(self.drone_fsm_context.state)
        goal_handle.succeed()
        self._current_goal_handle = None
        return result

    # =============================================================================================
    # Dispatch (action → método do mixin px4_commands)
    # =============================================================================================

    def _execute_command(self: 'DroneNode', request) -> bool:
        """Despacha cada comando para o método correspondente em DronePX4CommandsMixin."""
        command = request.command
        try:
            match command:
                case "ARM":
                    # Quem aciona o PX4 é o estado POUSADO_DESARMADO ao detectar pending_command.
                    self.drone_fsm_context.pending_command = "ARM"
                case "TAKEOFF":
                    alt = request.altitude if not math.isnan(request.altitude) else None
                    self.takeoff(alt)
                case "GOTO":
                    lat = request.lat if not math.isnan(request.lat) else None
                    lon = request.lon if not math.isnan(request.lon) else None
                    alt = request.alt if not math.isnan(request.alt) else None
                    yaw = request.yaw if not math.isnan(request.yaw) else None
                    if request.use_focus:
                        flat = request.focus_lat if not math.isnan(request.focus_lat) else None
                        flon = request.focus_lon if not math.isnan(request.focus_lon) else None
                        self.goto(
                            lat=lat, lon=lon, alt=alt,
                            use_focus=True, focus_lat=flat, focus_lon=flon,
                        )
                    else:
                        self.goto(lat=lat, lon=lon, alt=alt, yaw=yaw)
                case "LAND":
                    self.land()
                case "RTL":
                    self.rtl()
                case _:
                    self.get_logger().warn(f"Comando não reconhecido: {command}")
                    return False
            return True
        except Exception as e:
            self.get_logger().error(f"Erro ao executar comando {command}: {e}")
            return False

    # =============================================================================================
    # Critério de conclusão por comando
    # =============================================================================================

    def _is_command_complete(self: 'DroneNode', command: str) -> bool:
        """Avalia se um comando ja completou, com base no estado das duas FSMs + TargetStack."""
        lifecycle = self.drone_fsm_context.state

        match command:
            case "ARM":
                return lifecycle == DroneFSMDescription.POUSADO_ARMADO

            case "TAKEOFF":
                # TAKEOFF completa quando entramos em EM_VOO (DECOLANDO → EM_VOO ao atingir altitude).
                return lifecycle == DroneFSMDescription.EM_VOO

            case "GOTO":
                # GOTO completa quando o drone está em EM_VOO, a pilha esvaziou
                # (DeslocamentoFSM concluiu) e voltou ao PLANANDO (hover).
                sctx = self.deslocamento_fsm_context
                return (
                    lifecycle == DroneFSMDescription.EM_VOO
                    and sctx.target_stack.is_empty
                    and sctx.state == DeslocamentoFSMDescription.PLANANDO
                )

            case "LAND":
                return lifecycle in (
                    DroneFSMDescription.POUSADO_ARMADO,
                    DroneFSMDescription.POUSADO_DESARMADO,
                )

            case "RTL":
                return lifecycle == DroneFSMDescription.POUSADO_DESARMADO

            case _:
                return True

    # =============================================================================================
    # Métricas de progresso para feedback
    # =============================================================================================

    def _calculate_distance_to_target(self: 'DroneNode') -> float:
        """Distância 3D (m) até o target ativo. Zero se não houver target ou posição."""
        target = self.deslocamento_fsm_context.target_stack.current
        if target is None or self.state_px4.local_position is None:
            return 0.0
        cur = self.state_px4.local_position
        tx, ty, tz = target.local_position
        return math.sqrt((tx - cur.x) ** 2 + (ty - cur.y) ** 2 + (tz - cur.z) ** 2)

    def _calculate_progress_percent(self: 'DroneNode', command: str) -> float:
        """
        Percentual de progresso para comandos de trajetória (GOTO/RTL).

        Para RTL, como delegamos ao autopilot, não temos `initial_distance_to_target`
        com semântica direta — retorna 0 nesse caso.
        """
        if command != "GOTO":
            return 0.0
        d0 = self.deslocamento_fsm_context.initial_distance_to_target
        if d0 is None or d0 <= 0:
            return 0.0
        d_now = self._calculate_distance_to_target()
        progress = 1.0 - (d_now / d0)
        return max(0.0, min(100.0, progress * 100.0))
