"""Servidor de comandos com uma reserva atômica e conclusão por telemetria."""

import math
import time

from rclpy.action import CancelResponse, GoalResponse
from px4_msgs.msg import VehicleStatus

from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor.nodes.drone_node.command_manager import ActiveCommand
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import (
    DeslocamentoFSMDescription as TS,
)


class DroneActionServerMixin:
    """Serializa goals; cancelamento freia antes de liberar a próxima operação."""

    def init_action_server_state(self):
        """Inicializa propriedade por nó antes de criar o ActionServer."""
        self._active_command = None
        self._current_goal_handle = None
        self._command_sequence = 0
        self.command_timeouts = {
            'ARM': 10., 'DISARM': 10., 'TAKEOFF': 60., 'GOTO': 120.,
            'LAND': 120., 'RTL': 300., 'STOP': 30.,
        }
        self.command_cancel_timeout = 30.

    @staticmethod
    def _validate_parameters(request):
        """NaN é ausência nas coordenadas; infinito nunca é uma referência válida."""
        if request.command == 'TAKEOFF':
            if not math.isnan(request.altitude) and (
                not math.isfinite(request.altitude) or request.altitude <= 0
            ):
                raise ValueError('TAKEOFF exige altitude positiva e finita, ou NaN para default')
        if request.command != 'GOTO':
            return
        for field in ('lat', 'lon', 'alt', 'yaw', 'focus_lat', 'focus_lon'):
            value = getattr(request, field)
            if math.isinf(value):
                raise ValueError(f'GOTO: {field} não pode ser infinito')
        for field, bound in (('lat', 90), ('lon', 180)):
            value = getattr(request, field)
            if not math.isnan(value) and not -bound <= value <= bound:
                raise ValueError(f'GOTO: {field} fora do intervalo GPS')
        if request.use_focus:
            for field, bound in (('focus_lat', 90), ('focus_lon', 180)):
                value = getattr(request, field)
                if not math.isfinite(value) or not -bound <= value <= bound:
                    raise ValueError(f'GOTO com foco: {field} inválido')

    def goal_callback(self, goal_request):
        """Reserva dentro do lock, antes de qualquer callback de execução."""
        with self._control_lock:
            if self._active_command is not None:
                self.get_logger().warn('Comando rejeitado: outra operação ainda está ativa')
                return GoalResponse.REJECT
            allowed, reason = self.drone_fsm_context.verifica_validade_do_comando(
                goal_request.command
            )
            try:
                self._validate_parameters(goal_request)
            except ValueError as error:
                allowed, reason = False, str(error)
            if not allowed:
                self.get_logger().warn(f'Comando rejeitado: {reason}')
                return GoalResponse.REJECT
            self._command_sequence += 1
            self._active_command = ActiveCommand(self._command_sequence, goal_request)
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Não tenta substituir o autopiloto durante LAND/RTL nativos."""
        with self._control_lock:
            operation = self._active_command
            if operation is None or (
                operation.handle is not None and operation.handle is not goal_handle
            ):
                return CancelResponse.REJECT
            if operation.request.command in ('LAND', 'RTL'):
                self.get_logger().warn('Cancelamento de LAND/RTL nativo não suportado')
                return CancelResponse.REJECT
            operation.cancel_requested = True
            if operation.dispatched:
                self.stop()
            return CancelResponse.ACCEPT

    def execute_drone_command_callback(self, goal_handle):
        """Usa prazo monotônico, identifica falhas e libera somente sua reserva."""
        with self._control_lock:
            operation = self._active_command
            if operation is None or operation.handle is not None:
                return self._finish_command(goal_handle, False, 'Operação sem reserva válida')
            operation.handle = goal_handle
            self._current_goal_handle = goal_handle
        command = operation.request.command
        started = time.monotonic()
        stopping_since = None
        failure = None
        canceled = False
        try:
            with self._control_lock:
                if operation.cancel_requested or goal_handle.is_cancel_requested:
                    canceled = True
                    stopping_since = started
                else:
                    allowed, reason = self.drone_fsm_context.verifica_validade_do_comando(command)
                    if not allowed:
                        return self._finish_command(goal_handle, False, reason)
                    self.trajectory.begin_command()
                    self._px4_command_error = None
                    self._px4_commands_awaiting_ack_log.clear()
                    self._execute_command(operation.request)
                    operation.dispatched = True
            while True:
                with self._control_lock:
                    if self._is_drone_node_shutting_down:
                        return self._finish_command(goal_handle, False, 'Nó em encerramento')
                    if stopping_since is None:
                        canceled = operation.cancel_requested or goal_handle.is_cancel_requested
                        failure = self._command_failure(command)
                        if time.monotonic() - started >= self.command_timeouts[command]:
                            failure = f'Timeout aguardando conclusão de {command}'
                        if canceled or failure:
                            if command in ('LAND', 'RTL'):
                                self._release_failed_native_handover()
                                return self._finish_command(
                                    goal_handle, False, failure or 'Cancelado'
                                )
                            self.stop()
                            stopping_since = time.monotonic()
                        elif self._is_command_complete(command):
                            return self._finish_command(
                                goal_handle, True, f'Comando {command} concluído'
                            )
                    if stopping_since is not None:
                        physically_stopped = self.trajectory.stopped or not self.state_px4.is_armed
                        # rclpy marca CANCELING depois que cancel_callback retorna.
                        # Não conclua como CANCELED antes dessa atualização concorrente.
                        cancellation_ready = not canceled or goal_handle.is_cancel_requested
                        if physically_stopped and cancellation_ready:
                            return self._finish_command(
                                goal_handle, False, failure or 'Ação cancelada', canceled=canceled
                            )
                        if time.monotonic() - stopping_since >= self.command_cancel_timeout:
                            return self._finish_command(
                                goal_handle, False,
                                (failure or 'Cancelamento') + '; frenagem não confirmada',
                                canceled=canceled and goal_handle.is_cancel_requested,
                            )
                    self._publish_command_feedback(goal_handle, command)
                # Fora do lock: callbacks de telemetria e timers precisam avançar
                # para confirmar conclusão/frenagem enquanto este goal aguarda.
                time.sleep(0.05)
        except Exception as error:
            self.get_logger().error(f'Falha na operação {command}: {error}')
            with self._control_lock:
                if command not in ('LAND', 'RTL'):
                    self.stop()
                else:
                    self._release_failed_native_handover()
                return self._finish_command(goal_handle, False, str(error))
        finally:
            with self._control_lock:
                if self._active_command is operation:
                    self._active_command = None
                    self._current_goal_handle = None

    def _release_failed_native_handover(self):
        """Se AUTO nunca foi assumido, permite nova tentativa sem prender a admissão."""
        context = self.drone_fsm_context
        offboard = self.state_px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if offboard and not context.native_mode_observed:
            context.native_command = None
            self.stop()
        # Em AUTO o PX4 mantém a autoridade, mesmo após timeout do cliente.

    def _command_failure(self, command):
        """Falhas têm prioridade sobre uma pilha vazia ou um estado de chegada."""
        if self._px4_command_error:
            return self._px4_command_error
        context = self.drone_fsm_context
        if context.state == DS.EMERGENCIA:
            return 'Controle transferido ao failsafe'
        if context.state == DS.OFFBOARD_DESATIVADO:
            return 'Controle OFFBOARD perdido'
        if not self.telemetry_fresh():
            detail = self.telemetry_failure_detail()
            return f'Telemetria local expirada ({detail})'
        if command in ('GOTO', 'TAKEOFF', 'STOP') and self.trajectory.navigation_error:
            return self.trajectory.navigation_error
        if command in ('GOTO', 'TAKEOFF') and not self.state_px4.is_armed:
            return 'Drone desarmado durante movimento'
        return None

    def _finish_command(self, handle, success, message, *, canceled=False):
        result = DroneCommand.Result()
        result.success = success
        result.message = message
        result.final_state = int(self.drone_fsm_context.state)
        if canceled:
            handle.canceled()
        elif success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def _execute_command(self, request):
        """Erros de validação/preparo chegam ao resultado, sem falsos sucessos."""
        self._validate_parameters(request)
        command = request.command
        if command == 'ARM':
            self.drone_fsm_context.pending_command = 'ARM'
        elif command == 'DISARM':
            self.disarm_drone()
        elif command == 'TAKEOFF':
            self.takeoff(None if math.isnan(request.altitude) else request.altitude)
        elif command == 'GOTO':
            self.goto(
                request.lat, request.lon, request.alt, request.yaw,
                request.use_focus, request.focus_lat, request.focus_lon,
            )
        elif command == 'STOP':
            self.stop()
        elif command == 'LAND':
            self.land()
        elif command == 'RTL':
            self.rtl()
        else:
            raise ValueError(f'Comando não suportado: {command}')

    def _is_command_complete(self, command):
        state = self.drone_fsm_context.state
        px4 = self.state_px4
        if command == 'ARM':
            return px4.is_armed and px4.is_landed and state == DS.POUSADO_ARMADO
        if command == 'DISARM':
            return not px4.is_armed and px4.is_landed
        if command == 'TAKEOFF':
            return state == DS.EM_VOO and self.trajectory.stopped
        if command == 'GOTO':
            return all((
                state == DS.EM_VOO,
                self.deslocamento_fsm_context.target_stack.is_empty,
                self.deslocamento_fsm_context.state == TS.PLANANDO,
                self.trajectory.stopped,
            ))
        if command == 'STOP':
            return self.trajectory.stopped
        if command == 'LAND':
            return px4.is_landed and state in (DS.POUSADO_ARMADO, DS.POUSADO_DESARMADO)
        if command == 'RTL':
            return px4.is_landed and not px4.is_armed and state == DS.POUSADO_DESARMADO
        return False

    def _publish_command_feedback(self, goal_handle, command):
        feedback = DroneCommand.Feedback()
        state = self.drone_fsm_context.state
        feedback.current_state = int(state)
        feedback.state_name = state.name
        feedback.distance_to_target = self._calculate_distance_to_target()
        feedback.progress_percent = self._calculate_progress_percent(command)
        goal_handle.publish_feedback(feedback)

    def _calculate_distance_to_target(self):
        target = self.deslocamento_fsm_context.target_stack.current
        position = self.state_px4.local_position
        if target is None or position is None:
            return 0.0
        return math.dist(target.local_position, (position.x, position.y, position.z))

    def _calculate_progress_percent(self, command):
        initial = self.deslocamento_fsm_context.initial_distance_to_target
        if command != 'GOTO' or initial is None or initial <= 0:
            return 0.0
        return max(0., min(100., 100. * (1. - self._calculate_distance_to_target() / initial)))
