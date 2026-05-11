# action_server.py
# =================================================================================================
# MIXIN: ACTION SERVER CALLBACKS (interface com Mission Node)
# =================================================================================================
# Agrupa os callbacks do ActionServer DroneCommand: aceitação do goal, cancelamento,
# execução e acompanhamento de progresso. Implementado como Mixin herdado por DroneNode.
# =================================================================================================

import math
import time
from typing import TYPE_CHECKING

from rclpy.action import CancelResponse, GoalResponse

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor_msgs.action import DroneCommand
from drone_inspetor.common.enums import DroneStateDescription
from drone_inspetor.common.log_colors import LogPrefix


class DroneActionServerMixin:
    """Mixin com callbacks do ActionServer DroneCommand."""

    # Atributo de classe: referência ao goal ativo (None = nenhum goal em execução)
    _current_goal_handle = None

    # ==================================================================
    # ACTION SERVER CALLBACKS (comunicação com Mission Node)
    # ==================================================================

    def goal_callback(self: 'DroneNode', goal_request):
        """
        Callback para decidir se aceita ou rejeita um goal de action.
        Valida se o comando pode ser executado no estado atual do drone.
        
        Args:
            goal_request: Request contendo o Goal do action
            
        Returns:
            GoalResponse.ACCEPT se o comando é válido, GoalResponse.REJECT caso contrário
        """
        command = goal_request.command
        self.get_logger().info(LogPrefix.mission_rx(f"(DroneCommand Action): {command}"))

        # Valida se o comando é permitido no estado atual
        can_execute, error_msg = self.drone_context.verifica_validade_do_comando(command)

        if can_execute:
            self.get_logger().info(f"Comando do Mission Node aceito: {command}")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(f"Comando do Mission Node rejeitado: {error_msg}")
            return GoalResponse.REJECT

    def cancel_callback(self: 'DroneNode', goal_handle):
        """
        Callback para tratar requisições de cancelamento.
        Aceita o cancelamento e executa o comando STOP para parar o drone.
        
        Args:
            goal_handle: Handle do goal a ser cancelado
            
        Returns:
            CancelResponse.ACCEPT para aceitar o cancelamento
        """
        self.get_logger().info("Cancelamento de action solicitado. Executando STOP...")
        self.stop()  # Função existente que para o drone e limpa a pilha
        return CancelResponse.ACCEPT

    def execute_drone_command_callback(self: 'DroneNode', goal_handle):
        """
        Callback principal que executa o comando.
        Publica feedback periódico durante a execução e retorna o resultado.
        
        COMANDOS SUPORTADOS (DroneCommand Action):
        - command="ARM": Arma os motores
        - command="TAKEOFF", altitude=3.0: Decola até a altitude especificada
        - command="LAND": Pousa na posição atual
        - command="GOTO", lat=..., lon=..., alt=..., yaw=...: Move o drone para posição com yaw final opcional
        - command="GOTO" + use_focus=True + focus_lat/focus_lon: Move o drone mantendo o yaw apontado ao foco
        - command="RTL": Retorna para casa
        
        Args:
            goal_handle: Handle do goal em execução
            
        Returns:
            DroneCommand.Result com o resultado da execução
        """
        self._current_goal_handle = goal_handle
        
        # Extrai parâmetros do goal
        request = goal_handle.request
        command = request.command
        
        # Executa o comando (inicia o movimento/ação)
        success = self._execute_command(request)
        
        if not success:
            result = DroneCommand.Result()
            result.success = False
            result.message = f"Falha ao iniciar comando: {command}"
            result.final_state = int(self.drone_context.state)
            goal_handle.abort()
            self._current_goal_handle = None
            return result
        
        # Para todos os comandos, publica feedback até completar
        feedback_msg = DroneCommand.Feedback()
        
        # Timeouts por tipo de comando (segundos)
        command_timeouts = {
            "ARM": 10.0,
            "TAKEOFF": 60.0,
            "LAND": 60.0,
            "GOTO": 120.0,
            "RTL": 180.0,
        }
        timeout = command_timeouts.get(command, 60.0)
        start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Loop de feedback até o comando completar, cancelar ou timeout
        while not self._is_command_complete(command):
            # Verifica timeout
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - start_time
            if elapsed > timeout:
                self.get_logger().warn(f"Timeout ({timeout}s) aguardando conclusão de {command}")
                result = DroneCommand.Result()
                result.success = False
                result.message = f"Timeout ({timeout}s) aguardando conclusão de {command}"
                result.final_state = int(self.drone_context.state)
                goal_handle.abort()
                self._current_goal_handle = None
                return result
            
            # Verifica cancelamento
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f"Action cancelada durante execução de {command}")
                result = DroneCommand.Result()
                result.success = False
                result.message = "Ação cancelada"
                result.final_state = int(self.drone_context.state)
                goal_handle.canceled()
                self._current_goal_handle = None
                return result
            
            # Atualiza e publica feedback
            if self._is_drone_node_shutting_down:
                return DroneCommand.Result()

            try:
                current_state = self.drone_context.state
                feedback_msg.current_state = int(current_state)
                feedback_msg.state_name = current_state.name
                feedback_msg.distance_to_target = self._calculate_distance_to_target()
                feedback_msg.progress_percent = self._calculate_progress_percent(command)
                goal_handle.publish_feedback(feedback_msg)
            except Exception as e:
                # Ignora erros de feedback se estiver encerrando ou contexto inválido
                if not self._is_drone_node_shutting_down:
                     self.get_logger().warn(f"Erro ao publicar feedback: {e}")
            
            # Aguarda antes do próximo ciclo
            time.sleep(0.1)
        
        # Comando completado com sucesso
        result = DroneCommand.Result()
        result.success = True
        result.message = f"Comando {command} completado com sucesso"
        result.final_state = int(self.drone_context.state)
        goal_handle.succeed()
        self._current_goal_handle = None
        return result

    def _execute_command(self: 'DroneNode', request) -> bool:
        """
        Executa o comando especificado no request da action.
        
        Args:
            request: Request da action contendo comando e parâmetros
            
        Returns:
            bool: True se o comando foi iniciado com sucesso, False caso contrário
        """
        command = request.command
        
        try:
            match command:
                case "ARM":
                    self.arm()
                case "TAKEOFF":
                    # Extrai altitude do request (usa padrão se NaN)
                    alt = request.altitude if not math.isnan(request.altitude) else self.drone_context.takeoff_altitude
                    self.takeoff(alt)
                case "GOTO":
                    # Extrai coordenadas do request (NaN = não especificado)
                    lat = request.lat if not math.isnan(request.lat) else None
                    lon = request.lon if not math.isnan(request.lon) else None
                    alt = request.alt if not math.isnan(request.alt) else None
                    yaw = request.yaw if not math.isnan(request.yaw) else None
                    if request.use_focus:
                        focus_lat = request.focus_lat if not math.isnan(request.focus_lat) else None
                        focus_lon = request.focus_lon if not math.isnan(request.focus_lon) else None
                        self.goto(
                            lat=lat,
                            lon=lon,
                            alt=alt,
                            use_focus=True,
                            focus_lat=focus_lat,
                            focus_lon=focus_lon,
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

    def _is_command_complete(self: 'DroneNode', command: str) -> bool:
        """
        Verifica se o comando foi completado.
        
        A lógica depende do tipo de comando:
        - ARM: drone está armado (POUSADO_ARMADO)
        - TAKEOFF: drone está voando pronto (VOANDO_PRONTO)
        - GOTO: drone está voando pronto E flag de requisição foi limpa
        - LAND: drone está pousado (POUSADO_ARMADO ou POUSADO_DESARMADO)
        - RTL: drone está pousado e desarmado (POUSADO_DESARMADO)
        
        Args:
            command: Comando que está sendo executado
            
        Returns:
            bool: True se o comando foi completado
        """
        # Importa o enum localmente para evitar dependência circular
        from drone_inspetor.common.enums import DroneStateDescription
        
        current_state = self.drone_context.state
        
        match command:
            case "ARM":
                return current_state == DroneStateDescription.POUSADO_ARMADO
            
            case "TAKEOFF":
                return current_state == DroneStateDescription.VOANDO_PRONTO
            
            case "GOTO":
                # GOTO completo quando:
                # 1. Estado é VOANDO_PRONTO (terminou trajetória)
                # 2. Pilha de waypoints está vazia (todos concluídos)
                # 3. Nenhum comando pendente
                return (
                    current_state == DroneStateDescription.VOANDO_PRONTO
                    and self.drone_context.waypoint_stack.is_empty
                    and self.drone_context.pending_command is None
                )

            case "LAND":
                return current_state in [
                    DroneStateDescription.POUSADO_ARMADO,
                    DroneStateDescription.POUSADO_DESARMADO
                ]
            
            case "RTL":
                return current_state == DroneStateDescription.POUSADO_DESARMADO
            
            case _:
                # Para comandos desconhecidos, considera completo imediatamente
                return True

    def _calculate_distance_to_target(self: 'DroneNode') -> float:
        """
        Calcula a distância restante até o alvo em metros.
        
        Returns:
            float: Distância em metros, ou 0.0 se não há alvo definido
        """
        if self.drone_context.target_local_position is None:
            return 0.0
        
        if self.drone_context.state_px4.local_position is None:
            return 0.0
        
        dx = self.drone_context.target_local_position[0] - self.drone_context.state_px4.local_position.x
        dy = self.drone_context.target_local_position[1] - self.drone_context.state_px4.local_position.y
        dz = self.drone_context.target_local_position[2] - self.drone_context.state_px4.local_position.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def _calculate_progress_percent(self: 'DroneNode', command: str) -> float:
        """
        Calcula o percentual de progresso estimado para o comando.
        
        Args:
            command: Comando em execução
            
        Returns:
            float: Percentual de progresso (0.0 a 100.0)
        """
        # Para comandos de trajetória, usa a distância
        if command in ["GOTO", "RTL"]:
            if self.drone_context.initial_distance_to_target is not None and self.drone_context.initial_distance_to_target > 0:
                current_distance = self._calculate_distance_to_target()
                progress = 1.0 - (current_distance / self.drone_context.initial_distance_to_target)
                return max(0.0, min(100.0, progress * 100.0))
        
        # Para outros comandos, retorna 0 ou 100 baseado no estado
        return 0.0

