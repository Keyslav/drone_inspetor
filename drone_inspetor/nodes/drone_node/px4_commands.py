"""Tradução de comandos em destinos locais, referências de decolagem e ordens PX4."""

import math

from typing import TYPE_CHECKING

from px4_msgs.msg import VehicleCommand, VehicleCommandAck

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.ros_interfaces import Topics


class DronePX4CommandsMixin:
    """Mixin com comandos de alto nível traduzidos para mensagens PX4."""

    # Setup de estado interno do mixin

    def init_px4_commands_state(self: 'DroneNode') -> None:
        """
        Inicializa IDs de comandos aguardando ACK e a falha associada à operação.

        O ACK contém o ID do comando, não o UUID da action. Confirmações não
        substituem a observação da telemetria para concluir uma operação.
        """
        self._px4_commands_awaiting_ack_log: set[int] = set()
        self._px4_command_error = None

    # Callback de ACK do PX4

    def px4_command_ack_callback(self, msg) -> None:
        """Distingue ACK negativo de confirmação; o resultado observa telemetria."""
        with self._control_lock:
            self.state_px4.last_command_ack = msg
            if msg.command not in self._px4_commands_awaiting_ack_log:
                return
            if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
                return
            self._px4_commands_awaiting_ack_log.discard(msg.command)
            description = self._get_command_description(msg.command)
            if msg.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self._px4_command_error = f'PX4 rejeitou {description}: código {msg.result}'
                self.get_logger().error(self._px4_command_error)
            else:
                self.get_logger().info(f'PX4 aceitou {description}; aguardando estado físico')

    def _get_command_description(self: 'DroneNode', command_id: int) -> str:
        """Descrição legível de um comando PX4 (para logs)."""
        nomes = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARMAR/DESARMAR",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "DEFINIR_MODO",
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF: "DECOLAGEM",
            VehicleCommand.VEHICLE_CMD_NAV_LAND: "POUSO",
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: "RETORNAR_AO_LANÇAMENTO",
        }
        return nomes.get(command_id, f"Comando_{command_id}")

    # Publish helper

    def publish_vehicle_command(self: 'DroneNode', command: int, **params) -> None:
        """
        Constrói e publica uma VehicleCommand. Adiciona o id ao set de pendentes
        para logging do ACK correspondente.
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.param_px4_target_system_id
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self._px4_commands_awaiting_ack_log.add(command)
        self.px4_vehicle_command_pub.publish(msg)

    # ARM

    def arm_drone(self: 'DroneNode') -> None:
        """
        Envia comando para ARMAR os motores ao PX4. Chamado pelo estado POUSADO_DESARMADO
        quando um comando "ARM" é recebido via Action.
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Solicitando ARMAR motores..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def disarm_drone(self: 'DroneNode') -> None:
        """Envia comando para DESARMAR os motores ao PX4."""
        if not self.state_px4.is_landed:
            raise ValueError('DISARM é permitido somente quando o drone está pousado')
        self.get_logger().warn(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Solicitando DESARMAR motores..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    # TAKEOFF

    def takeoff(self: 'DroneNode', altitude: 'float | None' = None) -> None:
        """
        Inicia a manobra de decolagem.

        Apenas marca o pending_command e a takeoff_altitude no contexto — o estado
        POUSADO_ARMADO da DroneFSM consome o pending_command, transita para DECOLANDO,
        e `Trajectory.compute_vertical_takeoff` produz a referência até a altitude alvo.

        Args:
            altitude: Altitude alvo em metros. Se None, usa `context.takeoff_altitude`
                      (default 2.5m).
        """
        ctx = self.drone_fsm_context
        if altitude is not None:
            if not math.isfinite(altitude) or altitude <= 0:
                raise ValueError('Altitude de decolagem deve ser positiva e finita')
            ctx.takeoff_altitude = float(altitude)
        ctx.pending_command = "TAKEOFF"
        self.get_logger().info(f"TAKEOFF solicitado: altitude alvo {ctx.takeoff_altitude:.2f}m.")

    # GOTO (sem foco e com foco)

    def goto(self, lat=None, lon=None, alt=None, yaw=None, use_focus=False,
             focus_lat=None, focus_lon=None):
        """Prepara GPS absoluto→NED antes de alterar a pilha; NaN mantém cada eixo."""
        def provided(value):
            return value is not None and not math.isnan(value)

        def coordinate(value, name, bound=None):
            if value is None or not math.isfinite(value) or (
                bound is not None and abs(value) > bound
            ):
                raise ValueError(f'GOTO: {name} inválido')
            return value

        px4 = self.state_px4
        current = px4.local_position
        if current is None:
            raise ValueError('GOTO: posição local desconhecida')
        target_x, target_y, target_z = current.x, current.y, current.z
        lat_used = px4.global_position.lat if px4.global_position else None
        lon_used = px4.global_position.lon if px4.global_position else None
        alt_used = px4.global_position.alt if px4.global_position else None
        if provided(lat) or provided(lon):
            lat_used = coordinate(lat if provided(lat) else lat_used, 'latitude', 90)
            lon_used = coordinate(lon if provided(lon) else lon_used, 'longitude', 180)
            # A altura enviada aqui afeta só Z, descartado nessa conversão horizontal.
            target_x, target_y, _ = self.trajectory.global_to_local_position(
                lat_used, lon_used, coordinate(px4.home_global_alt, 'altitude HOME')
            )
        if provided(alt):
            alt_used = coordinate(alt, 'altitude')
            home = px4.home_local_position
            if home is None:
                raise ValueError('GOTO: HOME local desconhecido')
            from drone_inspetor.common.coordinates import amsl_to_local_down
            target_z = amsl_to_local_down(
                alt_used, coordinate(px4.home_global_alt, 'altitude HOME'), home[2])
        if not all(math.isfinite(value) for value in (target_x, target_y, target_z)):
            raise ValueError('GOTO: destino NED não finito')

        final_yaw_norm = None
        if provided(yaw) and not use_focus:
            final_yaw_norm = (coordinate(yaw, 'yaw') + 180.) % 360. - 180.
        focus_local = None
        if use_focus:
            focus_lat = coordinate(focus_lat, 'latitude de foco', 90)
            focus_lon = coordinate(focus_lon, 'longitude de foco', 180)
            focus_local = self.trajectory.global_to_local_position(
                focus_lat, focus_lon, coordinate(px4.home_global_alt, 'altitude HOME')
            )
        self.deslocamento_fsm_context.target_stack.push_missao(
            local_pos=[target_x, target_y, target_z],
            latitude=lat_used, longitude=lon_used, altitude=alt_used,
            final_yaw_deg=None if final_yaw_norm is None else final_yaw_norm % 360.,
            final_yaw_deg_normalized=final_yaw_norm,
            final_yaw_rad=None if final_yaw_norm is None else math.radians(final_yaw_norm),
            focus_local_position=focus_local,
            focus_latitude=focus_lat if use_focus else None,
            focus_longitude=focus_lon if use_focus else None,
        )
        self.drone_fsm_context.pending_command = None
        self.drone_fsm_context.pending_use_focus = use_focus
        self.get_logger().info(f'GOTO preparado: NED {[target_x, target_y, target_z]}')

    # LAND

    def land(self: 'DroneNode') -> None:
        """
        Comando de pouso: delegado ao PX4 nativo (AUTO_LAND).

        O PX4 conduz a descida automaticamente. Nossa DroneFSM permanece em EM_VOO
        observando is_landed; ao detectar pouso, transita para POUSADO_ARMADO (e depois
        POUSADO_DESARMADO quando o PX4 auto-desarmar).
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Solicitando POUSO nativo (AUTO_LAND)..."
            )
        )
        # Cancela qualquer manobra Offboard ativa antes de delegar ao autopilot.
        self._handover_native('LAND', VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # RTL

    def rtl(self: 'DroneNode') -> None:
        """
        Comando de retorno à base: delegado ao PX4 nativo (AUTO_RTL).

        O PX4 sobe à altitude RTL configurada nele, volta para o HOME e pousa.
        DroneFSM permanece em EM_VOO até o pouso e desarmamento finalizarem.
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Solicitando RTL nativo (AUTO_RTL)..."
            )
        )
        # Cancela manobras Offboard antes de delegar ao autopilot.
        self._handover_native('RTL', VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def emergency_rtl(self: 'DroneNode') -> None:
        """
        Variante para emergência (chamada pelo estado EMERGENCIA). Envia o mesmo comando
        que `rtl()`, mas com log de alerta — semanticamente é falha, não missão.
        """
        self.get_logger().error(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] EMERGÊNCIA — RTL nativo do PX4."
            )
        )
        self._handover_native('RTL', VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    # STOP

    def _handover_native(self, command, px4_command):
        """Registra a intenção antes de publicar o comando de mudança de modo."""
        self.deslocamento_fsm_context.reset()
        self.deslocamento_fsm.reset_to_planando()
        context = self.drone_fsm_context
        context.pending_command = None
        context.native_command = command
        context.native_mode_observed = False
        self.publish_vehicle_command(px4_command)

    def stop(self):
        """Descarta destinos e mantém o perfil para a frenagem suave de compute_hover."""
        context = self.drone_fsm_context
        context.pending_command = None
        self.deslocamento_fsm_context.reset()
        self.deslocamento_fsm.reset_to_planando()
        if context.state == DS.DECOLANDO:
            if self.state_px4.is_landed:
                state = DS.POUSADO_ARMADO if self.state_px4.is_armed else DS.POUSADO_DESARMADO
            else:
                state = DS.EM_VOO
            self.drone_fsm.transition_to(state)

    # Modos PX4 (utilitários)

    def set_offboard_mode(self: 'DroneNode') -> None:
        """Pede ao PX4 para entrar em modo OFFBOARD. Requer setpoints Offboard sendo publicados."""
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Pedindo modo OFFBOARD..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def set_position_mode(self: 'DroneNode') -> None:
        """Pede ao PX4 para entrar em modo POSITION (POSCTL) — drone fica hover manual."""
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Pedindo modo POSITION..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0
        )
