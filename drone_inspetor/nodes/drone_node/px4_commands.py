# =================================================================================================
# px4_commands.py
# =================================================================================================
# MIXIN: Comandos de baixo nível para o PX4 (alto-nível) do DroneNode
# =================================================================================================
# Métodos que traduzem comandos de alto nível (arm, takeoff, goto, land, rtl, stop, emergency_rtl)
# em mensagens VehicleCommand para o PX4 e/ou em alterações na TargetStack/lifecycle FSM.
# Implementado como Mixin herdado por DroneNode.
#
# Estratégia atual (versão inicial — vai evoluir):
#   - ARM/TAKEOFF: comandos diretos ao PX4 (sem usar a TargetStack — TAKEOFF é vertical puro,
#                  observado pelo lifecycle DECOLANDO).
#   - GOTO:        empilha um Target MISSAO na TargetStack; DeslocamentoFSM cuida da execução.
#   - LAND / RTL:  delegados ao PX4 nativo (VEHICLE_CMD_NAV_LAND / RETURN_TO_LAUNCH). O autopilot
#                  conduz o pouso; nossa DroneFSM detecta `is_landed` e volta para POUSADO_*.
#   - EMERGENCY:   alias para RTL nativo do PX4 — chamado pelo estado EMERGENCIA.
#   - STOP:        cancela manobras (limpa a TargetStack), drone fica hover via DeslocamentoFSM.
# =================================================================================================

import math
from typing import TYPE_CHECKING

from px4_msgs.msg import VehicleCommand, VehicleCommandAck

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.ros_interfaces import Topics


class DronePX4CommandsMixin:
    """Mixin com comandos de alto nível traduzidos para mensagens PX4."""

    # =============================================================================================
    # Setup de estado interno do mixin
    # =============================================================================================

    def init_px4_commands_state(self: 'DroneNode') -> None:
        """
        Inicializa o estado interno da camada de comandos PX4.
        Deve ser chamado pelo __init__ do DroneNode.

        Atributo `_px4_commands_awaiting_ack_log`: ids de comandos enviados ao PX4 que
        estamos esperando ACK. Filtra o callback de ACK p/ logar apenas o que enviamos
        (o PX4 publica ACKs de TODAS as fontes — QGroundControl, MAVSDK etc.).
        """
        self._px4_commands_awaiting_ack_log: set[int] = set()

    # =============================================================================================
    # Callback de ACK do PX4
    # =============================================================================================

    def px4_command_ack_callback(self: 'DroneNode', msg) -> None:
        """Loga ACKs de comandos enviados pelo drone_node (filtra os de outras fontes)."""
        self.state_px4.last_command_ack = msg
        if msg.command not in self._px4_commands_awaiting_ack_log:
            return
        self._px4_commands_awaiting_ack_log.discard(msg.command)

        result_text = (
            "ACEITO" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else "REJEITADO"
        )
        descricao = self._get_command_description(msg.command)
        self.get_logger().info(
            LogPrefix.px4_rx(f"ACK {descricao} ({msg.command}): {result_text} (cód {msg.result})")
        )

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

    # =============================================================================================
    # Publish helper
    # =============================================================================================

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
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.px4_vehicle_command_pub.publish(msg)
        self._px4_commands_awaiting_ack_log.add(command)

    # =============================================================================================
    # ARM
    # =============================================================================================

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
        self.get_logger().warn(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] Solicitando DESARMAR motores..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )

    # =============================================================================================
    # TAKEOFF
    # =============================================================================================

    def takeoff(self: 'DroneNode', altitude: 'float | None' = None) -> None:
        """
        Inicia a manobra de decolagem.

        Apenas marca o pending_command e a takeoff_altitude no contexto — o estado
        POUSADO_ARMADO da DroneFSM consome o pending_command, transita para DECOLANDO,
        e a `vertical_takeoff_trajectory` faz o drone subir até a altitude alvo.

        Args:
            altitude: Altitude alvo em metros. Se None, usa `context.takeoff_altitude`
                      (default 2.5m).
        """
        ctx = self.drone_fsm_context
        if altitude is not None and not math.isnan(altitude):
            ctx.takeoff_altitude = float(altitude)
        ctx.pending_command = "TAKEOFF"
        self.get_logger().info(f"TAKEOFF solicitado: altitude alvo {ctx.takeoff_altitude:.2f}m.")

    # =============================================================================================
    # GOTO (sem foco e com foco)
    # =============================================================================================

    def goto(
        self: 'DroneNode',
        lat: 'float | None' = None,
        lon: 'float | None' = None,
        alt: 'float | None' = None,
        yaw: 'float | None' = None,
        use_focus: bool = False,
        focus_lat: 'float | None' = None,
        focus_lon: 'float | None' = None,
    ) -> None:
        """
        Empilha um target de missão na TargetStack — DeslocamentoFSM cuida de executá-lo.

        Comportamento de NaN/None nos parâmetros: mantém o valor atual do drone para
        aquele eixo (lat/lon/alt) ou ignora (yaw final).

        Args:
            lat, lon, alt: Coordenadas GPS do destino. None ou NaN = manter atual.
            yaw:           Yaw final em graus (-180 a 180). None = não rotacionar no fim.
            use_focus:     Se True, mantém o yaw apontando para focus_lat/focus_lon durante
                           todo o trajeto (em vez de seguir a direção do movimento).
            focus_lat,
            focus_lon:     Coordenadas do ponto de foco. Obrigatórios se use_focus=True.
        """
        def _missing(v):
            return v is None or (isinstance(v, float) and math.isnan(v))

        if use_focus:
            if _missing(lat) or _missing(lon) or _missing(alt):
                self.get_logger().error(
                    "GOTO com foco: lat/lon/alt obrigatórios."
                )
                return
            if _missing(focus_lat) or _missing(focus_lon):
                self.get_logger().error(
                    "GOTO com foco: focus_lat/focus_lon obrigatórios."
                )
                return

        dctx = self.drone_fsm_context
        sctx = self.deslocamento_fsm_context
        px4 = self.state_px4

        # Resolve local_position do target a partir de lat/lon/alt + posição atual como fallback.
        if _missing(lat) or _missing(lon):
            target_x = px4.local_position.x
            target_y = px4.local_position.y
            lat_used = px4.global_position.lat if px4.global_position else None
            lon_used = px4.global_position.lon if px4.global_position else None
        else:
            converted = self.global_to_local_position(lat, lon, px4.global_position.alt)
            if converted is None:
                self.get_logger().error("GOTO: falha ao converter coordenadas globais.")
                return
            target_x, target_y = converted[0], converted[1]
            lat_used, lon_used = lat, lon

        if _missing(alt):
            target_z = px4.local_position.z
            alt_used = px4.global_position.alt if px4.global_position else None
        else:
            # NED: altitude positiva (acima do solo) → z negativo no frame local.
            alt_diff = alt - px4.home_global_alt
            target_z = -alt_diff
            alt_used = alt

        # Yaw final (rad/deg)
        final_yaw_deg = final_yaw_rad = final_yaw_norm = None
        if not _missing(yaw) and not use_focus:
            final_yaw_norm = float(yaw)
            final_yaw_deg = final_yaw_norm if final_yaw_norm >= 0 else final_yaw_norm + 360
            final_yaw_rad = math.radians(final_yaw_norm)

        # Foco: conversão para local (se aplicável).
        focus_local = None
        if use_focus:
            focus_z_ref = -px4.local_position.z  # altitude atual (positiva) p/ alimentar a conversão
            focus_local = self.global_to_local_position(focus_lat, focus_lon, focus_z_ref)
            if focus_local is None:
                self.get_logger().error("GOTO com foco: falha ao converter coordenadas de foco.")
                return

        # Empilha o target. A DeslocamentoFSM, em PLANANDO, detectará o novo topo no próximo tick.
        sctx.target_stack.push_missao(
            local_pos=[target_x, target_y, target_z],
            latitude=lat_used,
            longitude=lon_used,
            altitude=alt_used,
            final_yaw_deg=final_yaw_deg,
            final_yaw_deg_normalized=final_yaw_norm,
            final_yaw_rad=final_yaw_rad,
            focus_local_position=focus_local,
            focus_latitude=focus_lat if use_focus else None,
            focus_longitude=focus_lon if use_focus else None,
        )
        dctx.pending_command = None  # processado de imediato — não há lifecycle a transicionar.
        dctx.pending_use_focus = use_focus

        modo = "com foco" if use_focus else "sem foco"
        self.get_logger().info(
            f"GOTO ({modo}) solicitado: target_local=[{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]"
        )

    # =============================================================================================
    # LAND
    # =============================================================================================

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
        self.deslocamento_fsm_context.target_stack.clear()
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.drone_fsm_context.pending_command = "LAND"

    # =============================================================================================
    # RTL
    # =============================================================================================

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
        self.deslocamento_fsm_context.target_stack.clear()
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.drone_fsm_context.pending_command = "RTL"

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
        self.deslocamento_fsm_context.target_stack.clear()
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    # =============================================================================================
    # STOP
    # =============================================================================================

    def stop(self: 'DroneNode') -> None:
        """
        Cancela a manobra atual: limpa a TargetStack e captura a posição atual como hover.

        O lifecycle permanece em EM_VOO; a DeslocamentoFSM detecta target_stack vazio
        no próximo tick e transita para PLANANDO (hover na last_static_position).
        """
        self.get_logger().info("STOP solicitado — cancelando manobras e indo a hover.")
        self.deslocamento_fsm_context.reset()

    # =============================================================================================
    # Modos PX4 (utilitários)
    # =============================================================================================

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
