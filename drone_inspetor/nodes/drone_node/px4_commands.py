# px4_commands.py
# =================================================================================================
# MIXIN: COMANDOS PARA O PX4
# =================================================================================================
# Agrupa os métodos que traduzem comandos de alto nível (arm, takeoff, goto, land, rtl, stop)
# em mensagens VehicleCommand para o PX4. Implementado como Mixin herdado por DroneNode.
# =================================================================================================

import math
from typing import TYPE_CHECKING

from px4_msgs.msg import VehicleCommand, VehicleCommandAck

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor.common.enums import DroneStateDescription
from drone_inspetor.common.log_colors import LogPrefix
from drone_inspetor.ros_interfaces import Topics


class DronePX4CommandsMixin:
    """Mixin com comandos de baixo nível para o PX4."""

    # ==================================================================
    # SEÇÃO 4: AÇÕES - Funções Principais (comandos para o PX4)
    # ==================================================================

    def init_px4_commands_state(self: 'DroneNode'):
        """
        Inicializa o estado interno da camada de comandos PX4.
        Deve ser chamado pelo __init__ da classe host (DroneNode).

        `_px4_commands_awaiting_ack_log`: conjunto de IDs de comandos PX4
        enviados via publish_vehicle_command(). Usado para filtrar o
        callback de ACK e logar apenas os comandos que nós enviamos
        (o PX4 publica ACKs de TODAS as fontes, inclusive QGroundControl).
        """
        self._px4_commands_awaiting_ack_log = set()

    def _get_command_description(self: 'DroneNode', command_id: int) -> str:
        """
        Retorna a descrição legível de um comando PX4 baseado no seu ID.

        Args:
            command_id: ID numérico do comando

        Returns:
            str: Descrição do comando ou "Comando_{ID}" se não encontrado
        """
        # Mapeamento dos comandos mais comuns do PX4
        command_descriptions = {
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARMAR/DESARMAR",
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "DEFINIR_MODO",
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF: "DECOLAGEM",
            VehicleCommand.VEHICLE_CMD_NAV_LAND: "POUSO",
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: "RETORNAR_AO_LANÇAMENTO",
            VehicleCommand.VEHICLE_CMD_DO_SET_HOME: "DEFINIR_HOME",
            VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER: "DEFINIR_PARÂMETRO",
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION: "TERMINAÇÃO_DE_VOO",
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED: "ALTERAR_VELOCIDADE",
            VehicleCommand.VEHICLE_CMD_DO_SET_ROI: "DEFINIR_ROI",
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL: "CONTROLE_DE_MOUNT",
            # VehicleCommand.VEHICLE_CMD_DO_SET_CAMERA_TRIGGER: "DISPARAR_CÂMERA",
            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION: "TRANSIÇÃO_VTOL",
            VehicleCommand.VEHICLE_CMD_NAV_VTOL_TAKEOFF: "DECOLAGEM_VTOL",
            VehicleCommand.VEHICLE_CMD_NAV_VTOL_LAND: "POUSO_VTOL",
            # VehicleCommand.VEHICLE_CMD_NAV_GUIDED_ENABLE: "HABILITAR_GUIDADO",
            # VehicleCommand.VEHICLE_CMD_NAV_ORBIT: "ÓRBITA",
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_CALIBRATION: "CALIBRAÇÃO_PREFLIGHT",
            VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: "REINICIAR_DESLIGAR",
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION: "REPOSICIONAR",
            VehicleCommand.VEHICLE_CMD_DO_PAUSE_CONTINUE: "PAUSAR_CONTINUAR",
            VehicleCommand.VEHICLE_CMD_DO_ORBIT: "ÓRBITA",
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST: "TESTE_DE_MOTOR",
        }
        return command_descriptions.get(command_id, f"Comando_{command_id}")

    def px4_command_ack_callback(self: 'DroneNode', msg):
        """
        Processa confirmações de comandos enviados ao PX4.
        Apenas loga ACKs para comandos do PX4 enviados pelo drone_node (rastreados em _px4_commands_awaiting_ack_log).
        """
        self.drone_context.state_px4.last_command_ack = msg

        # Verifica se o comando foi enviado pelo drone_node
        if msg.command not in self._px4_commands_awaiting_ack_log:
            return  # Ignora ACKs de comandos não enviados por nós

        # Remove o comando do conjunto de pendentes
        self._px4_commands_awaiting_ack_log.discard(msg.command)

        result_text = "ACEITO" if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else "REJEITADO"
        command_description = self._get_command_description(msg.command)
        self.get_logger().info(LogPrefix.px4_rx(f"ACK Comando {command_description} ({msg.command}): {result_text} (código: {msg.result})"))

    def publish_vehicle_command(self: 'DroneNode', command, **params):
        """
        Função central para construir e publicar uma mensagem VehicleCommand para o PX4.
        
        Args:
            command: ID do comando (constantes VehicleCommand.VEHICLE_CMD_*)
            **params: Parâmetros do comando (param1-param7)
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
        msg.target_system = 1      # ID do sistema alvo (1 = autopilot)
        msg.target_component = 1   # ID do componente alvo (1 = autopilot)
        msg.source_system = 255    # ID do sistema fonte (255 = companion computer)
        msg.source_component = 1   # ID do componente fonte
        msg.from_external = True   # Indica que o comando vem de fonte externa
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # Timestamp em microssegundos
        
        self.px4_vehicle_command_pub.publish(msg)
        
        # Adiciona ao conjunto de comandos pendentes para rastrear ACK
        self._px4_commands_awaiting_ack_log.add(command)
        
        self.get_logger().debug(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                f"Comando (ID: {command}) publicado com parâmetros: {params}"
            )
        )

    def arm(self: 'DroneNode'):
        """
        Envia o comando para armar os motores do drone.
        Comando: VEHICLE_CMD_COMPONENT_ARM_DISARM com param1=1.0
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "Enviando comando para ARMAR motores..."
            )
        )
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self: 'DroneNode'):
        """
        Envia o comando para desarmar os motores do drone.
        Comando: VEHICLE_CMD_COMPONENT_ARM_DISARM com param1=0.0
        Usado em situações de emergência quando o drone já está pousado.
        """
        self.get_logger().warn(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "Enviando comando para DESARMAR motores..."
            )
        )
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def takeoff(self: 'DroneNode', altitude=None):
        """
        Inicia decolagem até a altitude especificada.
        O drone deve estar armado e no chão.
        
        Args:
            altitude: Altitude alvo em metros. Se None, usa takeoff_altitude padrão.
        """
        if altitude is None:
            altitude = self.drone_context.takeoff_altitude
        
        # Configura trajetória de decolagem (vertical)
        # Armazena posição atual como origem
        current_x = self.drone_context.state_px4.local_position.x
        current_y = self.drone_context.state_px4.local_position.y
        current_z = self.drone_context.state_px4.local_position.z
        
        self.drone_context.origin_local_position = [current_x, current_y, current_z]
        
        # Calcula altitude alvo relativa à posição HOME (não à posição atual)
        # A altitude alvo é em relação ao HOME, onde Z=0 no frame NED
        # Se home_local_position existe, usa como referência; senão usa 0
        home_z = self.drone_context.state_px4.home_local_position[2] if self.drone_context.state_px4.home_local_position else 0.0
        target_z = home_z - altitude  # Z negativo = para cima no frame NED
        
        self.drone_context.target_local_position = [current_x, current_y, target_z]
        
        # Mantém yaw do home durante decolagem (normalized = -180/180, deg = 0-360)
        self.drone_context.target_final_yaw_deg = self.drone_context.state_px4.home_yaw_deg
        self.drone_context.target_final_yaw_deg_normalized = self.drone_context.state_px4.home_yaw_deg_normalized
        self.drone_context.target_final_yaw_rad = self.drone_context.state_px4.home_yaw_rad
        self.drone_context.target_direction_yaw_deg = self.drone_context.state_px4.home_yaw_deg
        self.drone_context.target_direction_yaw_deg_normalized = self.drone_context.state_px4.home_yaw_deg_normalized
        self.drone_context.target_direction_yaw_rad = self.drone_context.state_px4.home_yaw_rad
        
        # Define comando pendente (será processado pelo estado POUSADO_ARMADO)
        self.drone_context.pending_command = "TAKEOFF"
        
        current_alt = -current_z  # Converte para altitude positiva
        self.get_logger().info(
            f"TAKEOFF solicitado: altitude atual={current_alt:.2f}m, "
            f"altitude alvo={altitude}m (relativa ao HOME)"
        )



    def goto(self: 'DroneNode', lat=None, lon=None, alt=None, yaw=None,
             use_focus=False, focus_lat=None, focus_lon=None):
        """
        Move o drone para uma posição específica usando coordenadas globais.
        Converte as coordenadas globais para locais e armazena para uso nos cálculos de trajetória.
        O drone deve estar armado e em modo Offboard para que isso funcione.

        Modo padrão (use_focus=False):
        Sequência de execução (completa):
        1. Rotaciona o drone para apontar na direção do destino
        2. Move o drone até a posição alvo
        3. Rotaciona o drone para o yaw alvo (se especificado)

        Casos especiais com NaN (modo padrão):
        - lat/lon NaN, alt válido: pula rotação inicial, apenas muda altitude
        - lat/lon/alt NaN, yaw válido: vai direto para rotação final (apenas gira)
        - Todos NaN: comando ignorado

        Modo com foco (use_focus=True):
        Sequência de execução:
        1. Rotaciona para apontar ao foco (VOANDO_GIRANDO_COM_FOCO)
        2. Move até o destino mantendo yaw no foco (VOANDO_A_CAMINHO_COM_FOCO)
        3. Aguarda estabilização no destino
        Neste modo, o parâmetro `yaw` é ignorado e lat/lon/alt + focus_lat/focus_lon
        são obrigatórios.

        Args:
            lat: Latitude alvo (graus) - pode ser NaN para manter posição atual
            lon: Longitude alvo (graus) - pode ser NaN para manter posição atual
            alt: Altitude alvo GLOBAL em metros (MSL) - pode ser NaN para manter altitude atual
            yaw: Yaw alvo final (graus, -180 a 180) - opcional, None/NaN mantém o yaw atual
                 (ignorado quando use_focus=True)
            use_focus: Se True, mantém o yaw apontando para focus_lat/focus_lon durante o trajeto
            focus_lat: Latitude do ponto de foco (graus) - obrigatório quando use_focus=True
            focus_lon: Longitude do ponto de foco (graus) - obrigatório quando use_focus=True
        """
        if use_focus:
            self._goto_with_focus(lat=lat, lon=lon, alt=alt,
                                  focus_lat=focus_lat, focus_lon=focus_lon)
            return

        # Verifica se lat/lon são NaN
        lat_is_nan = lat is None or (isinstance(lat, float) and math.isnan(lat))
        lon_is_nan = lon is None or (isinstance(lon, float) and math.isnan(lon))
        alt_is_nan = alt is None or (isinstance(alt, float) and math.isnan(alt))
        yaw_is_nan = yaw is None or (isinstance(yaw, float) and math.isnan(yaw))

        # Se todos são NaN, não há nada a fazer
        if lat_is_nan and lon_is_nan and alt_is_nan and yaw_is_nan:
            self.get_logger().error("Não é possível executar GOTO: Todos os parâmetros são NaN/None.")
            return
        
        # Armazena coordenadas globais de origem
        self.drone_context.origin_latitude = self.drone_context.state_px4.global_position.lat
        self.drone_context.origin_longitude = self.drone_context.state_px4.global_position.lon
        self.drone_context.origin_altitude = self.drone_context.state_px4.global_position.alt
        
        # Armazena posição local de origem
        self.drone_context.origin_local_position = [
            self.drone_context.state_px4.local_position.x, 
            self.drone_context.state_px4.local_position.y, 
            self.drone_context.state_px4.local_position.z
        ]
        
        # Define target_latitude/longitude/altitude (usando NaN para indicar "manter atual")
        self.drone_context.target_latitude = lat if not lat_is_nan else float('nan')
        self.drone_context.target_longitude = lon if not lon_is_nan else float('nan')
        self.drone_context.target_altitude = alt if not alt_is_nan else float('nan')
        
        # Calcula target_local_position baseado nos valores disponíveis
        if lat_is_nan or lon_is_nan:
            # lat/lon NaN: mantém posição X/Y atual
            target_local_x = self.drone_context.state_px4.local_position.x
            target_local_y = self.drone_context.state_px4.local_position.y
        else:
            # Converte lat/lon para local
            target_local = self.global_to_local_position(lat, lon, self.drone_context.state_px4.global_position.alt)
            if target_local is None:
                self.get_logger().error("Não é possível executar GOTO: Erro ao converter coordenadas globais para locais.")
                return
            target_local_x = target_local[0]
            target_local_y = target_local[1]
        
        if alt_is_nan:
            # alt NaN: mantém altitude atual
            target_local_z = self.drone_context.state_px4.local_position.z
        else:
            # Converte altitude GLOBAL para local (z é negativo em NED)
            # Calcula z baseado na diferença entre altitude global alvo e altitude global do home
            alt_diff = alt - self.drone_context.state_px4.home_global_alt
            target_local_z = -alt_diff
        
        self.drone_context.target_local_position = [target_local_x, target_local_y, target_local_z]
        
        # Calcula distância inicial para cálculo de progresso
        dx = target_local_x - self.drone_context.state_px4.local_position.x
        dy = target_local_y - self.drone_context.state_px4.local_position.y
        dz = target_local_z - self.drone_context.state_px4.local_position.z
        self.drone_context.initial_distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
        # Armazena yaw alvo (pode ser None/NaN)
        if not yaw_is_nan:
            # yaw é recebido na faixa -180 a 180 (normalized)
            yaw_normalized = yaw
            yaw_0_360 = yaw if yaw >= 0 else yaw + 360
            self.drone_context.target_final_yaw_deg = yaw_0_360
            self.drone_context.target_final_yaw_deg_normalized = yaw_normalized
            self.drone_context.target_final_yaw_rad = math.radians(yaw_normalized)
        else:
            self.drone_context.target_final_yaw_deg = None
            self.drone_context.target_final_yaw_deg_normalized = None
            self.drone_context.target_final_yaw_rad = None
        
        # Calcula o yaw de direção apenas se houver movimento horizontal
        if not (lat_is_nan or lon_is_nan):
            dx = target_local_x - self.drone_context.state_px4.local_position.x
            dy = target_local_y - self.drone_context.state_px4.local_position.y
            target_direction_yaw = math.degrees(math.atan2(dy, dx))
            # Normaliza para -180 a 180
            if target_direction_yaw > 180:
                target_direction_yaw -= 360
            elif target_direction_yaw < -180:
                target_direction_yaw += 360
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_context.target_direction_yaw_deg_normalized = target_direction_yaw
            target_direction_yaw_0_360 = target_direction_yaw if target_direction_yaw >= 0 else target_direction_yaw + 360
            self.drone_context.target_direction_yaw_deg = target_direction_yaw_0_360
            self.drone_context.target_direction_yaw_rad = math.radians(target_direction_yaw)
        else:
            # Sem movimento horizontal, não precisa de yaw de direção
            self.drone_context.target_direction_yaw_deg = None
            self.drone_context.target_direction_yaw_deg_normalized = None
            self.drone_context.target_direction_yaw_rad = None
        
        # Empilha waypoint de missão e define comando pendente
        self.drone_context.waypoint_stack.push_mission(
            local_pos=[target_local_x, target_local_y, target_local_z],
            latitude=lat if not lat_is_nan else None,
            longitude=lon if not lon_is_nan else None,
            altitude=alt if not alt_is_nan else None,
            direction_yaw_deg=self.drone_context.target_direction_yaw_deg,
            direction_yaw_deg_normalized=self.drone_context.target_direction_yaw_deg_normalized,
            direction_yaw_rad=self.drone_context.target_direction_yaw_rad,
            final_yaw_deg=self.drone_context.target_final_yaw_deg,
            final_yaw_deg_normalized=self.drone_context.target_final_yaw_deg_normalized,
            final_yaw_rad=self.drone_context.target_final_yaw_rad,
        )
        self.drone_context.pending_command = "GOTO"
        self.drone_context.pending_use_focus = False

        # Log informativo
        lat_str = f"{lat:.6f}" if not lat_is_nan else "NaN"
        lon_str = f"{lon:.6f}" if not lon_is_nan else "NaN"
        alt_str = f"{alt:.2f}" if not alt_is_nan else "NaN"
        yaw_str = f"{yaw:.1f}°" if not yaw_is_nan else "NaN"
        
        self.get_logger().info(
            f"GOTO solicitado: lat={lat_str}, lon={lon_str}, alt={alt_str}, yaw={yaw_str}"
        )
        self.get_logger().info(
            f"GOTO: target_local=[{target_local_x:.2f}, {target_local_y:.2f}, {target_local_z:.2f}], "
            f"direction_yaw={self.drone_context.target_direction_yaw_deg if self.drone_context.target_direction_yaw_deg is not None else 'N/A'}"
        )


    def _goto_with_focus(self: 'DroneNode', lat=None, lon=None, alt=None, focus_lat=None, focus_lon=None):
        """
        Implementação interna do GOTO com foco — chamada por goto() quando use_focus=True.

        Move o drone para uma posição específica mantendo o yaw apontando para um ponto de foco.
        Durante toda a trajetória, o drone mantém o yaw direcionado para o ponto de foco.

        Sequência de execução:
        1. Rotaciona para apontar ao foco (VOANDO_GIRANDO_COM_FOCO)
        2. Move até o destino mantendo yaw no foco (VOANDO_A_CAMINHO_COM_FOCO)
        3. Aguarda estabilização no destino

        Args:
            lat: Latitude alvo (graus) - obrigatório
            lon: Longitude alvo (graus) - obrigatório
            alt: Altitude alvo (metros) - obrigatório
            focus_lat: Latitude do ponto de foco (graus) - obrigatório
            focus_lon: Longitude do ponto de foco (graus) - obrigatório
        """
        # Trata NaN como ausente
        def _is_missing(v):
            return v is None or (isinstance(v, float) and math.isnan(v))

        if _is_missing(lat) or _is_missing(lon) or _is_missing(alt):
            self.get_logger().error(
                "Não é possível executar GOTO com use_focus=True: "
                "Coordenadas de destino incompletas (lat, lon, alt são obrigatórias)."
            )
            return

        if _is_missing(focus_lat) or _is_missing(focus_lon):
            self.get_logger().error(
                "Não é possível executar GOTO com use_focus=True: "
                "Coordenadas de foco incompletas (focus_lat, focus_lon são obrigatórias)."
            )
            return

        # Armazena coordenadas globais para referência
        self.drone_context.origin_latitude = self.drone_context.state_px4.global_position.lat
        self.drone_context.origin_longitude = self.drone_context.state_px4.global_position.lon
        self.drone_context.origin_altitude = self.drone_context.state_px4.global_position.alt
        self.drone_context.target_latitude = lat
        self.drone_context.target_longitude = lon
        self.drone_context.target_altitude = alt
        self.drone_context.focus_latitude = focus_lat
        self.drone_context.focus_longitude = focus_lon

        # Converte coordenadas globais para locais
        self.drone_context.origin_local_position = [
            self.drone_context.state_px4.local_position.x,
            self.drone_context.state_px4.local_position.y,
            self.drone_context.state_px4.local_position.z
        ]

        target_local = self.global_to_local_position(lat, lon, alt)
        if target_local is None:
            self.get_logger().error(
                "Não é possível executar GOTO com use_focus=True: "
                "Erro ao converter coordenadas de destino para locais."
            )
            return

        self.drone_context.target_local_position = target_local

        # Converte ponto de foco para local (usando altitude atual para simplificar)
        focus_local = self.global_to_local_position(focus_lat, focus_lon, -self.drone_context.state_px4.local_position.z)
        if focus_local is None:
            self.get_logger().error(
                "Não é possível executar GOTO com use_focus=True: "
                "Erro ao converter coordenadas de foco para locais."
            )
            return

        self.drone_context.focus_local_position = focus_local

        # Empilha waypoint de missão com foco e define comando pendente
        self.drone_context.waypoint_stack.push_mission(
            local_pos=target_local,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            focus_local_position=focus_local,
            focus_latitude=focus_lat,
            focus_longitude=focus_lon,
        )
        self.drone_context.pending_command = "GOTO"
        self.drone_context.pending_use_focus = True

        self.get_logger().info(
            f"GOTO (use_focus=True) solicitado: destino=[{lat:.6f}, {lon:.6f}, {alt:.2f}], "
            f"foco=[{focus_lat:.6f}, {focus_lon:.6f}]"
        )
        self.get_logger().info(
            f"GOTO (use_focus=True): destino local=[{target_local[0]:.2f}, {target_local[1]:.2f}, {target_local[2]:.2f}], "
            f"foco local=[{focus_local[0]:.2f}, {focus_local[1]:.2f}]"
        )


    def land(self: 'DroneNode'):
        """
        Inicia pouso usando o comando nativo do PX4 (VEHICLE_CMD_NAV_LAND).
        O PX4 muda temporariamente para modo AUTO_LAND e executa o pouso de forma autônoma.
        O drone desce na posição atual com taxa de descida controlada pelo PX4.
        Após pousar, se setpoints Offboard estiverem sendo publicados, retorna ao modo Offboard.
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "Enviando comando LAND nativo do PX4 (pouso na posição atual)..."
            )
        )
        
        # Envia comando de pouso nativo - o PX4 cuida de tudo
        # O drone pousa na posição atual, não vai para home
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        # Define comando pendente para a máquina de estados
        self.drone_context.pending_command = "LAND"
        
        self.get_logger().info(
            f"LAND nativo solicitado: posição atual X={self.drone_context.state_px4.local_position.x:.2f}m, "
            f"Y={self.drone_context.state_px4.local_position.y:.2f}m, "
            f"Alt={-self.drone_context.state_px4.local_position.z:.2f}m"
        )



    def rtl(self: 'DroneNode'):
        """
        Inicia retorno à base via Offboard com sequência:
        1. RETORNANDO_GIRANDO_INICIO: Gira para apontar na direção do home
        2. RETORNANDO_A_CAMINHO: Sobe até a altitude RTL enquanto vai para o home (X=0, Y=0)
        3. RETORNANDO_GIRANDO_FIM: Gira para o yaw final do home (home_yaw)
        4. POUSANDO: Pousa usando comando nativo do PX4
        
        Nota: O PX4 desarma automaticamente após pousar (parâmetro COM_DISARM_LAND).
        """
        self.get_logger().info(
            f"Iniciando RTL via Offboard: girar → subir {self.drone_context.rtl_altitude}m → ir para home → girar para yaw home → pousar..."
        )
        
        # Configura origem da trajetória
        current_x = self.drone_context.state_px4.local_position.x
        current_y = self.drone_context.state_px4.local_position.y
        current_z = self.drone_context.state_px4.local_position.z
        
        self.drone_context.origin_local_position = [current_x, current_y, current_z]
        
        # Alvo: posição HOME na altitude RTL
        # Usa home_local_position se disponível, senão assume (0, 0)
        if self.drone_context.state_px4.home_local_position is not None:
            home_x = self.drone_context.state_px4.home_local_position[0]
            home_y = self.drone_context.state_px4.home_local_position[1]
            home_z = self.drone_context.state_px4.home_local_position[2]
        else:
            home_x = 0.0
            home_y = 0.0
            home_z = 0.0
            self.get_logger().warn("RTL: home_local_position não disponível, usando (0, 0, 0)")
        
        # Altitude RTL relativa ao HOME (Z negativo = para cima no frame NED)
        target_z = home_z - self.drone_context.rtl_altitude
        
        self.drone_context.target_local_position = [home_x, home_y, target_z]
        
        # Calcula yaw de direção para apontar para home
        dx = home_x - current_x
        dy = home_y - current_y
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            target_direction_yaw = math.degrees(math.atan2(dy, dx))
            # Normaliza para -180 a 180
            if target_direction_yaw > 180:
                target_direction_yaw -= 360
            elif target_direction_yaw < -180:
                target_direction_yaw += 360
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_context.target_direction_yaw_deg_normalized = target_direction_yaw
            target_direction_yaw_0_360 = target_direction_yaw if target_direction_yaw >= 0 else target_direction_yaw + 360
            self.drone_context.target_direction_yaw_deg = target_direction_yaw_0_360
            self.drone_context.target_direction_yaw_rad = math.radians(target_direction_yaw)
        else:
            # Já está no home, mantém yaw atual
            self.drone_context.target_direction_yaw_deg_normalized = self.drone_context.state_px4.current_yaw_deg_normalized
            current_yaw_0_360 = self.drone_context.state_px4.current_yaw_deg_normalized
            if current_yaw_0_360 < 0:
                current_yaw_0_360 += 360
            self.drone_context.target_direction_yaw_deg = current_yaw_0_360
            self.drone_context.target_direction_yaw_rad = self.drone_context.state_px4.current_yaw_rad
        
        # Yaw final será o yaw do home (normalized = -180/180, deg = 0-360)
        self.drone_context.target_final_yaw_deg = self.drone_context.state_px4.home_yaw_deg
        self.drone_context.target_final_yaw_deg_normalized = self.drone_context.state_px4.home_yaw_deg_normalized
        self.drone_context.target_final_yaw_rad = self.drone_context.state_px4.home_yaw_rad
        
        # Empilha waypoint do HOME e define comando pendente
        self.drone_context.waypoint_stack.push_mission(
            local_pos=[home_x, home_y, target_z],
            direction_yaw_deg=self.drone_context.target_direction_yaw_deg,
            direction_yaw_deg_normalized=self.drone_context.target_direction_yaw_deg_normalized,
            direction_yaw_rad=self.drone_context.target_direction_yaw_rad,
            final_yaw_deg=self.drone_context.target_final_yaw_deg,
            final_yaw_deg_normalized=self.drone_context.target_final_yaw_deg_normalized,
            final_yaw_rad=self.drone_context.target_final_yaw_rad,
        )
        self.drone_context.pending_command = "RTL"
        
        self.get_logger().info(
            f"RTL solicitado: origem=[{current_x:.2f}, {current_y:.2f}, {-current_z:.2f}m], "
            f"alvo=home [{home_x:.2f}, {home_y:.2f}, {self.drone_context.rtl_altitude}m], "
            f"yaw_direção={self.drone_context.target_direction_yaw_deg:.1f}°, home_yaw={self.drone_context.state_px4.home_yaw_deg:.1f}°"
        )


    def rtl_native(self: 'DroneNode'):
        """
        Envia o comando RTL NATIVO do PX4 (VEHICLE_CMD_NAV_RETURN_TO_LAUNCH).
        Diferente do rtl() acima (que executa o retorno via Offboard),
        este apenas delega o controle ao autopilot: o PX4 muda automaticamente
        para AUTO_RTL e assume retorno e pouso.
        Usado em situações de emergência (ex: bateria crítica).
        """
        self.get_logger().warn(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "EMERGÊNCIA - Enviando RTL NATIVO do PX4 (controle delegado ao autopilot)..."
            )
        )
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)


    def stop(self: 'DroneNode'):
        """
        Para o drone no ar durante uma missão de GOTO ou RTL.

        Ação:
        1. Reseta todas as variáveis de trajetória (limpa pilha de waypoints)
        2. Armazena a posição atual como última posição estática para hover estável
        3. Muda o estado para VOANDO_PRONTO para aguardar novo comando

        Este comando só pode ser recebido durante estados de movimento:
        - GOTO (sem foco): VOANDO_GIRANDO_INICIO, VOANDO_A_CAMINHO, VOANDO_GIRANDO_FIM
        - GOTO (com foco, use_focus=True): VOANDO_GIRANDO_COM_FOCO, VOANDO_A_CAMINHO_COM_FOCO
        - RTL: RETORNANDO_GIRANDO_INICIO, RETORNANDO_A_CAMINHO, RETORNANDO_GIRANDO_FIM
        """
        current_state = self.drone_context.state.name
        self.get_logger().info(f"STOP recebido durante estado {current_state} - parando drone...")
        
        # Reseta todas as variáveis de trajetória, limpa pilha e armazena posição para hover
        self.drone_context.reset_trajectory_vars()
        
        # Muda para estado VOANDO_PRONTO para aguardar novo comando
        self.drone_fsm.transition_to(DroneStateDescription.VOANDO_PRONTO)
        
        self.get_logger().info(
            f"STOP executado: drone parado em hover, aguardando novo comando. "
            f"Posição estática: [{self.drone_context.last_static_position[0]:.2f}, "
            f"{self.drone_context.last_static_position[1]:.2f}, {-self.drone_context.last_static_position[2]:.2f}m]"
        )


    # ==================================================================
    # SEÇÃO 6: CONTROLE DE MODO - Funções Auxiliares
    # ==================================================================

    def set_offboard_mode(self: 'DroneNode'):
        """
        Envia o comando para mudar o modo de voo para Offboard.
        Isso só funcionará se setpoints Offboard estiverem sendo publicados continuamente.
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "Enviando comando para mudar para o modo OFFBOARD..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0,  # Custom mode enabled
            param2=6.0   # Offboard mode
        )

    def set_position_mode(self: 'DroneNode'):
        """
        Envia o comando para mudar o modo de voo para Position (POSCTL).
        O drone irá parar no ar mantendo a posição atual.
        """
        self.get_logger().info(
            LogPrefix.px4_tx(
                f"[{Topics.PX4.VEHICLE_COMMAND.name}] "
                "Enviando comando para mudar para o modo POSITION..."
            )
        )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode enabled
            param2=3.0   # Position mode (POSCTL)
        )
