# trajectory.py
# =================================================================================================
# MIXIN: CÁLCULO DE TRAJETÓRIA E CONVERSÃO GPS
# =================================================================================================
# Agrupa as rotinas de geração de setpoints, cálculo de próximo passo, ajuste de yaw
# incremental e conversão global↔local para o DroneNode. Implementado como Mixin para ser
# herdado por DroneNode, mantendo acesso direto a self.drone_context, self.get_logger(), etc.
# =================================================================================================

import math
from typing import TYPE_CHECKING

from px4_msgs.msg import TrajectorySetpoint

if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode

from drone_inspetor.common.enums import DroneStateDescription


class DroneTrajectoryMixin:
    """Mixin com funções de cálculo de trajetória e conversão GPS."""

    # ==================================================================
    # SEÇÃO 2: CÁLCULO DE TRAJETÓRIA - Funções Auxiliares
    # ==================================================================

    def create_static_position_setpoint(self: 'DroneNode'):
        """
        Cria um setpoint de trajetória para manter a posição estável (hover).
        
        Usa a última posição estática armazenada (last_static_position) ao invés da posição atual
        para evitar instabilidade causada por flutuações de GPS ou vento.
        A last_static_position é atualizada ao finalizar cada comando de movimento.
        
        IMPORTANTE: Nunca envia local_position diretamente para o trajectory. Se não houver
        last_static_position, armazena a posição atual primeiro e depois usa esse valor.
        
        Returns:
            TrajectorySetpoint: Mensagem com última posição estática conhecida e yaw
        """
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Se não houver posição estática, armazena a posição atual primeiro
        if self.drone_context.last_static_position is None:
            if self.drone_context.state_px4.local_position is not None:
                self.drone_context.last_static_position = [
                    self.drone_context.state_px4.local_position.x,
                    self.drone_context.state_px4.local_position.y,
                    self.drone_context.state_px4.local_position.z
                ]
                self.get_logger().debug(
                    f"Posição estática inicializada: "
                    f"[{self.drone_context.last_static_position[0]:.2f}, "
                    f"{self.drone_context.last_static_position[1]:.2f}, "
                    f"{self.drone_context.last_static_position[2]:.2f}]"
                )
            else:
                # Posição local indisponível - usa origem como fallback seguro
                self.get_logger().warn("Posição local indisponível para criar static setpoint")
                trajectory_msg.position[0] = 0.0
                trajectory_msg.position[1] = 0.0
                trajectory_msg.position[2] = 0.0
                trajectory_msg.yaw = float('nan')
                return trajectory_msg
        
        # Se não houver yaw estático, armazena o yaw atual primeiro
        if self.drone_context.last_static_yaw_rad is None:
            # Armazena ambas versões (normalized = -180/180, deg = 0-360)
            self.drone_context.last_static_yaw_deg_normalized = self.drone_context.state_px4.current_yaw_deg_normalized
            self.drone_context.last_static_yaw_deg = self.drone_context.state_px4.current_yaw_deg_normalized
            if self.drone_context.last_static_yaw_deg < 0:
                self.drone_context.last_static_yaw_deg += 360
            self.drone_context.last_static_yaw_rad = self.drone_context.state_px4.current_yaw_rad
            self.get_logger().debug(
                f"Yaw estático inicializado: {self.drone_context.last_static_yaw_deg_normalized:.2f}°"
            )
        
        # Usa a posição estática armazenada para manter estabilidade
        trajectory_msg.position[0] = self.drone_context.last_static_position[0]
        trajectory_msg.position[1] = self.drone_context.last_static_position[1]
        trajectory_msg.position[2] = self.drone_context.last_static_position[2]
        
        # Usa o yaw estático armazenado para manter orientação durante hover
        trajectory_msg.yaw = self.drone_context.last_static_yaw_rad
        
        return trajectory_msg

    def create_moving_trajectory_setpoint(self: 'DroneNode'):
        """
        Cria um setpoint de trajetória calculando o próximo ponto baseado no estado atual.

        Para estados de deslocamento (VOANDO_A_CAMINHO, VOANDO_A_CAMINHO_COM_FOCO,
        RETORNANDO_A_CAMINHO), delega ao TrajectoryProfile (perfil trapezoidal) que produz
        position+velocity+acceleration coerentes. Para os demais estados (rotações, decolagem,
        hover), envia apenas position com velocity/acceleration zerados (o drone fica parado
        no eixo XYZ enquanto gira ou aguarda).

        Returns:
            TrajectorySetpoint: Mensagem com setpoint completo
        """
        next_x, next_y, next_z, vel_xyz, acc_xyz, target_yaw_rad = (
            self.calculate_next_trajectory_by_state()
        )

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        trajectory_msg.position[0] = next_x
        trajectory_msg.position[1] = next_y
        trajectory_msg.position[2] = next_z

        trajectory_msg.velocity[0] = vel_xyz[0]
        trajectory_msg.velocity[1] = vel_xyz[1]
        trajectory_msg.velocity[2] = vel_xyz[2]

        trajectory_msg.acceleration[0] = acc_xyz[0]
        trajectory_msg.acceleration[1] = acc_xyz[1]
        trajectory_msg.acceleration[2] = acc_xyz[2]

        if target_yaw_rad is not None:
            trajectory_msg.yaw = target_yaw_rad
        else:
            trajectory_msg.yaw = float('nan')

        return trajectory_msg



    def calculate_next_trajectory_by_state(self: 'DroneNode'):
        """
        Calcula o próximo ponto da trajetória baseado na posição atual e no destino.
        Usa match/case para processar cada estado com verificações globais antes.

        Para estados de deslocamento (VOANDO_A_CAMINHO, VOANDO_A_CAMINHO_COM_FOCO,
        RETORNANDO_A_CAMINHO), delega a `_advance_profile()` que opera o TrajectoryProfile
        e devolve velocidade/aceleração coerentes com o perfil trapezoidal. Os demais estados
        retornam vel/acc zerados (drone parado nesses eixos enquanto gira/decola/pousa).

        Returns:
            Tupla (x, y, z, vel_xyz, acc_xyz, yaw_rad).
        """
        zero3 = (0.0, 0.0, 0.0)

        # === VERIFICAÇÕES GLOBAIS ===

        if self.drone_context.state_px4.local_position is None:
            self.get_logger().warn("Trajetória: Posição local indisponível", throttle_duration_sec=2.0)
            return (0.0, 0.0, 0.0, zero3, zero3, None)

        if self.drone_context.target_local_position is None:
            self.get_logger().warn("Trajetória: Posição alvo não definida", throttle_duration_sec=2.0)
            return (
                self.drone_context.state_px4.local_position.x,
                self.drone_context.state_px4.local_position.y,
                self.drone_context.state_px4.local_position.z,
                zero3, zero3,
                self.drone_context.state_px4.current_yaw_rad,
            )

        # === CÁLCULOS COMUNS ===

        target_x = self.drone_context.target_local_position[0]
        target_y = self.drone_context.target_local_position[1]
        target_z = self.drone_context.target_local_position[2]

        current_x = self.drone_context.state_px4.local_position.x
        current_y = self.drone_context.state_px4.local_position.y
        current_z = self.drone_context.state_px4.local_position.z

        # === MATCH/CASE POR ESTADO ===

        current_state = self.drone_context.state

        match current_state:

            # ============== ESTADOS DE DECOLAGEM ==============

            case DroneStateDescription.VOANDO_DECOLANDO:
                # Sobe até altitude alvo mantendo posição X,Y e yaw inicial.
                # Decolagem segue controle de posição puro (perfil trapezoidal não se aplica
                # ao takeoff, que é gerenciado pelo controlador interno do PX4).
                return (current_x, current_y, target_z, zero3, zero3,
                        self.drone_context.state_px4.home_yaw_rad)

            # ============== ESTADOS DE ROTAÇÃO (sem deslocamento) ==============

            case DroneStateDescription.VOANDO_GIRANDO_INICIO:
                self._reset_profile_if_active()
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_context.state_px4.current_yaw_rad,
                    self.drone_context.target_direction_yaw_rad,
                    self.drone_context.yaw_step_deg,
                )
                return (current_x, current_y, current_z, zero3, zero3, next_yaw_rad)

            case DroneStateDescription.VOANDO_GIRANDO_COM_FOCO:
                self._reset_profile_if_active()
                focus_yaw_rad = self._calculate_focus_yaw(current_x, current_y)
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_context.state_px4.current_yaw_rad,
                    focus_yaw_rad,
                    self.drone_context.yaw_step_deg,
                )
                return (current_x, current_y, current_z, zero3, zero3, next_yaw_rad)

            case DroneStateDescription.VOANDO_GIRANDO_FIM:
                self._reset_profile_if_active()
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_context.state_px4.current_yaw_rad,
                    self.drone_context.target_final_yaw_rad,
                    self.drone_context.yaw_step_deg,
                )
                hover_x, hover_y, hover_z, hover_yaw = self._get_hover_position_with_yaw(
                    next_yaw_rad, (target_x, target_y, target_z)
                )
                return (hover_x, hover_y, hover_z, zero3, zero3, hover_yaw)

            case DroneStateDescription.RETORNANDO_GIRANDO_INICIO:
                self._reset_profile_if_active()
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_context.state_px4.current_yaw_rad,
                    self.drone_context.target_direction_yaw_rad,
                    self.drone_context.yaw_step_deg,
                )
                return (current_x, current_y, current_z, zero3, zero3, next_yaw_rad)

            case DroneStateDescription.RETORNANDO_GIRANDO_FIM:
                self._reset_profile_if_active()
                next_yaw_rad = self._calculate_incremental_yaw(
                    self.drone_context.state_px4.current_yaw_rad,
                    self.drone_context.state_px4.home_yaw_rad,
                    self.drone_context.yaw_step_deg,
                )
                hover_x, hover_y, hover_z, hover_yaw = self._get_hover_position_with_yaw(
                    next_yaw_rad, (target_x, target_y, target_z)
                )
                return (hover_x, hover_y, hover_z, zero3, zero3, hover_yaw)

            # ============== ESTADOS DE DESLOCAMENTO (perfil trapezoidal) ==============

            case DroneStateDescription.VOANDO_A_CAMINHO:
                if self.drone_context.target_direction_yaw_rad is not None:
                    yaw_to_use = self.drone_context.target_direction_yaw_rad
                elif self.drone_context.last_static_yaw_rad is not None:
                    yaw_to_use = self.drone_context.last_static_yaw_rad
                else:
                    yaw_to_use = self.drone_context.state_px4.current_yaw_rad
                pos, vel, acc = self._advance_profile(
                    (current_x, current_y, current_z), (target_x, target_y, target_z)
                )
                return (*pos, vel, acc, yaw_to_use)

            case DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO:
                pos, vel, acc = self._advance_profile(
                    (current_x, current_y, current_z), (target_x, target_y, target_z)
                )
                # Yaw aponta da posição atual ao foco a cada tick
                focus_yaw_rad = self._calculate_focus_yaw(current_x, current_y)
                return (*pos, vel, acc, focus_yaw_rad)

            case DroneStateDescription.RETORNANDO_A_CAMINHO:
                pos, vel, acc = self._advance_profile(
                    (current_x, current_y, current_z), (target_x, target_y, target_z)
                )
                return (*pos, vel, acc, self.drone_context.target_direction_yaw_rad)

            # ============== ESTADO POUSANDO ==============

            case DroneStateDescription.POUSANDO:
                self._reset_profile_if_active()
                return (current_x, current_y, current_z, zero3, zero3,
                        self.drone_context.state_px4.home_yaw_rad)

            # ============== ESTADO PADRÃO ==============

            case _:
                self._reset_profile_if_active()
                self.get_logger().debug(
                    f"Trajetória: Estado {current_state.name} não requer setpoint específico",
                    throttle_duration_sec=5.0,
                )
                return (current_x, current_y, current_z, zero3, zero3,
                        self.drone_context.last_static_yaw_rad)



    def _calculate_incremental_yaw(self: 'DroneNode', current_yaw_rad: float, target_yaw_rad: float, max_step_deg: float) -> float:
        """
        Calcula o próximo yaw incremental para rotação suave.
        
        Em vez de enviar o target_yaw diretamente, esta função calcula um passo
        intermediário de no máximo max_step_deg graus (ou a distância restante se menor).
        
        Args:
            current_yaw_rad: Yaw atual em radianos (-pi a pi)
            target_yaw_rad: Yaw alvo em radianos (-pi a pi)
            max_step_deg: Passo máximo em graus (ex: 45.0)
        
        Returns:
            float: Próximo yaw em radianos (-pi a pi)
        """
        from drone_inspetor.common.math_utils import yaw_step_toward
        next_deg = yaw_step_toward(
            math.degrees(current_yaw_rad),
            math.degrees(target_yaw_rad),
            max_step_deg,
        )
        return math.radians(next_deg)

    def _calculate_focus_yaw(self: 'DroneNode', from_x: float, from_y: float) -> float:
        """
        Calcula e armazena o yaw para apontar ao focus a partir de uma posição.
        
        Args:
            from_x: Posição X de origem (metros)
            from_y: Posição Y de origem (metros)
        
        Returns:
            float: Yaw em radianos (-pi a pi)
        """
        from drone_inspetor.common.math_utils import yaw_deg_to_0_360
        focus_dx = self.drone_context.focus_local_position[0] - from_x
        focus_dy = self.drone_context.focus_local_position[1] - from_y
        focus_yaw_rad = math.atan2(focus_dy, focus_dx)  # atan2 já retorna -π..π
        focus_yaw_deg = math.degrees(focus_yaw_rad)

        # Armazena as 3 representações redundantes (consumidas por callers diferentes)
        self.drone_context.focus_yaw_deg_normalized = focus_yaw_deg
        self.drone_context.focus_yaw_deg = yaw_deg_to_0_360(focus_yaw_deg)
        self.drone_context.focus_yaw_rad = focus_yaw_rad

        return focus_yaw_rad

    # ------------------------------------------------------------------
    # Integração com o TrajectoryProfile (perfil trapezoidal)
    # ------------------------------------------------------------------

    # Período do timer de trajetória (precisa casar com o create_timer no DroneNode).
    _TRAJ_DT = 0.02

    def _advance_profile(self: 'DroneNode', current_pos: tuple, target_pos: tuple) -> tuple:
        """
        Garante que o profile está rodando para o segmento (current → target) atual e
        avança um tick. Detecta segmento novo comparando origem/destino do profile com
        o target_local_position vigente — ao mudar (entrada nos estados de deslocamento
        ou re-target por desvio de obstáculo), reinicia o segmento a partir da posição
        atual.

        Returns:
            (pos_xyz, vel_xyz, acc_xyz)
        """
        profile = self.trajectory_profile

        is_new_segment = (
            not profile.is_active()
            or profile.target is None
            or self._distance_squared(profile.target, target_pos) > 1e-4
        )
        if is_new_segment:
            v_initial = self._current_speed_along(target_pos, current_pos)
            profile.start_segment(current_pos, target_pos, v_initial=v_initial)

        v_target_obs = self.drone_context.obstacles.get_velocity_cap_from_obstacles(
            self.param_cruise_velocity
        )
        return profile.tick(self._TRAJ_DT, current_pos, v_target_obs=v_target_obs)

    def _reset_profile_if_active(self: 'DroneNode') -> None:
        """Encerra o segmento corrente quando o estado não é de deslocamento."""
        if self.trajectory_profile.is_active():
            self.trajectory_profile.reset()

    def _current_speed_along(self: 'DroneNode', target_pos: tuple, current_pos: tuple) -> float:
        """
        Componente escalar não-negativa da velocidade real do drone projetada na direção
        atual→destino. Permite emendar segmentos sem derrubar a velocidade a zero.
        """
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        dz = target_pos[2] - current_pos[2]
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            return 0.0
        ux, uy, uz = dx / norm, dy / norm, dz / norm
        vx = self.drone_context.state_px4.current_velocity_x
        vy = self.drone_context.state_px4.current_velocity_y
        vz = self.drone_context.state_px4.current_velocity_z
        return max(0.0, vx * ux + vy * uy + vz * uz)

    @staticmethod
    def _distance_squared(a: tuple, b: tuple) -> float:
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2

    def _get_hover_position_with_yaw(self: 'DroneNode', yaw_rad: float, fallback_pos: tuple) -> tuple:
        """
        Retorna posição para hover estável com yaw especificado.
        Usa last_static_position se disponível, senão usa fallback_pos.
        
        Args:
            yaw_rad: Yaw em radianos
            fallback_pos: (x, y, z) posição fallback se last_static_position não disponível
        
        Returns:
            tuple: (x, y, z, yaw_rad)
        """
        if self.drone_context.last_static_position is not None:
            return (
                self.drone_context.last_static_position[0],
                self.drone_context.last_static_position[1],
                self.drone_context.last_static_position[2],
                yaw_rad
            )
        return (fallback_pos[0], fallback_pos[1], fallback_pos[2], yaw_rad)


    def global_to_local_position(self: 'DroneNode', target_lat, target_lon, target_alt):
        """
        Converte coordenadas globais (GPS) para coordenadas locais (NED).
        
        Usa a posição GPS do HOME (onde local = 0,0,0) armazenada quando o drone armou.
        Isso garante que a conversão seja consistente durante todo o voo.
        
        Args:
            target_lat: Latitude alvo (graus)
            target_lon: Longitude alvo (graus)
            target_alt: Altitude alvo (metros)
        
        Returns:
            Lista [x, y, z] com posição local em metros (coordenadas NED)
            None se não houver referência HOME disponível
        """
        # Verifica se a referência HOME foi armazenada (quando armou)
        if self.drone_context.state_px4.home_global_lat is None:
            self.get_logger().warn(
                "global_to_local_position: Referência HOME não disponível. "
                "O drone precisa armar primeiro para definir a origem.",
                throttle_duration_sec=5.0
            )
            return None
        
        # Calcula offset da posição HOME (origem do frame local) para a posição alvo
        offset = self.global_to_local_offset(
            self.drone_context.state_px4.home_global_lat,
            self.drone_context.state_px4.home_global_lon,
            self.drone_context.state_px4.home_global_alt,
            target_lat,
            target_lon,
            target_alt
        )
        
        # O offset É a posição local diretamente (pois o HOME é onde local = 0,0,0)
        # No frame NED: X=norte, Y=leste, Z=para baixo (negativo = para cima)
        local_x = offset[0]  # Norte
        local_y = offset[1]  # Leste
        local_z = -offset[2]  # Para cima (inverte porque Z é negativo para cima no NED)
        
        return [local_x, local_y, local_z]
    

    def global_to_local_offset(self: 'DroneNode', lat1, lon1, alt1, lat2, lon2, alt2):
        """Delega para math_utils.global_to_local_offset (mantido por compat de API)."""
        from drone_inspetor.common.math_utils import global_to_local_offset
        return list(global_to_local_offset(lat1, lon1, alt1, lat2, lon2, alt2))
    
