# trajectory_profile.py
# =================================================================================================
# PERFIL DE VELOCIDADE TRAPEZOIDAL PARA DESLOCAMENTO DO DRONE
# =================================================================================================
# Encapsula a FSM interna de fases (ACEL / CRUISE / BRAKE / OBSTACLE_BRAKE / DONE) que gera, a
# cada tick, um setpoint de posição/velocidade/aceleração coerente com um perfil trapezoidal
# (com degeneração para triangular em segmentos curtos). Stateless em relação ao DroneNode:
# recebe a configuração no construtor, é (re)inicializado por segmento via start_segment() e
# atualizado por tick().
# =================================================================================================

import math
from enum import Enum, auto
from typing import Optional


class TrajectoryPhase(Enum):
    IDLE = auto()
    ACEL = auto()
    CRUISE = auto()
    BRAKE = auto()
    OBSTACLE_BRAKE = auto()
    DONE = auto()


class TrajectoryProfile:
    """
    Gerador de setpoints com perfil trapezoidal de velocidade ao longo de um segmento reto
    do ponto de partida ao destino.

    USO:
        profile = TrajectoryProfile(vc=3.0, ad=1.0, ao=2.0, arrival_tol=0.2)
        profile.start_segment(origin, target)
        # a cada tick (dt segundos):
        pos, vel, acc = profile.tick(dt, current_pos=..., v_target_obs=None)
        if profile.is_done(): ...
    """

    def __init__(self, vc: float, ad: float, ao: float,
                 arrival_tol: float = 0.2, arrival_v_tol: float = 0.1):
        """
        Args:
            vc:          Velocidade de cruzeiro (m/s).
            ad:          Aceleração de deslocamento (m/s²) usada em ACEL e BRAKE.
            ao:          Desaceleração quando obstáculo próximo (m/s²). Deve ser > ad.
            arrival_tol: Distância (m) para considerar destino alcançado.
            arrival_v_tol: Velocidade abaixo da qual o destino é considerado estabilizado.
        """
        self.vc = vc
        self.ad = ad
        self.ao = ao
        self.arrival_tol = arrival_tol
        self.arrival_v_tol = arrival_v_tol

        # --- Estado do segmento atual ---
        self.phase: TrajectoryPhase = TrajectoryPhase.IDLE
        self.origin: Optional[tuple] = None
        self.target: Optional[tuple] = None
        self.direction: Optional[tuple] = None  # vetor unitário 3D
        self.total_distance: float = 0.0

        # Escalar de velocidade desejada (m/s, sempre >= 0; vetor = direction * v_des)
        self.v_des: float = 0.0
        # Escalar de aceleração desejada (m/s², com sinal: + acelera no sentido do movimento)
        self.a_des: float = 0.0
        # Posição integrada do setpoint (referência primária enviada ao PX4)
        self.pos_setpoint: Optional[tuple] = None

    # ------------------------------------------------------------------
    # Ciclo de vida do segmento
    # ------------------------------------------------------------------

    def start_segment(self, origin: tuple, target: tuple, v_initial: float = 0.0) -> None:
        """
        Inicializa um novo segmento. A direção fica congelada até o fim do segmento — desvios
        de posição reais são corrigidos pelo controlador interno do PX4, não recalculados aqui.

        Args:
            origin:    (x, y, z) ponto de partida (geralmente posição atual do drone)
            target:    (x, y, z) ponto de destino
            v_initial: Velocidade escalar inicial (m/s). Útil para emendar segmentos sem
                       parar; default 0 (parte do hover).
        """
        ox, oy, oz = origin
        tx, ty, tz = target
        dx, dy, dz = tx - ox, ty - oy, tz - oz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.origin = (ox, oy, oz)
        self.target = (tx, ty, tz)
        self.total_distance = dist
        self.pos_setpoint = (ox, oy, oz)
        self.v_des = max(0.0, v_initial)
        self.a_des = 0.0

        if dist < self.arrival_tol:
            self.direction = (0.0, 0.0, 0.0)
            self.phase = TrajectoryPhase.DONE
            self.v_des = 0.0
            return

        self.direction = (dx / dist, dy / dist, dz / dist)
        self.phase = TrajectoryPhase.ACEL

    def reset(self) -> None:
        """Encerra o segmento atual e zera o estado. Próximo tick retorna IDLE/zerado."""
        self.phase = TrajectoryPhase.IDLE
        self.origin = None
        self.target = None
        self.direction = None
        self.total_distance = 0.0
        self.v_des = 0.0
        self.a_des = 0.0
        self.pos_setpoint = None

    def is_done(self) -> bool:
        return self.phase == TrajectoryPhase.DONE

    def is_active(self) -> bool:
        return self.phase not in (TrajectoryPhase.IDLE, TrajectoryPhase.DONE)

    # ------------------------------------------------------------------
    # Tick — chamado pelo timer de trajetória (ex.: 50 Hz, dt = 0.02)
    # ------------------------------------------------------------------

    def tick(self, dt: float, current_pos: tuple,
             v_target_obs: Optional[float] = None) -> tuple:
        """
        Avança o perfil em dt segundos e retorna (pos, vel, acc) como tuplas 3D em frame local NED.

        Args:
            dt:           Intervalo desde o último tick (segundos).
            current_pos:  (x, y, z) posição real do drone agora — usada apenas para calcular a
                          distância restante real ao destino (não para corrigir direção).
            v_target_obs: Cap de velocidade imposto por obstáculo (m/s). None = sem cap.

        Returns:
            (pos_xyz, vel_xyz, acc_xyz) — todos tuplas (x, y, z).
            Quando IDLE/DONE, posição é o último setpoint conhecido (ou origem/destino) e
            velocidade/aceleração são zero.
        """
        if self.phase == TrajectoryPhase.IDLE:
            zero = (0.0, 0.0, 0.0)
            pos = self.pos_setpoint or zero
            return pos, zero, zero

        if self.phase == TrajectoryPhase.DONE:
            zero = (0.0, 0.0, 0.0)
            return self.target or zero, zero, zero

        # --- Distância restante ao destino, medida pela posição REAL ---
        # (mais robusto que usar pos_setpoint, que pode divergir levemente)
        cx, cy, cz = current_pos
        tx, ty, tz = self.target
        dx, dy, dz = tx - cx, ty - cy, tz - cz
        s_remaining = math.sqrt(dx * dx + dy * dy + dz * dz)

        # --- Decide fase deste tick ---
        self._update_phase(s_remaining, v_target_obs)

        # --- Atualiza a_des conforme fase ---
        if self.phase == TrajectoryPhase.ACEL:
            self.a_des = +self.ad
        elif self.phase == TrajectoryPhase.CRUISE:
            self.a_des = 0.0
        elif self.phase == TrajectoryPhase.BRAKE:
            self.a_des = -self.ad
        elif self.phase == TrajectoryPhase.OBSTACLE_BRAKE:
            self.a_des = -self.ao
        else:  # DONE caiu aqui via _update_phase
            zero = (0.0, 0.0, 0.0)
            self.v_des = 0.0
            self.a_des = 0.0
            return self.target, zero, zero

        # --- Integra v_des com saturação coerente ---
        new_v = self.v_des + self.a_des * dt
        v_cap_cruise = self.vc if v_target_obs is None else min(self.vc, v_target_obs)
        if self.phase == TrajectoryPhase.ACEL:
            new_v = min(new_v, v_cap_cruise)
        elif self.phase == TrajectoryPhase.CRUISE:
            new_v = v_cap_cruise
        elif self.phase in (TrajectoryPhase.BRAKE, TrajectoryPhase.OBSTACLE_BRAKE):
            new_v = max(new_v, 0.0)
        self.v_des = new_v

        # --- Integra posição do setpoint usando v médio do tick (trapezoidal) ---
        v_avg = max(0.0, self.v_des - 0.5 * self.a_des * dt)
        step = v_avg * dt
        if step > s_remaining:
            step = s_remaining  # não passa do destino
        psx = self.pos_setpoint[0] + self.direction[0] * step
        psy = self.pos_setpoint[1] + self.direction[1] * step
        psz = self.pos_setpoint[2] + self.direction[2] * step
        self.pos_setpoint = (psx, psy, psz)

        # --- Vetores de saída ---
        vel = (self.direction[0] * self.v_des,
               self.direction[1] * self.v_des,
               self.direction[2] * self.v_des)
        acc = (self.direction[0] * self.a_des,
               self.direction[1] * self.a_des,
               self.direction[2] * self.a_des)

        return self.pos_setpoint, vel, acc

    # ------------------------------------------------------------------
    # Lógica de transição de fase
    # ------------------------------------------------------------------

    def _update_phase(self, s_remaining: float, v_target_obs: Optional[float]) -> None:
        # Chegada: distância pequena E velocidade baixa
        if s_remaining < self.arrival_tol and self.v_des < self.arrival_v_tol:
            self.phase = TrajectoryPhase.DONE
            return

        # Distância de frenagem necessária para zerar a partir de v_des com -ad
        # (mesma fórmula serve para perfil triangular: a transição ACEL→BRAKE acontece
        # naturalmente quando s_brake alcança s_remaining antes de v_des chegar a vc)
        s_brake = (self.v_des * self.v_des) / (2.0 * self.ad) if self.ad > 0 else 0.0

        # --- Prioridade 1: obstáculo demanda redução abaixo de v_des ---
        if v_target_obs is not None and self.v_des > v_target_obs + 1e-3:
            self.phase = TrajectoryPhase.OBSTACLE_BRAKE
            return

        # --- Prioridade 2: hora de frear para chegar parado no destino ---
        if s_remaining <= s_brake + 1e-3:
            self.phase = TrajectoryPhase.BRAKE
            return

        # --- Caso contrário: acelera até vc (ou cap por obstáculo), depois cruza ---
        v_cap = self.vc if v_target_obs is None else min(self.vc, v_target_obs)
        if self.v_des < v_cap - 1e-3:
            self.phase = TrajectoryPhase.ACEL
        else:
            self.phase = TrajectoryPhase.CRUISE
