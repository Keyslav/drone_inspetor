# =================================================================================================
# lidar_obstacle.py
# =================================================================================================
# PROCESSAMENTO DE DADOS BRUTOS DO LIDAR → FLAGS DE OBSTÁCULO
# =================================================================================================
# Recebe LidarMSG (point_vector + ground_distance) e processa os dados brutos para
# gerar todas as 11 flags de obstáculo. Lógica replicada do lidar_node original.
#
# O LiDAR tem cobertura 360° horizontal + sensor inferior, portanto atualiza TODAS as flags:
#   - Distância:  8m, 5m, 3m, 2m, 1m
#   - Quadrante:  front_90, right_90, back_90, left_90
#   - Inferior:   down_1m, down_05m
#
# Cooldown de 3s: uma flag ativada permanece True por no mínimo 3 segundos,
# evitando oscilações rápidas causadas por ruído ou intermitência do sensor.
# =================================================================================================

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_inspetor_msgs.msg import LidarMSG


class LidarObstacle:
    """
    Processa dados brutos do LiDAR (LidarMSG) e gera flags de obstáculo.

    O LiDAR cobre 360° no plano horizontal e possui sensor inferior,
    sendo capaz de atualizar todas as 11 flags de obstáculo.
    """

    # Tempo mínimo (s) que uma flag ativa permanece True antes de poder ser resetada.
    COOLDOWN_SECONDS: float = 3.0

    # Range válido de distância do LiDAR — pontos fora são descartados.
    MIN_RANGE: float = 0.1   # metros
    MAX_RANGE: float = 12.0  # metros

    def __init__(self):
        # Flags de obstáculo detectadas pelo LiDAR.
        self._flags: dict[str, bool] = {
            'have_obstacles_8m': False,
            'have_obstacles_5m': False,
            'have_obstacles_3m': False,
            'have_obstacles_2m': False,
            'have_obstacles_1m': False,
            'have_obstacles_front_90': False,
            'have_obstacles_right_90': False,
            'have_obstacles_back_90': False,
            'have_obstacles_left_90': False,
            'have_obstacles_down_1m': False,
            'have_obstacles_down_05m': False,
        }

        # Timestamp de última ativação de cada flag (para cooldown).
        self._flag_set_times: dict[str, float] = {k: 0.0 for k in self._flags}

    # =============================================================================================
    # API pública
    # =============================================================================================

    def process(self, msg: 'LidarMSG') -> None:
        """
        Processa uma mensagem LidarMSG e atualiza as flags de obstáculo.

        Args:
            msg: LidarMSG contendo point_vector [dist1, ang1, dist2, ang2, ...] (rad)
                 e ground_distance (metros).
        """
        # 1) Processa o point_vector (pares distância/ângulo do LiDAR 2D horizontal).
        self._process_point_vector(msg.point_vector)

        # 2) Processa o sensor inferior (ground_distance).
        self._process_ground_distance(msg.ground_distance)

    @property
    def flags(self) -> dict[str, bool]:
        """Retorna cópia das flags atuais (leitura segura)."""
        return dict(self._flags)

    # =============================================================================================
    # Processamento interno
    # =============================================================================================

    def _process_point_vector(self, point_vector: list) -> None:
        """
        Processa o vetor de pontos do LiDAR 2D.

        O point_vector contém pares intercalados [dist1, ang1, dist2, ang2, ...]:
            - dist: distância em metros
            - ang:  ângulo em radianos

        Atualiza flags de distância (qualquer ângulo) e quadrante (apenas dist <= 1m).
        Após processar todos os pontos, tenta resetar flags não confirmadas (com cooldown).
        """
        import math

        # Reseta flags para False (respeitando cooldown) antes de reavaliar.
        self._reset_all_flags()

        # Itera sobre pares (distância, ângulo) no point_vector.
        for i in range(0, len(point_vector) - 1, 2):
            dist = point_vector[i]
            angle_rad = point_vector[i + 1]

            # Descarta pontos fora do range válido.
            if not (self.MIN_RANGE <= dist <= self.MAX_RANGE):
                continue

            # --- Flags de distância (qualquer ângulo horizontal) ---
            if dist <= 8.0:
                self._set_flag('have_obstacles_8m', True)
            if dist <= 5.0:
                self._set_flag('have_obstacles_5m', True)
            if dist <= 3.0:
                self._set_flag('have_obstacles_3m', True)
            if dist <= 2.0:
                self._set_flag('have_obstacles_2m', True)
            if dist <= 1.0:
                self._set_flag('have_obstacles_1m', True)

            # --- Flags de quadrante (apenas obstáculos a <= 1 metro) ---
            if dist > 1.0:
                continue

            # Normaliza ângulo para [-180, 180] graus.
            angle_deg = math.degrees(angle_rad)
            angle_norm = ((angle_deg + 180) % 360) - 180

            # Classifica no quadrante correspondente (arcos de 90°).
            if -45 <= angle_norm <= 45:
                self._set_flag('have_obstacles_front_90', True)
            elif 45 < angle_norm <= 135:
                self._set_flag('have_obstacles_right_90', True)
            elif -135 <= angle_norm < -45:
                self._set_flag('have_obstacles_left_90', True)
            else:
                # Trás: +135° a +180° ou -135° a -180°
                self._set_flag('have_obstacles_back_90', True)

    def _process_ground_distance(self, ground_distance: float) -> None:
        """
        Processa a leitura do sensor LiDAR inferior.

        Args:
            ground_distance: Distância ao solo/obstáculo abaixo (metros).
        """
        if ground_distance <= 0.0:
            return

        self._set_flag('have_obstacles_down_1m', ground_distance <= 1.0)
        self._set_flag('have_obstacles_down_05m', ground_distance <= 0.5)

    # =============================================================================================
    # Gerenciamento de flags com cooldown
    # =============================================================================================

    def _set_flag(self, name: str, value: bool) -> None:
        """
        Atualiza uma flag com cooldown.

        Se value=True: ativa a flag e registra o timestamp.
        Se value=False: só desativa se já passou o cooldown desde a última ativação.

        Args:
            name:  Nome da flag (chave em self._flags).
            value: Novo valor desejado.
        """
        current_time = time.monotonic()

        if value:
            self._flags[name] = True
            self._flag_set_times[name] = current_time
        else:
            # Só reseta se passaram COOLDOWN_SECONDS desde a última ativação.
            if current_time - self._flag_set_times[name] >= self.COOLDOWN_SECONDS:
                self._flags[name] = False

    def _reset_all_flags(self) -> None:
        """Tenta resetar todas as flags para False (respeitando cooldown)."""
        for name in self._flags:
            self._set_flag(name, False)
