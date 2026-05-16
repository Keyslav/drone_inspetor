# =================================================================================================
# depth_obstacle.py
# =================================================================================================
# ARMAZENAMENTO DE FLAGS DE OBSTÁCULO DA CÂMERA DE PROFUNDIDADE
# =================================================================================================
# Recebe ObstaclesMSG publicada pelo depth_node (que já processa a imagem de profundidade)
# e armazena apenas as flags que a câmera frontal consegue inferir:
#
#   - Distância:  8m, 5m, 3m, 2m, 1m  (campo de visão frontal)
#   - Quadrante:  front_90             (sensor frontal → apenas frente)
#
# Flags NÃO atualizadas (permanecem False):
#   - right_90, back_90, left_90  (sem cobertura lateral/traseira)
#   - down_1m, down_05m           (sem sensor inferior)
# =================================================================================================

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drone_inspetor_msgs.msg import ObstaclesMSG


# Campos que a câmera depth PODE atualizar (cobertura frontal).
_DEPTH_UPDATABLE_FIELDS = (
    'have_obstacles_8m',
    'have_obstacles_5m',
    'have_obstacles_3m',
    'have_obstacles_2m',
    'have_obstacles_1m',
    'have_obstacles_front_90',
)


class DepthObstacle:
    """
    Armazena flags de obstáculo vindas da câmera de profundidade (depth_node).

    A câmera depth é frontal — só cobre o campo de visão à frente do drone.
    Flags laterais, traseiras e inferiores não são atualizadas (permanecem False).
    """

    def __init__(self):
        # Flags atualizáveis pela câmera depth.
        self._flags: dict[str, bool] = {field: False for field in _DEPTH_UPDATABLE_FIELDS}

    # =============================================================================================
    # API pública
    # =============================================================================================

    def update(self, msg: 'ObstaclesMSG') -> None:
        """
        Atualiza as flags a partir de uma ObstaclesMSG publicada pelo depth_node.

        Apenas os campos que a câmera frontal consegue inferir são extraídos.
        Os demais campos da ObstaclesMSG são ignorados (cobertos pelo LiDAR).

        Args:
            msg: ObstaclesMSG com flags de obstáculo do depth_node.
        """
        for field in _DEPTH_UPDATABLE_FIELDS:
            self._flags[field] = getattr(msg, field)

    @property
    def flags(self) -> dict[str, bool]:
        """Retorna cópia das flags atuais (leitura segura)."""
        return dict(self._flags)
