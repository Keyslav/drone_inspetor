# =================================================================================================
# PlanandoState — estado da DeslocamentoFSM
# =================================================================================================
# DeslocamentoFSMDescription.PLANANDO
#
# Significado: drone em HOVER, aguardando aparecer um target na pilha. É o estado
# "idle" da DeslocamentoFSM: nenhum movimento ativo, apenas mantém posição usando
# `context.last_static_position` como referência.
#
# Transições:
#     target_stack.current não-vazio + posição local conhecida → GIRANDO_INICIO.
# =================================================================================================

import math

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS


class PlanandoState(BaseState):
    """Hover idle, aguardando target ser empilhado na TargetStack."""

    def on_enter(self) -> None:
        # Captura a posição atual como referência de hover para o caso de termos
        # acabado de chegar de uma manobra (GIRANDO_FIM faz isso, mas reforça aqui).
        self.context.store_static_position()
        # Limpa auxiliares de yaw da manobra anterior.
        self.context.yaw_aligned_time = None
        self.node.get_logger().info(
            "DeslocamentoFSM: PLANANDO (hover, aguardando target).",
            throttle_duration_sec=10.0,
        )

    def on_step(self):
        ctx = self.context
        target = ctx.target_stack.current

        # Pilha vazia: continua em hover. Nada para transicionar.
        if target is None:
            return None

        # Sem posição local ainda: não dá para calcular direção de aproximação.
        if ctx.state_px4.local_position is None:
            return None

        # Há target no topo da pilha: calcula yaw de direção e parte para a manobra.
        # O cálculo da direção é feito aqui (e não no push da pilha) porque a direção
        # depende da posição atual no momento de iniciar a manobra — pode ter mudado
        # caso o drone tenha ficado parado um tempo entre o push e o início.
        self._calcular_yaw_direcao(target)

        # Direção indefinida (target praticamente sobre o drone): pula direto para
        # GIRANDO_FIM (alinha yaw final, se houver) ou pop imediato via o próprio fluxo.
        if target.direction_yaw_rad is None:
            return TS.GIRANDO_FIM

        return TS.GIRANDO_INICIO

    # =============================================================================================
    # Helpers
    # =============================================================================================

    def _calcular_yaw_direcao(self, target) -> None:
        """
        Calcula o yaw de direção do drone para o target, considerando a posição atual.

        Se o target estiver praticamente em cima do drone (sem componente XY apreciável),
        define os campos como None — sinal de que não há manobra de giro inicial necessária.
        """
        cur = self.node.state_px4.local_position
        dx = target.local_position[0] - cur.x
        dy = target.local_position[1] - cur.y

        # Threshold de 0.1m: abaixo disso, mover lateral não faz sentido — fica só o yaw final.
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            target.direction_yaw_deg = None
            target.direction_yaw_deg_normalized = None
            target.direction_yaw_rad = None
            return

        yaw_deg = math.degrees(math.atan2(dy, dx))
        # Normaliza para [-180, 180]
        yaw_norm = ((yaw_deg + 180.0) % 360.0) - 180.0
        target.direction_yaw_deg_normalized = yaw_norm
        target.direction_yaw_deg = yaw_norm if yaw_norm >= 0 else yaw_norm + 360
        target.direction_yaw_rad = math.radians(yaw_norm)
