# =================================================================================================
# GirandoInicioState — estado da DeslocamentoFSM
# =================================================================================================
# DeslocamentoFSMDescription.GIRANDO_INICIO
#
# Significado: drone girando em torno do próprio eixo para alinhar o yaw com a
# DIREÇÃO do target ativo (target_stack.current). A translação só começa depois
# que o yaw está estável dentro da tolerância.
#
# Transições:
#     |yaw - direction_yaw| <= tolerância + estabilização concluída → DESLOCANDO.
#     target.direction_yaw_rad é None (target em cima do drone) → GIRANDO_FIM.
#     Pilha esvaziou (cancelamento externo) → PLANANDO.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS


class GirandoInicioState(BaseState):
    """Girando para alinhar yaw com a direção do target."""

    def on_enter(self) -> None:
        # Zera timer de estabilização — começamos do zero.
        self.context.yaw_aligned_time = None
        target = self.context.target_stack.current
        if target is not None and target.direction_yaw_deg is not None:
            self.node.get_logger().info(
                f"DeslocamentoFSM: GIRANDO_INICIO → yaw alvo {target.direction_yaw_deg:.1f}°."
            )

    def on_step(self):
        ctx = self.context
        target = ctx.target_stack.current

        # Pilha esvaziou (ex.: cancelamento externo): volta ao idle.
        if target is None:
            return TS.PLANANDO

        # Sem direção definida (target sobre o drone): pula direto para o yaw final.
        if target.direction_yaw_rad is None:
            return TS.GIRANDO_FIM

        px4 = ctx.state_px4

        # Em período de estabilização: yaw já alinhado, aguardando tempo.
        if ctx.yaw_aligned_time is not None:
            elapsed = ctx.now() - ctx.yaw_aligned_time
            if elapsed >= ctx.yaw_stabilization_delay:
                self.node.get_logger().info(
                    "DeslocamentoFSM: yaw inicial estável. Iniciando deslocamento."
                )
                ctx.yaw_aligned_time = None
                return TS.DESLOCANDO
            return None

        # Verifica se o yaw atual já está dentro da tolerância.
        yaw_diff = ctx.yaw_diff_shortest(
            px4.current_yaw_deg_normalized,
            target.direction_yaw_deg_normalized,
        )

        if abs(yaw_diff) <= ctx.yaw_tolerance_deg:
            ctx.yaw_aligned_time = ctx.now()
            self.node.get_logger().info(
                f"DeslocamentoFSM: yaw inicial alinhado ({target.direction_yaw_deg:.1f}°). "
                "Aguardando estabilização..."
            )

        return None
