# =================================================================================================
# GirandoFimState — estado da DeslocamentoFSM
# =================================================================================================
# DeslocamentoFSMDescription.GIRANDO_FIM
#
# Significado: drone chegou ao target, agora gira para o yaw FINAL especificado.
# Quando o yaw final está estabilizado (ou se não houver yaw final), desempilha
# o target e devolve o controle ao próximo target da pilha (ou PLANANDO se vazia).
#
# Transições:
#     target.final_yaw_rad é None → pop() + PLANANDO (sem yaw final a alinhar).
#     |yaw - final_yaw| <= tolerância + estabilização concluída → pop() + decisão:
#         pilha vazia → PLANANDO.
#         pilha não vazia (havia mais um target embaixo) → PLANANDO (re-inicia o ciclo
#                                                          que detectará o novo topo).
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS


class GirandoFimState(BaseState):
    """Girando para alinhar yaw final no destino. Desempilha ao concluir."""

    def on_enter(self) -> None:
        self.context.yaw_aligned_time = None
        target = self.context.target_stack.current
        if target is not None and target.final_yaw_deg is not None:
            self.node.get_logger().info(
                f"DeslocamentoFSM: GIRANDO_FIM → yaw alvo {target.final_yaw_deg:.1f}°."
            )

    def on_step(self):
        ctx = self.context
        target = ctx.target_stack.current

        # Pilha esvaziou inesperadamente: idle.
        if target is None:
            return TS.PLANANDO

        # Sem yaw final: nada para alinhar → desempilha imediato.
        if target.final_yaw_rad is None:
            self._concluir_target()
            return TS.PLANANDO

        px4 = ctx.state_px4

        # Em período de estabilização: yaw já alinhado, aguardando o delay.
        if ctx.yaw_aligned_time is not None:
            elapsed = ctx.now() - ctx.yaw_aligned_time
            if elapsed >= ctx.yaw_stabilization_delay:
                self.node.get_logger().info(
                    "DeslocamentoFSM: yaw final estável. Target concluído."
                )
                self._concluir_target()
                return TS.PLANANDO
            return None

        # Checa se o yaw atual já está dentro da tolerância.
        yaw_diff = ctx.yaw_diff_shortest(
            px4.current_yaw_deg_normalized,
            target.final_yaw_deg_normalized,
        )

        if abs(yaw_diff) <= ctx.yaw_tolerance_deg:
            ctx.yaw_aligned_time = ctx.now()
            self.node.get_logger().info(
                f"DeslocamentoFSM: yaw final alinhado ({target.final_yaw_deg:.1f}°). "
                "Aguardando estabilização..."
            )

        return None

    # =============================================================================================
    # Helpers
    # =============================================================================================

    def _concluir_target(self) -> None:
        """Marca o target como concluído: pop da pilha + captura posição como hover."""
        self.context.target_stack.pop()
        self.context.store_static_position()
        self.context.yaw_aligned_time = None
