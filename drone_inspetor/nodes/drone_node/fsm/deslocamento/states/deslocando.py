"""Observa a chegada; geração de movimento e desvios pertencem a Trajectory."""

import math

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import (
    DeslocamentoFSMDescription as TS,
)


class DeslocandoState(BaseState):
    """Conclui somente com posição e velocidade estabilizadas."""

    def on_enter(self):
        context = self.context
        context.trajectory_start_time = context.now()
        target = context.target_stack.current
        position = self.node.state_px4.local_position
        if target is not None and position is not None:
            context.initial_distance_to_target = math.dist(
                target.local_position, (position.x, position.y, position.z))

    def on_step(self):
        if self.context.target_stack.current is None:
            return TS.PLANANDO
        if self.node.trajectory.navigation_error:
            return TS.PLANANDO
        if self.node.trajectory_profile.is_done():
            self.context.last_static_position = list(self.node.trajectory_profile.target)
            return TS.GIRANDO_FIM
        return None
