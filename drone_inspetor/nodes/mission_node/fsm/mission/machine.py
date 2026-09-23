"""Máquina de missão: autoridade única de estado e de encerramento de sessão."""

from drone_inspetor.base_classes.base_state_machine import BaseStateMachine
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS


class MissionFSM(BaseStateMachine):
    """Valida saúde antes de decisões; reset sempre passa pela mesma transição."""

    def __init__(self, context, runtime):
        """Recebe dados de sessão e dependências sem herdar o nó ROS."""
        super().__init__()
        self.context = context
        self.node = runtime

    def transition_to(self, new_id):
        """Sincroniza efeitos de entrada sem manter cópia do estado no contexto."""
        if new_id == self.current_state_id:
            return
        old_id = self.current_state_id
        if new_id == MS.DESATIVADO:
            self.node.actions.cancel()
            self.node.cv.stop_inspection()
            self.context.clear()
        elif new_id == MS.PRONTO:
            self.context.clear()
        elif new_id == MS.RETORNANDO:
            self.node.cv.stop_inspection()
        super().transition_to(new_id)
        old_name = old_id.name if old_id is not None else 'inicial'
        self.node.get_logger().info(f'MISSÃO: {old_name} -> {new_id.name}')

    def reset(self, reason=''):
        """Invalida operações e dados mesmo se a máquina já estiver desativada."""
        if reason:
            self.node.get_logger().warning(reason)
        if self.current_state_id == MS.DESATIVADO:
            self.node.actions.cancel()
            self.node.cv.stop_inspection()
            self.context.clear()
        else:
            self.transition_to(MS.DESATIVADO)

    def tick(self):
        """Supervisiona transporte e telemetria antes do ciclo enter/step da FSM."""
        self.node.actions.poll()
        self.node.cv.poll()
        if not self.node.telemetry_healthy():
            if self.current_state_id != MS.DESATIVADO:
                self.reset('Telemetria ausente ou expirada; missão desativada')
            return
        drone = self.node.drone
        finished_return = (
            self.current_state_id == MS.RETORNANDO and drone.is_landed and not drone.is_armed
        )
        if self.current_state_id != MS.DESATIVADO and not finished_return:
            if drone.state in (DS.OFFBOARD_DESATIVADO, DS.EMERGENCIA):
                self.reset(f'Controle indisponível: {drone.state.name}')
                return
        if self.context.cancel_mission:
            self.context.cancel_mission = False
            self.context.on_mission = False
            self.node.actions.cancel()
            self.node.cv.stop_inspection()
            self.transition_to(MS.RETORNANDO)
            return
        super().tick()

    def register_all_states(self):
        """Registra todos os estados da missão."""
        from .states.desativado import DesativadoState
        from .states.pronto import ProntoState
        from .states.executando_armando import ExecutandoArmandoState
        from .states.executando_decolando import ExecutandoDecolandoState
        from .states.executando_inspecionando import ExecutandoInspecionandoState
        from .states.executando_inspecionando_detectando import ExecutandoDetectandoState
        from .states.executando_inspecionando_escaneando import ExecutandoEscaneandoState
        from .states.executando_inspecionando_escaneamento_finalizado import (
            ExecutandoEscaneamentoFinalizadoState,
        )
        from .states.executando_inspecionando_falha import ExecutandoFalhaState
        from .states.inspecao_finalizada import InspecaoFinalizadaState
        from .states.retornando import RetornandoState

        states = MS
        for state_id, state_cls in [
            (states.DESATIVADO, DesativadoState),
            (states.PRONTO, ProntoState),
            (states.EXECUTANDO_ARMANDO, ExecutandoArmandoState),
            (states.EXECUTANDO_DECOLANDO, ExecutandoDecolandoState),
            (states.EXECUTANDO_INSPECIONANDO, ExecutandoInspecionandoState),
            (states.EXECUTANDO_INSPECIONANDO_DETECTANDO, ExecutandoDetectandoState),
            (states.EXECUTANDO_INSPECIONANDO_ESCANEANDO, ExecutandoEscaneandoState),
            (states.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO,
             ExecutandoEscaneamentoFinalizadoState),
            (states.EXECUTANDO_INSPECIONANDO_FALHA, ExecutandoFalhaState),
            (states.INSPECAO_FINALIZADA, InspecaoFinalizadaState),
            (states.RETORNANDO, RetornandoState),
        ]:
            self.register(state_id, state_cls(self.context, self.node))
