# machine.py
# =================================================================================================
# MÁQUINA DE ESTADOS DE MISSÃO (SUBCLASSE DE STATEMACHINE)
# =================================================================================================
# Implementa verificações globais antes do processamento de estado:
# 1. Aguarda primeiro contato com drone_node
# 2. Detecta perda de modo OFFBOARD → reset
# 3. Processa flag cancel_mission → RETORNANDO
# =================================================================================================

from drone_inspetor.base_classes.base_state_machine import BaseStateMachine
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription
from drone_inspetor.nodes.mission_node.fsm.mission.context import MissionFSMContext


class MissionFSM(BaseStateMachine):
    """Máquina de estados de missão com verificações globais."""

    def __init__(self, context: MissionFSMContext, node):
        super().__init__()
        self.context = context
        self.node = node

    def transition_to(self, new_id) -> None:
        """Transiciona e sincroniza context.state."""
        if new_id == self._current_state_id:
            return
        old_name = self._current_state_id.name if self._current_state_id else "None"
        super().transition_to(new_id)
        self.context.state = new_id
        self.node.get_logger().info(f"MUDANÇA DE ESTADO MISSION: {old_name} -> {new_id.name}")

    def tick(self) -> None:
        """Executa verificações globais e depois o ciclo de estado."""
        old_id = self._current_state_id

        # 1. Bloqueio: aguarda primeiro contato com drone_node
        if self.node.essencial_topics["drone_state"]["last_received"] == 0.0:
            self.node.get_logger().info("Aguardando conexão com drone_node...", throttle_duration_sec=5.0)
            return

        drone_state = self.node.drone.state

        # 2. Offboard perdido → reset
        if self._current_state_id != MissionFSMDescription.DESATIVADO:
            if drone_state == DroneFSMDescription.OFFBOARD_DESATIVADO:
                self.node.get_logger().error(
                    f"Drone saiu do modo OFFBOARD ({drone_state.name}). Resetando missão..."
                )
                self.context.reset()
                self.transition_to(MissionFSMDescription.DESATIVADO)
                return

        # 3. Cancel mission flag
        if self.context.cancel_mission:
            self.node.get_logger().warn("Flag cancel_mission detectada. Transicionando para RETORNANDO...")
            self.context.on_mission = False
            self.context.cancel_mission = False
            self.node.cancel_current_action()
            self.transition_to(MissionFSMDescription.RETORNANDO)
            return

        # Se um global check transicionou, não processa on_step neste ciclo
        if self._current_state_id != old_id:
            return

        super().tick()

    def register_all_states(self):
        """Registra todos os estados da missão."""
        from drone_inspetor.nodes.mission_node.fsm.mission.states.desativado import DesativadoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.pronto import ProntoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_armando import ExecutandoArmandoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_decolando import ExecutandoDecolandoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_inspecionando import ExecutandoInspecionandoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_inspecionando_detectando import ExecutandoDetectandoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_inspecionando_escaneando import ExecutandoEscaneandoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_inspecionando_escaneamento_finalizado import ExecutandoEscaneamentoFinalizadoState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.executando_inspecionando_falha import ExecutandoFalhaState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.inspecao_finalizada import InspecaoFinalizadaState
        from drone_inspetor.nodes.mission_node.fsm.mission.states.retornando import RetornandoState

        DS = MissionFSMDescription
        for state_id, state_cls in [
            (DS.DESATIVADO, DesativadoState),
            (DS.PRONTO, ProntoState),
            (DS.EXECUTANDO_ARMANDO, ExecutandoArmandoState),
            (DS.EXECUTANDO_DECOLANDO, ExecutandoDecolandoState),
            (DS.EXECUTANDO_INSPECIONANDO, ExecutandoInspecionandoState),
            (DS.EXECUTANDO_INSPECIONANDO_DETECTANDO, ExecutandoDetectandoState),
            (DS.EXECUTANDO_INSPECIONANDO_ESCANEANDO, ExecutandoEscaneandoState),
            (DS.EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO, ExecutandoEscaneamentoFinalizadoState),
            (DS.EXECUTANDO_INSPECIONANDO_FALHA, ExecutandoFalhaState),
            (DS.INSPECAO_FINALIZADA, InspecaoFinalizadaState),
            (DS.RETORNANDO, RetornandoState),
        ]:
            self.register(state_id, state_cls(self.context, self.node))
