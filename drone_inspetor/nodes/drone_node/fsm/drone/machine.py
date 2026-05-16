# =================================================================================================
# DroneFSM — máquina de estados de lifecycle do DroneNode
# =================================================================================================
# Herda BaseStateMachine. Registra os 6 estados de lifecycle e executa verificações
# globais por ciclo (emergência, perda de offboard) ANTES de delegar ao estado atual.
#
# Espelha automaticamente o estado atual em `context.lifecycle_state` e
# `context.lifecycle_state_entry_time` em cada transição, para consumo por código
# externo (publishers de telemetria, GUI, mission_node).
# =================================================================================================

from drone_inspetor.base_classes.base_state_machine import BaseStateMachine
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
from drone_inspetor.nodes.drone_node.fsm.drone.context import DroneFSMContext
from drone_inspetor.nodes.drone_node.fsm.drone.states.offboard_desativado import OffboardDesativadoState
from drone_inspetor.nodes.drone_node.fsm.drone.states.pousado_desarmado import PousadoDesarmadoState
from drone_inspetor.nodes.drone_node.fsm.drone.states.pousado_armado import PousadoArmadoState
from drone_inspetor.nodes.drone_node.fsm.drone.states.decolando import DecolandoState
from drone_inspetor.nodes.drone_node.fsm.drone.states.em_voo import EmVooState
from drone_inspetor.nodes.drone_node.fsm.drone.states.emergencia import EmergenciaState


class DroneFSM(BaseStateMachine):
    """
    FSM principal do DroneNode — controla o ciclo de vida do drone.

    Estados (lifecycle):
        OFFBOARD_DESATIVADO, POUSADO_DESARMADO, POUSADO_ARMADO,
        DECOLANDO, EM_VOO, EMERGENCIA.

    A DeslocamentoFSM paralela (manobras) só é tickada quando o lifecycle está em EM_VOO,
    coordenada pelo DroneNode no seu loop de 50 Hz.
    """

    def __init__(self, context: DroneFSMContext, node):
        super().__init__()
        # Contexto compartilhado entre todos os estados da FSM (e também usado
        # pela DeslocamentoFSM paralela).
        self.context = context
        # Referência ao nó ROS2 (para logger/clock/helpers PX4).
        self.node = node

    # =============================================================================================
    # Registro dos estados
    # =============================================================================================
    def register_all_states(self) -> None:
        """Registra os 6 estados do lifecycle. Chamado uma única vez no startup."""
        self.register(DS.OFFBOARD_DESATIVADO, OffboardDesativadoState(self.context, self.node))
        self.register(DS.POUSADO_DESARMADO,   PousadoDesarmadoState(self.context, self.node))
        self.register(DS.POUSADO_ARMADO,      PousadoArmadoState(self.context, self.node))
        self.register(DS.DECOLANDO,           DecolandoState(self.context, self.node))
        self.register(DS.EM_VOO,              EmVooState(self.context, self.node))
        self.register(DS.EMERGENCIA,          EmergenciaState(self.context, self.node))

    # =============================================================================================
    # Override de transition_to — espelha o estado no contexto
    # =============================================================================================
    def transition_to(self, new_id) -> None:
        """
        Realiza a transição e atualiza o espelho do estado no contexto.

        Em adição ao comportamento base (chama on_exit/marca on_enter), grava o novo
        identificador em `context.lifecycle_state` e o timestamp em
        `context.lifecycle_state_entry_time`, para consumo externo (telemetria/GUI).
        """
        if new_id == self.current_state_id:
            return
        prev_name = self.current_state_id.name if self.current_state_id is not None else "—"
        new_name = new_id.name
        self.node.get_logger().info(f"[DroneFSM] {prev_name} → {new_name}")
        super().transition_to(new_id)
        self.context.state = new_id
        self.context.state_entry_time = self.context.now()

    # =============================================================================================
    # Override de tick — verificações globais antes do estado atual
    # =============================================================================================
    def tick(self) -> None:
        """
        Um ciclo da FSM lifecycle.

        Verificações globais executadas ANTES do estado atual:
            1. Emergência (bateria crítica) → transiciona para EMERGENCIA.

        Em seguida, executa on_enter/on_step do estado atual (via super().tick()).
        """
        # Verificação global: emergência tem prioridade sobre qualquer estado, exceto
        # quando JÁ estamos em EMERGENCIA (evita transição em loop).
        if (
            self.current_state_id != DS.EMERGENCIA
            and self.context.verifica_condicao_de_emergencia()
        ):
            self.transition_to(DS.EMERGENCIA)
            return

        super().tick()
