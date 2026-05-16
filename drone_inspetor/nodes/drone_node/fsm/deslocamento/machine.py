# =================================================================================================
# DeslocamentoFSM — máquina de estados de fases de manobra
# =================================================================================================
# Herda BaseStateMachine. Tickada APENAS enquanto a DroneFSM (lifecycle) está em EM_VOO.
# Coordenação: o DroneNode chama `self.deslocamento_fsm.tick()` em seu loop de 50 Hz
# somente se `self.drone_fsm.current_state_id == DroneFSMDescription.EM_VOO`.
#
# Espelha automaticamente o estado atual em `context.deslocamento_state` em cada transição,
# para consumo por código externo (publishers de telemetria, GUI, mission_node).
# =================================================================================================

from drone_inspetor.base_classes.base_state_machine import BaseStateMachine
from drone_inspetor.nodes.drone_node.fsm.deslocamento.description import DeslocamentoFSMDescription as TS
from drone_inspetor.nodes.drone_node.fsm.deslocamento.context import DeslocamentoFSMContext
from drone_inspetor.nodes.drone_node.fsm.deslocamento.states.planando import PlanandoState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.states.girando_inicio import GirandoInicioState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.states.deslocando import DeslocandoState
from drone_inspetor.nodes.drone_node.fsm.deslocamento.states.girando_fim import GirandoFimState


class DeslocamentoFSM(BaseStateMachine):
    """
    FSM secundária do DroneNode — gerencia as fases de manobra em voo.

    Estados:
        PLANANDO (hover idle), GIRANDO_INICIO, DESLOCANDO, GIRANDO_FIM.

    Acoplamento com a DroneFSM:
        - Só é tickada enquanto a DroneFSM está em EM_VOO (controle no DroneNode).
        - Não trata eventos de lifecycle (decolagem, pouso, emergência) — esses ficam
          com a DroneFSM, que pode resetar a DeslocamentoFSM ao sair de EM_VOO.

    Decisões da FSM consomem dois insumos:
        - context.target_stack: pilha de destinos (MISSAO + DESVIO encadeados).
        - context.state_px4:    telemetria atual do drone (posição, yaw).
    """

    def __init__(self, context: DeslocamentoFSMContext, node):
        super().__init__()
        # Mesmo contexto compartilhado com a DroneFSM e os mixins do nó.
        self.context = context
        self.node = node

    # =============================================================================================
    # Registro dos estados
    # =============================================================================================
    def register_all_states(self) -> None:
        """Registra os 4 estados. Chamado uma vez no startup do DroneNode."""
        self.register(TS.PLANANDO,       PlanandoState(self.context, self.node))
        self.register(TS.GIRANDO_INICIO, GirandoInicioState(self.context, self.node))
        self.register(TS.DESLOCANDO,     DeslocandoState(self.context, self.node))
        self.register(TS.GIRANDO_FIM,    GirandoFimState(self.context, self.node))

    # =============================================================================================
    # Override de transition_to — espelha o estado no contexto
    # =============================================================================================
    def transition_to(self, new_id) -> None:
        """
        Realiza a transição e atualiza `context.deslocamento_state`.

        Loga apenas em DEBUG: a DeslocamentoFSM transita com frequência (cada manobra
        passa por 4 estados), então loga em INFO encheria o terminal.
        """
        if new_id == self.current_state_id:
            return
        prev_name = self.current_state_id.name if self.current_state_id is not None else "—"
        new_name = new_id.name
        self.node.get_logger().debug(f"[DeslocamentoFSM] {prev_name} → {new_name}")
        super().transition_to(new_id)
        self.context.state = new_id

    # =============================================================================================
    # Reset programático (chamado pelo DroneNode ao sair de EM_VOO)
    # =============================================================================================
    def reset_to_planando(self) -> None:
        """
        Força a FSM ao estado PLANANDO, descartando manobras em curso.

        Chamado pelo DroneNode quando a DroneFSM sai de EM_VOO (pouso, OFFBOARD_DESATIVADO,
        EMERGENCIA): a DeslocamentoFSM precisa abandonar o estado atual de forma limpa
        para não retomar uma manobra obsoleta quando voltarmos a EM_VOO.
        """
        self.transition_to(TS.PLANANDO)
