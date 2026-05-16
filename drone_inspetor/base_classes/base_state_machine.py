# =================================================================================================
# BaseStateMachine
# =================================================================================================
# Template para máquinas de estado (orquestrador do State Pattern).
#
# Classe abstrata: existe apenas para ser herdada. Subclasses concretas (DroneFSM, DeslocamentoFSM,
# MissionFSM) tipicamente sobrescrevem `tick` para inserir verificações globais por ciclo
# (perda de offboard, emergência, etc.) ANTES de delegar ao estado atual.
# O prefixo `Base` sinaliza que a classe não deve ser instanciada diretamente.
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState


class BaseStateMachine:
    """
    Template de máquina de estados finita.

    Responsabilidades:
        - Registrar estados com identificadores (enum, str ou int).
        - Conduzir transições (chama on_exit do antigo → marca on_enter do novo).
        - Executar o ciclo tick: on_enter no primeiro ciclo após uma transição,
          on_step nos ciclos subsequentes.

    Subclasses concretas costumam:
        - Adicionar lógica global de pré-tick em `tick()` (override).
        - Expor método `register_all_states()` para popular `_states` no construtor.
    """

    def __init__(self):
        # Mapeamento {identificador → instância de BaseState}.
        self._states: dict = {}
        # Identificador do estado atualmente ativo (None até a primeira transição).
        self._current_state_id = None
        # Instância do estado atualmente ativo (None até a primeira transição).
        self._current_state: BaseState | None = None
        # Flag que controla se on_enter já foi executado no estado atual.
        self._entered = False

    @property
    def current_state_id(self):
        """Identificador (enum/str/int) do estado atual."""
        return self._current_state_id

    @property
    def current_state(self) -> BaseState | None:
        """Instância do estado atual (None se ainda não inicializado)."""
        return self._current_state

    def register(self, state_id, state: BaseState) -> None:
        """
        Registra um estado no mapa interno.

        Args:
            state_id: Identificador (tipicamente membro de um IntEnum).
            state:    Instância concreta de BaseState.
        """
        self._states[state_id] = state

    def transition_to(self, new_id) -> None:
        """
        Realiza a transição para um novo estado.

        Fluxo:
            1. Se já estamos no estado solicitado, retorna sem efeito.
            2. Chama on_exit do estado atual (se houver).
            3. Troca o ponteiro do estado atual.
            4. Marca on_enter como pendente — será executado no próximo tick.

        Args:
            new_id: Identificador do estado destino (deve estar registrado).
        """
        if new_id == self._current_state_id:
            return
        if self._current_state is not None:
            self._current_state.on_exit()
        self._current_state_id = new_id
        self._current_state = self._states[new_id]
        self._entered = False

    def tick(self) -> None:
        """
        Executa um ciclo da FSM.

        Comportamento:
            - Primeiro tick após uma transição: executa on_enter() do estado novo.
            - Ticks subsequentes: executa on_step().
            - Se on_step retorna um identificador diferente do atual, transiciona.
        """
        if self._current_state is None:
            return
        if not self._entered:
            self._current_state.on_enter()
            self._entered = True
        next_id = self._current_state.on_step()
        if next_id is not None and next_id != self._current_state_id:
            self.transition_to(next_id)
