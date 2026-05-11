"""
Classe base genérica para máquinas de estado (State Pattern).

Usada tanto pela FSM interna do drone_node quanto pela FSM de missão do mission_node.
Cada estado concreto herda de State e sobrescreve on_enter/on_step/on_exit conforme necessário.
"""

from typing import Any


class State:
    """
    Estado base de uma máquina de estados.

    Subclasses sobrescrevem on_enter, on_step e on_exit.
    Mesmo que vazios, os três métodos devem existir na interface.
    """

    def __init__(self, context: Any, node: Any):
        """
        Args:
            context: Contexto compartilhado (dados acessíveis por todos os estados).
            node: Referência ao nó ROS2 (para logger, clock, comandos PX4, etc.).
        """
        self.context = context
        self.node = node

    def on_enter(self) -> None:
        """Chamado UMA VEZ quando o estado é ativado (primeiro ciclo)."""
        pass

    def on_step(self) -> 'Any | None':
        """
        Chamado a cada ciclo após on_enter.

        Returns:
            Identificador do próximo estado para transicionar,
            ou None para permanecer no estado atual.
        """
        return None

    def on_exit(self) -> None:
        """Chamado UMA VEZ ao sair do estado (antes de entrar no próximo)."""
        pass


class StateMachine:
    """
    Orquestrador genérico de máquinas de estado.

    Gerencia registro de estados, transições (com on_exit → on_enter)
    e ciclo tick (on_enter no primeiro ciclo, on_step nos seguintes).
    """

    def __init__(self):
        self._states: dict = {}
        self._current_state_id = None
        self._current_state: State | None = None
        self._entered = False

    @property
    def current_state_id(self):
        return self._current_state_id

    @property
    def current_state(self) -> State | None:
        return self._current_state

    def register(self, state_id, state: State):
        """Registra um estado com seu identificador (enum, str, int, etc.)."""
        self._states[state_id] = state

    def transition_to(self, new_id) -> None:
        """
        Transiciona para um novo estado.
        Chama on_exit do estado atual (se houver) e marca on_enter como pendente.
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
        Executa um ciclo da máquina.
        Primeiro ciclo após transição: on_enter. Ciclos seguintes: on_step.
        Se on_step retorna um state_id diferente do atual, transiciona.
        """
        if self._current_state is None:
            return
        if not self._entered:
            self._current_state.on_enter()
            self._entered = True
        next_id = self._current_state.on_step()
        if next_id is not None and next_id != self._current_state_id:
            self.transition_to(next_id)
