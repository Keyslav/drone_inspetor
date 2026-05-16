# =================================================================================================
# BaseState
# =================================================================================================
# Template para estados de máquinas de estado (State Pattern).
#
# Classe abstrata: existe apenas para ser herdada. Subclasses concretas em cada FSM
# (DroneFSM, DeslocamentoFSM, MissionFSM, ...) sobrescrevem on_enter, on_step e on_exit.
# O prefixo `Base` sinaliza que a classe não deve ser instanciada diretamente.
# =================================================================================================

from typing import Any


class BaseState:
    """
    Template de estado de uma máquina de estados.

    Interface fixa (não estender):
        - on_enter(): executa uma única vez ao entrar no estado.
        - on_step(): executa a cada ciclo; pode retornar identificador de próximo estado.
        - on_exit(): executa uma única vez ao sair do estado.

    Cada estado concreto recebe:
        - context: objeto que carrega TODAS as variáveis compartilhadas entre estados
                   da mesma FSM (ex.: DroneNodeContext para a DroneFSM/DeslocamentoFSM).
        - node:    referência ao nó ROS2 dono da FSM (logger, clock, publishers).
    """

    def __init__(self, context: Any, node: Any):
        """
        Args:
            context: Contexto compartilhado da FSM (ex.: DroneNodeContext).
            node:    Nó ROS2 hospedeiro (para logger/clock/publishers).
        """
        self.context = context
        self.node = node

    def on_enter(self) -> None:
        """Chamado UMA VEZ quando o estado é ativado (antes do primeiro on_step)."""
        pass

    def on_step(self) -> 'Any | None':
        """
        Chamado a cada tick da FSM enquanto este for o estado atual.

        Returns:
            Identificador (geralmente um membro de IntEnum) do próximo estado para o qual
            transicionar, ou None para permanecer no estado atual.
        """
        return None

    def on_exit(self) -> None:
        """Chamado UMA VEZ ao sair do estado (antes do on_enter do próximo)."""
        pass
