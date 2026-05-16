# =================================================================================================
# DeslocamentoFSMDescription — enum das fases de MANOBRA do drone (DeslocamentoFSM)
# =================================================================================================
# Pareado com a DeslocamentoFSM. Só faz sentido enquanto a DroneFSM (lifecycle) está em EM_VOO.
# =================================================================================================

from enum import IntEnum


class DeslocamentoFSMDescription(IntEnum):
    """
    Fases de manobra do drone, gerenciadas pela DeslocamentoFSM.

    Esta FSM só é tickada quando a DroneFSM está em EM_VOO. Observa o topo da
    TargetStack (no DeslocamentoFSMContext) para decidir as transições:
        - Pilha vazia ou sem target ativo → PLANANDO (hover).
        - Novo target empilhado → GIRANDO_INICIO (alinha yaw com a direção do target).
        - Yaw alinhado → DESLOCANDO (translada até o target via TrajectoryProfile).
        - Target alcançado → GIRANDO_FIM (alinha yaw final se especificado).
        - Yaw final alinhado → desempilha target e volta para PLANANDO.

    Obstáculos não criam novos estados: são tratados injetando um target de desvio
    no topo da TargetStack, o que naturalmente retorna a FSM ao GIRANDO_INICIO.
    """

    # Hover na última posição estática. Estado "idle" — aguarda um target ser empilhado.
    PLANANDO = 0

    # Girando em torno do próprio eixo para alinhar yaw com a direção do target ativo.
    GIRANDO_INICIO = 1

    # Trajetória de translação ativa: vai até o target ativo via TrajectoryProfile,
    # mantendo o yaw da direção (ou apontando para o foco, se houver).
    DESLOCANDO = 2

    # Target de posição alcançado. Gira para o yaw final do target (se especificado)
    # e, ao concluir, desempilha — devolvendo o controle ao próximo target da pilha
    # ou retornando a PLANANDO se a pilha esvaziar.
    GIRANDO_FIM = 3
