# =================================================================================================
# MissionFSMDescription — enum dos estados da máquina de MISSÃO (MissionFSM)
# =================================================================================================
# Pareado com a MissionFSM (no MissionNode). Faixas numéricas:
#     0-9    → sistema (DESATIVADO, PRONTO)
#     10-19  → execução de nível 1 (ARMANDO/DECOLANDO/INSPECIONANDO)
#     20-29  → sub-fases de inspeção (nível 2)
#     30-49  → finalização (INSPECAO_FINALIZADA, RETORNANDO)
# =================================================================================================

from enum import IntEnum


class MissionFSMDescription(IntEnum):
    """
    Estados da máquina de missão (MissionNode).

    Hierarquia em três níveis:
        Nível 0 — sistema: DESATIVADO, PRONTO.
        Nível 1 — execução: EXECUTANDO_ARMANDO, EXECUTANDO_DECOLANDO, EXECUTANDO_INSPECIONANDO.
        Nível 2 — sub-fases da inspeção: detectando, escaneando, finalizando, falha.
    """

    # Nível 0 — sistema
    DESATIVADO = 0   # Aguardando tópicos essenciais (telemetria, etc.).
    PRONTO = 1       # Pronto para iniciar missão.

    # Nível 1 — execução da missão
    EXECUTANDO_ARMANDO = 10
    EXECUTANDO_DECOLANDO = 11
    EXECUTANDO_INSPECIONANDO = 12

    # Nível 2 — sub-fases de inspeção
    EXECUTANDO_INSPECIONANDO_DETECTANDO = 20
    EXECUTANDO_INSPECIONANDO_ESCANEANDO = 22
    EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO = 23
    EXECUTANDO_INSPECIONANDO_FALHA = 25

    # Finalização
    INSPECAO_FINALIZADA = 30
    RETORNANDO = 40
