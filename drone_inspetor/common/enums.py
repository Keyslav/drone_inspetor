"""
Fonte única de verdade para todos os enums e agrupamentos de estados do sistema drone_inspetor.
"""

from enum import IntEnum


# ==================================================================================================
# ESTADOS DO DRONE
# ==================================================================================================

class DroneStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis do drone.
    Usa IntEnum para comparações rápidas. Use .name para obter o nome como string.
    """
    # Estados em solo (0-9)
    POUSADO_DESARMADO = 0       # No chão, desarmado
    POUSADO_ARMADO = 1          # No chão, armado
    OFFBOARD_DESATIVADO = 2     # Fora do modo offboard (modo manual, POSCTL, etc.)

    # Estados de voo estável (10-19)
    VOANDO_PRONTO = 10          # Voando estável, aguardando comando (hover)

    # Estados de trajetória/movimento GOTO (20-29)
    VOANDO_DECOLANDO = 20       # Subindo até altitude de decolagem
    VOANDO_GIRANDO_INICIO = 21  # Girando para direção do target
    VOANDO_A_CAMINHO = 22       # Voando em direção ao target
    VOANDO_GIRANDO_FIM = 23     # Girando para yaw final
    VOANDO_GIRANDO_COM_FOCO = 25    # Girando para apontar para o focus antes de mover
    VOANDO_A_CAMINHO_COM_FOCO = 26  # Voando para target apontando para ponto de focus

    # Estados de retorno ao home RTL (30-39)
    RETORNANDO_GIRANDO_INICIO = 30  # Girando para apontar para HOME
    RETORNANDO_A_CAMINHO = 31       # Voando em direção ao HOME
    RETORNANDO_GIRANDO_FIM = 32     # Girando para yaw final do HOME

    # Estado de pouso (40)
    POUSANDO = 40               # Descendo para pousar (usado por RTL e LAND)

    # Estados de desvio de obstáculo - GOTO (50-54)
    VOANDO_A_CAMINHO_OBSTACULO = 50                  # Detectou obstáculo, calcula desvio
    VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO = 51   # Girando p/ direção do desvio
    VOANDO_A_CAMINHO_OBSTACULO_DESVIANDO = 52        # Voando em direção ao desvio
    VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM = 53      # Girando p/ direção do destino original
    VOANDO_A_CAMINHO_OBSTACULO_DESVIADO = 54         # Verifica se obstáculo sumiu p/ retomar rota

    # Estados de desvio de obstáculo - GOTO COM FOCO (55-59)
    VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO = 55                  # Detectou obstáculo, calcula desvio
    VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_INICIO = 56   # Girando p/ direção do desvio
    VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIANDO = 57        # Voando em direção ao desvio
    VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_FIM = 58      # Girando p/ direção do destino original
    VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIADO = 59         # Verifica se obstáculo sumiu p/ retomar rota

    # Estados de desvio de obstáculo - RTL (60-64)
    RETORNANDO_A_CAMINHO_OBSTACULO = 60                  # Detectou obstáculo, calcula desvio
    RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO = 61   # Girando p/ direção do desvio
    RETORNANDO_A_CAMINHO_OBSTACULO_DESVIANDO = 62        # Voando em direção ao desvio
    RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM = 63      # Girando p/ direção do HOME (destino original)
    RETORNANDO_A_CAMINHO_OBSTACULO_DESVIADO = 64         # Verifica se obstáculo sumiu p/ retomar rota

    # Emergência (99)
    EMERGENCIA = 99             # Failsafe/emergência ativa


# ==================================================================================================
# ESTADOS DA MÁQUINA DE ESTADOS DE MISSÃO
# ==================================================================================================

class MissionStateDescription(IntEnum):
    """
    Enum unificado que representa todos os estados possíveis da máquina de missão.
    """
    # Estados de nível 0 - Sistema
    DESATIVADO = 0                                          # Sistema desativado, aguardando tópicos essenciais
    PRONTO = 1                                              # Pronto para iniciar missão

    # Estados EXECUTANDO_MISSAO - Nível 1
    EXECUTANDO_ARMANDO = 10                                 # Armando motores
    EXECUTANDO_DECOLANDO = 11                               # Decolagem em progresso
    EXECUTANDO_INSPECIONANDO = 12                            # Percorrendo pontos de inspeção

    # Estados INSPECIONANDO - Nível 2
    EXECUTANDO_INSPECIONANDO_DETECTANDO = 20                 # Aguardando detecção do objeto alvo
    EXECUTANDO_INSPECIONANDO_ESCANEANDO = 22                 # Escaneando anomalias (detecção habilitada)
    EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO = 23    # Finalizando escaneamento
    EXECUTANDO_INSPECIONANDO_FALHA = 25                      # Falha na detecção do target

    # Estados de finalização
    INSPECAO_FINALIZADA = 30                                 # Pausa pós-inspeção
    RETORNANDO = 40                                          # Retornando ao home (RTL)


# ==================================================================================================
# COMANDOS DO DASHBOARD PARA MISSION NODE
# ==================================================================================================

class DashboardMissionCommandDescription(IntEnum):
    """
    Enum que representa os comandos enviados do Dashboard para o Mission Node.
    Os valores inteiros correspondem ao campo 'command' de DashboardMissionCommandMSG.
    """
    INICIAR_MISSAO = 1      # Iniciar missão (aceito apenas em PRONTO)
    CANCELAR_MISSAO = 2     # Cancelar missão e retornar (aceito em EXECUTANDO_*)


# ==================================================================================================
# AGRUPAMENTO DE ESTADOS DO DRONE
# ==================================================================================================

# Estados de movimento GOTO (destino simples)
DRONE_STATES_GOTO = [
    DroneStateDescription.VOANDO_GIRANDO_INICIO,
    DroneStateDescription.VOANDO_A_CAMINHO,
    DroneStateDescription.VOANDO_GIRANDO_FIM,
    DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO,
    DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO,
    DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_DESVIANDO,
    DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM,
    DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_DESVIADO,
]

# Estados de movimento GOTO com foco (subgrafo interno: GOTO com use_focus=True)
DRONE_STATES_GOTO_COM_FOCO = [
    DroneStateDescription.VOANDO_GIRANDO_COM_FOCO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_INICIO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIANDO,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_FIM,
    DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIADO,
]

# Estados de retorno RTL
DRONE_STATES_RTL = [
    DroneStateDescription.RETORNANDO_GIRANDO_INICIO,
    DroneStateDescription.RETORNANDO_A_CAMINHO,
    DroneStateDescription.RETORNANDO_GIRANDO_FIM,
    DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO,
    DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO,
    DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_DESVIANDO,
    DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM,
    DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_DESVIADO,
]

# Estados de pouso
DRONE_STATES_POUSANDO = [DroneStateDescription.POUSANDO]

# Estados pousado
DRONE_STATES_POUSADO = [
    DroneStateDescription.POUSADO_DESARMADO,
    DroneStateDescription.POUSADO_ARMADO,
]

# Todos os estados de movimento ativo (para verificar se drone está em trânsito)
DRONE_STATES_EM_MOVIMENTO = DRONE_STATES_GOTO + DRONE_STATES_GOTO_COM_FOCO + DRONE_STATES_RTL
