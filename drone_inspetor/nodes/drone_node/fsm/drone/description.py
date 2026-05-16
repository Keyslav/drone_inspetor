# =================================================================================================
# DroneFSMDescription — enum dos estados de LIFECYCLE do drone (DroneFSM)
# =================================================================================================
# Pareado com a DroneFSM. Os valores numéricos são reservados em faixas (0-9 lifecycle,
# 9 emergência) para permitir crescimento futuro sem renumerar.
#
# Convenção:
#     - IntEnum para que `int(state)` sirva como código estável em telemetria/log.
#     - `state.name` é usado para exibição em UI e logs.
# =================================================================================================

from enum import IntEnum


class DroneFSMDescription(IntEnum):
    """
    Estados de ciclo de vida ("lifecycle") do drone, gerenciados pela DroneFSM.

    A DroneFSM responde por:
        - Verificações globais por ciclo (offboard ativo, bateria, etc.).
        - Transições disparadas por eventos do PX4 (ARMED, IN_AIR) e por comandos
          recebidos via Action DroneCommand (ARM, TAKEOFF, GOTO, LAND, RTL).

    Fases de manobra (girar, deslocar) NÃO vivem aqui — ficam na DeslocamentoFSM,
    que roda em paralelo enquanto este enum estiver em EM_VOO.
    """

    # PX4 fora do modo OFFBOARD (manual, POSCTL, AUTO_*, etc.). DroneNode não envia setpoints.
    OFFBOARD_DESATIVADO = 0

    # No solo, motores desligados. Aguarda comando ARM.
    POUSADO_DESARMADO = 1

    # No solo, motores armados (rotores girando, drone NÃO subiu). Aguarda TAKEOFF.
    POUSADO_ARMADO = 2

    # Manobra de decolagem em curso: subida vertical até altitude alvo.
    # Transiciona para EM_VOO quando altitude alvo for atingida.
    DECOLANDO = 3

    # Drone está no ar (acima do solo). A DeslocamentoFSM assume o controle das fases
    # de manobra (hover, girar, deslocar). Persiste durante GOTO/RTL/LAND comandados.
    EM_VOO = 4

    # Failsafe ativo (bateria crítica, perda de offboard catastrófica, etc.).
    # Controle delegado ao autopilot (RTL nativo do PX4). Estado absorvente.
    EMERGENCIA = 9
