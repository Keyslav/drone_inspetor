# =================================================================================================
# enums.py
# =================================================================================================
# Enums cross-cutting que NÃO pertencem a uma FSM específica.
#
# Cada FSM mantém seu próprio `<FSM>Description` em description.py dentro do pacote da FSM:
#     DroneFSM         → drone_inspetor.nodes.drone_node.fsm.drone.description
#     DeslocamentoFSM  → drone_inspetor.nodes.drone_node.fsm.deslocamento.description
#     MissionFSM       → drone_inspetor.nodes.mission_node.fsm.mission.description
#
# Aqui ficam apenas códigos de comandos (Dashboard → MissionNode etc.) que circulam por
# múltiplas camadas e não são "estados" de nenhuma FSM.
# =================================================================================================

from enum import IntEnum


# =================================================================================================
# Comandos do Dashboard → MissionNode
# =================================================================================================
class DashboardMissionCommandDescription(IntEnum):
    """Códigos do campo `command` em DashboardMissionCommandMSG."""
    INICIAR_MISSAO = 1   # Aceito apenas em PRONTO.
    CANCELAR_MISSAO = 2  # Aceito em qualquer EXECUTANDO_*.
