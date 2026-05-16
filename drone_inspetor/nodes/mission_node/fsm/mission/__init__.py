# =================================================================================================
# MissionFSM — máquina de estados de MISSÃO do MissionNode
# =================================================================================================
# Gerencia a execução de uma missão em três níveis (sistema, execução, sub-fases de inspeção).
#
# Conteúdo do pacote:
#   - machine.py      → classe MissionFSM (herda BaseStateMachine).
#   - context.py      → classe MissionFSMContext (dados da missão + publish + validação).
#   - description.py  → classe MissionFSMDescription (enum dos estados).
#   - states/         → um arquivo por estado concreto.
# =================================================================================================
