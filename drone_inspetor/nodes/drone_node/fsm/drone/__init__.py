# =================================================================================================
# DroneFSM — máquina de estados de LIFECYCLE do DroneNode
# =================================================================================================
# Gerencia apenas o ciclo de vida do drone (no solo, decolando, no ar, emergência).
# As FASES de manobra (girar, deslocar) vivem na DeslocamentoFSM (pacote irmão
# `nodes/drone_node/fsm/deslocamento`), que roda em paralelo enquanto a DroneFSM
# está em EM_VOO.
#
# Conteúdo do pacote:
#   - machine.py      → classe DroneFSM (herda BaseStateMachine).
#   - context.py      → classe DroneFSMContext (variáveis da FSM).
#   - description.py  → classe DroneFSMDescription (enum dos 6 estados).
#   - states/         → um arquivo por estado concreto.
# =================================================================================================
