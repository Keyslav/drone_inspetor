# =================================================================================================
# Pacote `fsm` do DroneNode
# =================================================================================================
# Contém uma subpasta por FSM hospedada no DroneNode:
#
#   drone/        — DroneFSM (lifecycle do drone)
#   deslocamento/ — DeslocamentoFSM (fases de manobra em voo)
#
# Cada subpasta segue o mesmo layout:
#   machine.py        — classe da FSM (subclasse de BaseStateMachine).
#   context.py        — classe FSMContext (variáveis compartilhadas entre os estados).
#   description.py    — classe FSMDescription (enum dos estados).
#   states/           — um arquivo por estado concreto da FSM.
# =================================================================================================
