# =================================================================================================
# Pacote `fsm` do MissionNode
# =================================================================================================
# Contém uma subpasta por FSM hospedada no MissionNode. Por enquanto há apenas:
#
#   mission/ — MissionFSM (estado da missão em três níveis: sistema, execução, sub-fases).
#
# Cada subpasta segue o mesmo layout:
#   machine.py        — classe da FSM (subclasse de BaseStateMachine).
#   context.py        — classe FSMContext (variáveis compartilhadas entre os estados).
#   description.py    — classe FSMDescription (enum dos estados).
#   states/           — um arquivo por estado concreto da FSM.
# =================================================================================================
