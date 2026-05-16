# =================================================================================================
# DeslocamentoFSM — FSM de fases de MANOBRA do DroneNode
# =================================================================================================
# Tickada APENAS enquanto a DroneFSM (lifecycle) está em EM_VOO. Gerencia 4 estados:
#     PLANANDO       → hover (idle, aguardando target).
#     GIRANDO_INICIO → girando para alinhar yaw com a direção do target.
#     DESLOCANDO     → transladando até o target via TrajectoryProfile.
#     GIRANDO_FIM    → alinhando yaw final no destino.
#
# Conteúdo do pacote:
#   - machine.py      → classe DeslocamentoFSM (herda BaseStateMachine).
#   - context.py      → classe DeslocamentoFSMContext (variáveis da FSM).
#   - description.py  → classe DeslocamentoFSMDescription (enum).
#   - states/         → um arquivo por estado concreto.
#
# Helpers geométricos puros usados pelo estado DESLOCANDO ficam fora da FSM
# (em `nodes/drone_node/obstacle_avoidance.py`), já que são utilitários do nó
# e não pertencem semanticamente a essa máquina de estados.
# =================================================================================================
