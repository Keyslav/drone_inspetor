# retornando.py
# =================================================================================================
# ESTADO: RETORNANDO
# =================================================================================================
# Envia comando RTL ao drone e aguarda o ciclo completo: voo de retorno → pouso → desarme.
# Transiciona para PRONTO após o drone reportar POUSADO_DESARMADO.
# =================================================================================================
#
# Na arquitetura nova da DroneFSM, todo o trajeto RTL (giro inicial + translação + pouso) ocorre
# enquanto o drone está em EM_VOO. As fases internas (girar, deslocar) são responsabilidade
# da DeslocamentoFSM do drone_node, transparente ao mission_node. Por isso aqui basta observar
# apenas três estados externos: EM_VOO (em retorno), POUSADO_ARMADO (recém-pousou) e
# POUSADO_DESARMADO (terminou).
# =================================================================================================

from drone_inspetor.base_classes.base_state import BaseState
from drone_inspetor.nodes.mission_node.fsm.mission.description import MissionFSMDescription as MS
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS


class RetornandoState(BaseState):
    """
    Retorno ao ponto de origem via RTL. Aguarda pouso e desarmamento para ir a PRONTO.
    """

    def on_step(self):
        drone_state = self.node.drone.state

        # Aguarda action de RTL em andamento
        if self.node._action_in_progress:
            self.node.get_logger().info(
                "Aguardando conclusão do RTL...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone pousou e desarmou — missão encerrada com sucesso
        if drone_state == DS.POUSADO_DESARMADO:
            self.node.get_logger().info(
                "Drone pousado e desarmado. Retorno concluído com sucesso!"
            )
            self.context.reset()
            return MS.PRONTO

        # Drone pousado mas ainda armado — aguarda desarmamento automático
        if drone_state == DS.POUSADO_ARMADO:
            self.node.get_logger().info(
                "Drone pousado e armado. Aguardando desarmamento automático...",
                throttle_duration_sec=3.0,
            )
            return None

        # Drone ainda no ar (em retorno OU em pouso final, ambos cobertos por EM_VOO).
        # A DeslocamentoFSM do drone_node cuida das fases internas — daqui só monitoramos.
        if drone_state == DS.EM_VOO:
            self.node.get_logger().info(
                "Drone retornando ao home...",
                throttle_duration_sec=3.0,
            )

            # Se ainda não enviamos o RTL, manda agora. send_drone_action() é idempotente
            # (já bloqueia comando duplicado via _action_in_progress no MissionNode).
            if not self.node._action_in_progress:
                self.node.get_logger().info("Enviando comando RTL.")
                self.node.send_drone_action({"command": "RTL"})
            return None

        self.node.get_logger().warn(
            f"Estado inesperado durante retorno: {drone_state.name}."
        )
        return None
