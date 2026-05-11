# machine.py
# =================================================================================================
# MÁQUINA DE ESTADOS FSM DO DRONE (REFATORADA)
# =================================================================================================
# DroneFSM estende StateMachine com verificações globais executadas a cada ciclo
# antes de delegar ao estado atual: offboard, emergência, target impossível e desarmamento.
# =================================================================================================

# Importação para permitir anotações de tipo sem causar dependência circular em tempo de execução
from typing import TYPE_CHECKING

# Importa a mensagem VehicleStatus do PX4, que contém constantes sobre o estado de navegação
from px4_msgs.msg import VehicleStatus

# Importa a classe base da Máquina de Estados e o enum com todos os estados possíveis
from drone_inspetor.common.state import StateMachine
from drone_inspetor.common.enums import DroneStateDescription

# Se estiver apenas checando tipos, importa DroneFSMContext
if TYPE_CHECKING:
    from drone_inspetor.nodes.drone_node.fsm.context import DroneFSMContext


class DroneFSM(StateMachine):
    """
    Máquina de estados do drone com verificações globais pré-ciclo.

    Estende a classe base StateMachine adicionando funcionalidades essenciais:
    - Verificações globais antes de cada ciclo de atualização (tick), cobrindo:
      perda de modo offboard, condições de emergência, alvos impossíveis de alcançar
      e desarmamento inesperado.
    - Atualização do estado compartilhado (drone_context.state) e do tempo de entrada
      no estado (drone_context.state_entry_time) a cada transição.
    - Log padronizado de todas as transições de estado para facilitar o debug.
    - Método register_all_states() para inicializar e registrar os 14 estados concretos do drone.
    """

    def __init__(self, drone_context: "DroneFSMContext", node):
        """
        Args:
            drone_context: Contexto compartilhado da FSM (DroneFSMContext criado em DroneNode.__init__).
                           Carrega estado do PX4, alvos, pilha de waypoints, obstáculos, etc.
            node: Instância do DroneNode (rclpy.node.Node) para acesso a logger e relógio.
        """
        # Inicializa a classe base StateMachine
        super().__init__()
        # Armazena o contexto do drone, que guarda informações de voo, targets e obstáculos
        self.drone_context = drone_context
        # Armazena a referência para o nó ROS 2 para usar logs e medir o tempo
        self.node = node
        # Registra todos os estados da máquina logo na inicialização
        self.register_all_states()
        # Define o estado inicial da máquina como OFFBOARD_DESATIVADO
        self.transition_to(DroneStateDescription.OFFBOARD_DESATIVADO)

    # ------------------------------------------------------------------
    # Transição com efeitos colaterais no contexto
    # ------------------------------------------------------------------

    def transition_to(self, new_id) -> None:
        """
        Transiciona para um novo estado, atualizando drone_context.state,
        drone_context.state_entry_time e registrando o evento no logger.
        """
        # Se o estado de destino for igual ao atual, não faz nada para evitar transições desnecessárias
        if new_id == self._current_state_id:
            return
        
        # Pega o nome do estado atual (ou "NONE" se for a primeira inicialização)
        old_name = self._current_state_id.name if self._current_state_id is not None else "NONE"
        
        # Chama a implementação da classe base para lidar com a lógica interna da transição
        super().transition_to(new_id)
        
        # Atualiza a referência de qual é o estado atual no contexto compartilhado
        self.drone_context.state = new_id
        # Registra o instante de tempo em que o estado mudou, útil para timeouts e lógicas de tempo
        self.drone_context.state_entry_time = self.drone_context.now()
        
        # Imprime no log do ROS uma linha separadora para chamar atenção no terminal
        self.node.get_logger().info(
            "-----------------------------------------------------------------------------------"
        )
        # Imprime a mudança de estado para que possamos rastrear no terminal
        self.node.get_logger().info(
            f">>> FSM Transição: {old_name} -> {new_id.name}"
        )

    # ------------------------------------------------------------------
    # Tick com verificações globais
    # ------------------------------------------------------------------

    def tick(self) -> None:
        """
        Executa um ciclo da Máquina de Estados (FSM).

        Antes de processar o estado atual (delegando para o tick() do estado),
        esta função aplica verificações globais em ordem de prioridade.
        Se qualquer uma dessas verificações disparar uma transição para um novo estado,
        o processamento do estado atual é omitido neste ciclo específico, garantindo
        uma resposta rápida a condições críticas (ex: perda de offboard ou emergência).
        """
        # Armazena qual estado está ativo no início deste tick para saber se ele mudou pelas verificações globais
        state_before = self._current_state_id

        # --- 1. Verificação de modo Offboard ---
        # Garante que o drone só execute operações complexas se estiver no modo correto.
        # Atalho para facilitar o acesso às informações recebidas do PX4
        state_px4 = self.drone_context.state_px4  
        current = self._current_state_id
        
        # Se não estiver em modo offboard nem pouso automático e também já não estiver desativado ou pousando:
        if (
            state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and state_px4.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            and current != DroneStateDescription.OFFBOARD_DESATIVADO
            and current != DroneStateDescription.POUSANDO
        ):
            # Avisa no terminal que o modo offboard foi perdido inesperadamente
            self.node.get_logger().error(
                f"ENTROU OFFBOARD_DESATIVADO: state_px4.nav_state = {state_px4.get_nav_state_name()} "
                f"e drone_state = {current.name if current else 'NONE'}"
            )
            # Reseta os cálculos de trajetória e foca em desativar o drone com segurança
            self.drone_context.reset_trajectory_vars()
            self.transition_to(DroneStateDescription.OFFBOARD_DESATIVADO)

        # --- 2. Verificação de emergência ---
        # Checa condições críticas da bateria ou de hardware que exijam ação imediata.
        # Executada SEMPRE, exceto quando o drone já está em OFFBOARD_DESATIVADO
        # (controle já delegado ao PX4 — não faz sentido acionar emergência).
        # Ação depende do estado do drone:
        #   - Em voo:           envia RTL nativo ao PX4 (autopilot retorna e pousa)
        #   - POUSADO_ARMADO:   desarma (não envia RTL — não faz sentido decolar)
        #   - POUSADO_DESARMADO: nada a fazer (já seguro)
        # Em todos os casos a FSM vai para OFFBOARD_DESATIVADO.
        if self._current_state_id != DroneStateDescription.OFFBOARD_DESATIVADO:
            if self.drone_context.verifica_condicao_de_emergencia():
                current = self._current_state_id
                if current == DroneStateDescription.POUSADO_DESARMADO:
                    self.node.get_logger().error(
                        "EMERGÊNCIA detectada: drone já pousado e desarmado. Desativando Offboard."
                    )
                elif current == DroneStateDescription.POUSADO_ARMADO:
                    self.node.get_logger().error(
                        "EMERGÊNCIA detectada: drone pousado e armado. Desarmando e desativando Offboard."
                    )
                    self.node.disarm()
                else:
                    self.node.get_logger().error(
                        "EMERGÊNCIA detectada: drone em voo. Enviando RTL nativo ao PX4 e desativando Offboard."
                    )
                    self.node.rtl_native()
                self.drone_context.reset_trajectory_vars()
                self.transition_to(DroneStateDescription.OFFBOARD_DESATIVADO)

        # --- 3. Verificação de desarmamento ---
        # Proteção para garantir que o sistema não tente voar se as hélices pararem inesperadamente.
        if (
            self._current_state_id == state_before # Nenhuma verificação anterior disparou mudança
            and not state_px4.is_armed       # Os motores foram desligados (desarmado)
            and self._current_state_id != DroneStateDescription.POUSADO_DESARMADO # Já não está neste estado de espera
            and self._current_state_id != DroneStateDescription.OFFBOARD_DESATIVADO # Já não está inativo
        ):
            # Limpa trajetórias pendentes e muda para pousado desarmado imediatamente, abortando a missão
            self.drone_context.reset_trajectory_vars()
            self.transition_to(DroneStateDescription.POUSADO_DESARMADO)

        # --- Processa estado atual apenas se nenhuma verificação global causou uma transição ---
        # Se nenhuma das 3 checagens acima fez um self.transition_to(), o estado é mantido.
        if self._current_state_id == state_before:
            # Chama o método tick() da StateMachine, que por sua vez chama o tick() do estado ativo
            super().tick()

    # ------------------------------------------------------------------
    # Registro de todos os estados
    # ------------------------------------------------------------------

    def register_all_states(self) -> None:
        """Importa e registra todos os 14 estados concretos da FSM."""
        
        # Importações feitas internamente ao método para evitar dependência circular pesada
        # (já que esses estados geralmente importam o Contexto e a própria Máquina)
        from drone_inspetor.nodes.drone_node.fsm.states.offboard_desativado import (
            OffboardDesativadoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.pousado_desarmado import (
            PousadoDesarmadoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.pousado_armado import (
            PousadoArmadoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_pronto import (
            VoandoProntoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_decolando import (
            VoandoDecolandoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_girando_inicio import (
            VoandoGirandoInicioState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho import (
            VoandoACaminhoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_girando_fim import (
            VoandoGirandoFimState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_girando_com_foco import (
            VoandoGirandoComFocoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco import (
            VoandoACaminhoComFocoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_girando_inicio import (
            RetornandoGirandoInicioState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho import (
            RetornandoACaminhoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_girando_fim import (
            RetornandoGirandoFimState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.pousando import PousandoState

        # Estados de desvio de obstáculo (sub-FSM, 5 fases × 3 fluxos = 15 estados)
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_obstaculo import (
            VoandoACaminhoObstaculoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_obstaculo_girando_inicio import (
            VoandoACaminhoObstaculoGirandoInicioState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_obstaculo_desviando import (
            VoandoACaminhoObstaculoDesviandoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_obstaculo_girando_fim import (
            VoandoACaminhoObstaculoGirandoFimState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_obstaculo_desviado import (
            VoandoACaminhoObstaculoDesviadoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco_obstaculo import (
            VoandoACaminhoComFocoObstaculoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco_obstaculo_girando_inicio import (
            VoandoACaminhoComFocoObstaculoGirandoInicioState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco_obstaculo_desviando import (
            VoandoACaminhoComFocoObstaculoDesviandoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco_obstaculo_girando_fim import (
            VoandoACaminhoComFocoObstaculoGirandoFimState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.voando_a_caminho_com_foco_obstaculo_desviado import (
            VoandoACaminhoComFocoObstaculoDesviadoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho_obstaculo import (
            RetornandoACaminhoObstaculoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho_obstaculo_girando_inicio import (
            RetornandoACaminhoObstaculoGirandoInicioState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho_obstaculo_desviando import (
            RetornandoACaminhoObstaculoDesviandoState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho_obstaculo_girando_fim import (
            RetornandoACaminhoObstaculoGirandoFimState,
        )
        from drone_inspetor.nodes.drone_node.fsm.states.retornando_a_caminho_obstaculo_desviado import (
            RetornandoACaminhoObstaculoDesviadoState,
        )

        # Registra cada um dos estados atrelando a chave Enum (ID) à instância da classe,
        # passando o contexto compartilhado e o nó ROS como parâmetros.
        self.register(DroneStateDescription.OFFBOARD_DESATIVADO, OffboardDesativadoState(self.drone_context, self.node))
        self.register(DroneStateDescription.POUSADO_DESARMADO, PousadoDesarmadoState(self.drone_context, self.node))
        self.register(DroneStateDescription.POUSADO_ARMADO, PousadoArmadoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_PRONTO, VoandoProntoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_DECOLANDO, VoandoDecolandoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_GIRANDO_INICIO, VoandoGirandoInicioState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO, VoandoACaminhoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_GIRANDO_FIM, VoandoGirandoFimState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_GIRANDO_COM_FOCO, VoandoGirandoComFocoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO, VoandoACaminhoComFocoState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_GIRANDO_INICIO, RetornandoGirandoInicioState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO, RetornandoACaminhoState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_GIRANDO_FIM, RetornandoGirandoFimState(self.drone_context, self.node))
        self.register(DroneStateDescription.POUSANDO, PousandoState(self.drone_context, self.node))

        # Sub-FSM de desvio de obstáculo
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO, VoandoACaminhoObstaculoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO, VoandoACaminhoObstaculoGirandoInicioState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_DESVIANDO, VoandoACaminhoObstaculoDesviandoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM, VoandoACaminhoObstaculoGirandoFimState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_OBSTACULO_DESVIADO, VoandoACaminhoObstaculoDesviadoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO, VoandoACaminhoComFocoObstaculoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_INICIO, VoandoACaminhoComFocoObstaculoGirandoInicioState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIANDO, VoandoACaminhoComFocoObstaculoDesviandoState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_GIRANDO_FIM, VoandoACaminhoComFocoObstaculoGirandoFimState(self.drone_context, self.node))
        self.register(DroneStateDescription.VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO_DESVIADO, VoandoACaminhoComFocoObstaculoDesviadoState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO, RetornandoACaminhoObstaculoState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_INICIO, RetornandoACaminhoObstaculoGirandoInicioState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_DESVIANDO, RetornandoACaminhoObstaculoDesviandoState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_GIRANDO_FIM, RetornandoACaminhoObstaculoGirandoFimState(self.drone_context, self.node))
        self.register(DroneStateDescription.RETORNANDO_A_CAMINHO_OBSTACULO_DESVIADO, RetornandoACaminhoObstaculoDesviadoState(self.drone_context, self.node))
