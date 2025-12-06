"""
fsm.py
=================================================================================================
Gerenciador da interface visual da Máquina de Estados Finitos (FSM).

Gerencia a interface visual da Máquina de Estados Finitos (FSM) do sistema. Esta classe
é responsável por criar e atualizar os widgets que exibem o estado atual do drone e uma
representação hierárquica de todos os estados possíveis. Não requer acesso ao node ROS2.
=================================================================================================
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QTreeWidget, QTreeWidgetItem)
from PyQt6.QtCore import Qt
from .utils import gui_log_info

class FSMManager:
    """
    Gerencia a interface visual da Máquina de Estados Finitos (FSM) do sistema.
    
    Esta classe é responsável por criar e atualizar os widgets que exibem o estado atual
    do drone e uma representação hierárquica de todos os estados possíveis.
    """
    
    def __init__(self, signals=None):
        """
        Inicializa o gerenciador da FSM.

        Args:
            signals (DashboardSignals.FSMSignals, optional): Objeto de sinais específicos da FSM para comunicação com a GUI.
        """
        self.signals = signals  # Armazena a referência aos sinais da FSM
        
        # Inicializa os atributos que armazenarão os widgets da interface da FSM.
        self.fsm_tree = None
        self.fsm_states = {} # Dicionário para mapear nomes de estados para itens da árvore.

    def setup_b2_fsm(self):
        """
        Configura o painel B2, que contém a interface visual da FSM.
        Cria o layout e todos os widgets necessários para exibir os estados do sistema.

        Returns:
            QWidget: O widget principal contendo toda a interface da FSM.
        """
        # Registra o início da configuração do painel B2
        gui_log_info("FSMManager", "Configurando painel B2 - FSM")
        
        # Cria o widget principal para o painel B2 e seu layout vertical.
        b2_widget = QWidget()
        b2_layout = QVBoxLayout()
        
        # Define as margens e espaçamento do layout.
        b2_layout.setContentsMargins(2, 2, 2, 2)
        b2_layout.setSpacing(0)
        
        # Cria o QLabel para o título da FSM e configura seu alinhamento e altura máxima.
        fsm_title = QLabel("Estados da FSM")
        fsm_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        fsm_title.setMaximumHeight(33)
        
        # Define o estilo CSS para o título da FSM.
        fsm_title.setStyleSheet("""
            font-weight: bold; 
            color: #ecf0f1; 
            background-color: #5D2E0A; 
            padding: 4px; 
            border-radius: 3px;
            font-size: 14px;
            border: 1px solid #8B4513;
            margin-bottom: 6px;
        """)
        b2_layout.addWidget(fsm_title, 0, Qt.AlignmentFlag.AlignTop)
        
        # Configura a árvore de estados da FSM.
        self.setup_fsm_tree(b2_layout)
        
        # Define o estado inicial da FSM (agora apenas destaca na árvore).
        self.update_state("Iniciando")
        
        # Define o layout para o widget principal do painel B2 e o retorna.
        b2_widget.setLayout(b2_layout)
        return b2_widget

    def setup_fsm_tree(self, layout):
        """
        Configura o QTreeWidget que exibe a estrutura hierárquica dos estados da FSM.
        Define o estilo visual da árvore e popula com os estados possíveis do sistema.

        Args:
            layout (QLayout): O layout onde a árvore será adicionada.
        """
        # Registra o início da configuração da árvore FSM
        gui_log_info("FSMManager", "Configurando visualização da árvore FSM")
        
        # Cria o QTreeWidget e configura suas propriedades.
        self.fsm_tree = QTreeWidget()
        self.fsm_tree.setHeaderHidden(True) # Esconde o cabeçalho da árvore.
        self.fsm_tree.setSelectionMode(QTreeWidget.SelectionMode.SingleSelection) # Permite apenas uma seleção.
        self.fsm_tree.setFocusPolicy(Qt.FocusPolicy.NoFocus) # Remove o foco do teclado.
        
        # Define o estilo CSS para a árvore de estados.
        self.fsm_tree.setStyleSheet("""
            QTreeWidget {
                background-color: #34495e;
                border: 1px solid #7f8c8d;
                border-radius: 3px;
                font-size: 12px;
                color: #ecf0f1;
            }
            QTreeWidget::item {
                padding: 3px;
                border-bottom: 1px solid #7f8c8d;
            }
            QTreeWidget::item:selected {
                background-color: #3498db;
                color: white;
            }
        """)
        
        # Cria a estrutura de estados na árvore.
        self.create_fsm_tree_structure()
        
        # Adiciona a árvore de estados ao layout fornecido.
        layout.addWidget(self.fsm_tree, 1)

    def create_fsm_tree_structure(self):
        """
        Cria a estrutura hierárquica dos estados na árvore visual (QTreeWidget).
        Define todos os estados possíveis do sistema de inspeção do drone, organizados em categorias.
        
        Estrutura compatível com os IntEnums do fsm_node.py:
        - FSMState: DESATIVADO, PRONTO, EXECUTANDO_MISSAO, INSPECAO_FINALIZADA, RETORNO_HELIDECK
        - FSMSubState: ARMANDO, DECOLANDO, VOANDO, INSPECIONANDO, RETORNANDO, DESARMANDO
        - FSMInspectionState: DETECTANDO, CENTRALIZANDO, ESCANEANDO, FALHA_DE_DETECCAO
        - FSMScanState: SEM_ANOMALIA, ANOMALIA_DETECTADA, FOCANDO_ANOMALIA
        """
        # Limpa a árvore existente antes de recriar a estrutura.
        self.fsm_tree.clear()
        
        # Cria o item raiz da árvore.
        root = QTreeWidgetItem(self.fsm_tree)
        root.setText(0, "🤖 Sistema de Inspeção")
        root.setExpanded(True)
        
        # Inicializa o dicionário para armazenar referências aos itens da árvore por nome de estado.
        self.fsm_states = {}
        
        # ==================== ESTADOS PRINCIPAIS (FSMState) ====================
        
        # Estado: DESATIVADO
        desativado_item = QTreeWidgetItem(root)
        desativado_item.setText(0, "⛔ Desativado")
        self.fsm_states["DESATIVADO"] = desativado_item
        
        # Estado: PRONTO
        ready_item = QTreeWidgetItem(root)
        ready_item.setText(0, "✅ Pronto")
        self.fsm_states["PRONTO"] = ready_item
        
        # Estado: EXECUTANDO_MISSAO (com sub-estados)
        executing_mission_item = QTreeWidgetItem(root)
        executing_mission_item.setText(0, "🚀 Executando Missão")
        executing_mission_item.setExpanded(True)
        self.fsm_states["EXECUTANDO_MISSAO"] = executing_mission_item

        # ==================== SUB-ESTADOS DE EXECUTANDO_MISSAO (FSMSubState) ====================

        # Sub-estado: ARMANDO
        arming_item = QTreeWidgetItem(executing_mission_item)
        arming_item.setText(0, "⚙️ Armando")
        self.fsm_states["EXECUTANDO_MISSAO -> ARMANDO"] = arming_item

        # Sub-estado: DECOLANDO
        takeoff_item = QTreeWidgetItem(executing_mission_item)
        takeoff_item.setText(0, "⬆️ Decolando")
        self.fsm_states["EXECUTANDO_MISSAO -> DECOLANDO"] = takeoff_item

        # Sub-estado: VOANDO
        flying_item = QTreeWidgetItem(executing_mission_item)
        flying_item.setText(0, "✈️ Voando")
        self.fsm_states["EXECUTANDO_MISSAO -> VOANDO"] = flying_item

        # Sub-estado: INSPECIONANDO (com sub-sub-estados)
        inspecting_item = QTreeWidgetItem(executing_mission_item)
        inspecting_item.setText(0, "🔍 Inspecionando")
        inspecting_item.setExpanded(True)
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO"] = inspecting_item

        # ==================== SUB-ESTADOS DE INSPECIONANDO (FSMInspectionState) ====================

        # Sub-sub-estado: DETECTANDO
        detecting_item = QTreeWidgetItem(inspecting_item)
        detecting_item.setText(0, "🔄 Detectando")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> DETECTANDO"] = detecting_item

        # Sub-sub-estado: CENTRALIZANDO
        centralizing_item = QTreeWidgetItem(inspecting_item)
        centralizing_item.setText(0, "🎯 Centralizando")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> CENTRALIZANDO"] = centralizing_item

        # Sub-sub-estado: ESCANEANDO (com sub-sub-sub-estados)
        scanning_item = QTreeWidgetItem(inspecting_item)
        scanning_item.setText(0, "🔭 Escaneando")
        scanning_item.setExpanded(True)
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> ESCANEANDO"] = scanning_item

        # ==================== SUB-ESTADOS DE ESCANEANDO (FSMScanState) ====================

        # Sub-sub-sub-estado: SEM_ANOMALIA
        no_anomaly_item = QTreeWidgetItem(scanning_item)
        no_anomaly_item.setText(0, "✅ Sem Anomalia")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> ESCANEANDO -> SEM_ANOMALIA"] = no_anomaly_item

        # Sub-sub-sub-estado: ANOMALIA_DETECTADA
        anomaly_detected_item = QTreeWidgetItem(scanning_item)
        anomaly_detected_item.setText(0, "🚨 Anomalia Detectada")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> ESCANEANDO -> ANOMALIA_DETECTADA"] = anomaly_detected_item

        # Sub-sub-sub-estado: FOCANDO_ANOMALIA
        focusing_anomaly_item = QTreeWidgetItem(scanning_item)
        focusing_anomaly_item.setText(0, "🔎 Focando Anomalia")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> ESCANEANDO -> FOCANDO_ANOMALIA"] = focusing_anomaly_item

        # Sub-sub-estado: FALHA_DE_DETECCAO
        failure_detection_item = QTreeWidgetItem(inspecting_item)
        failure_detection_item.setText(0, "⚠️ Falha de Detecção")
        self.fsm_states["EXECUTANDO_MISSAO -> INSPECIONANDO -> FALHA_DE_DETECCAO"] = failure_detection_item

        # ==================== OUTROS ESTADOS PRINCIPAIS ====================
        
        # Estado: INSPECAO_FINALIZADA
        inspection_finished_item = QTreeWidgetItem(root)
        inspection_finished_item.setText(0, "✅ Inspeção Finalizada")
        self.fsm_states["INSPECAO_FINALIZADA"] = inspection_finished_item
        
        # Estado: RETORNO_HELIDECK (com sub-estados)
        return_helideck_item = QTreeWidgetItem(root)
        return_helideck_item.setText(0, "🏠 Retorno Helideck")
        return_helideck_item.setExpanded(True)
        self.fsm_states["RETORNO_HELIDECK"] = return_helideck_item

        # Sub-estado: RETORNANDO
        returning_item = QTreeWidgetItem(return_helideck_item)
        returning_item.setText(0, "🔙 Retornando")
        self.fsm_states["RETORNO_HELIDECK -> RETORNANDO"] = returning_item

        # Sub-estado: DESARMANDO
        disarming_item = QTreeWidgetItem(return_helideck_item)
        disarming_item.setText(0, "⚙️ Desarmando")
        self.fsm_states["RETORNO_HELIDECK -> DESARMANDO"] = disarming_item
        
        # Expande todos os itens da árvore para que todos os estados sejam visíveis.
        self.fsm_tree.expandAll()

    def highlight_current_state_in_tree(self, state):
        """
        Destaca visualmente o estado atual na árvore de estados (QTreeWidget).
        Remove qualquer destaque anterior e aplica o novo destaque, formatando a fonte em negrito.

        Args:
            state (str): O nome do estado a ser destacado na árvore.
        """
        # O estado pode vir como "PRINCIPAL -> SUB -> SUBSUB", então precisamos encontrar a chave correta
        # no dicionário self.fsm_states. A chave é exatamente a string completa do estado.
        if state in self.fsm_states:
            item = self.fsm_states[state]
            
            # Limpa qualquer seleção anterior na árvore.
            self.fsm_tree.clearSelection()
            
            # Seleciona o item correspondente ao estado atual e o define como item atual.
            item.setSelected(True)
            self.fsm_tree.setCurrentItem(item)
            
            # Itera sobre todos os estados para definir a fonte (negrito ou normal).
            for state_name, state_item in self.fsm_states.items():
                font = state_item.font(0)
                if state_name == state:
                    font.setBold(True) # Define negrito para o estado atual.
                else:
                    font.setBold(False) # Remove negrito dos outros estados.
                state_item.setFont(0, font)

    def update_state(self, state):
        """
        Atualiza a interface visual da FSM com um novo estado.
        Este método é chamado sempre que o estado do sistema muda, atualizando o destaque na árvore.

        Args:
            state (str): O novo estado do sistema (pode ser hierárquico, ex: "EXECUTANDO_MISSAO -> VOANDO").
        """
        # Registra a atualização do estado
        gui_log_info("FSMManager", f"Atualizando estado para: {state}")
        
        # Destaca o estado atual na árvore de estados.
        self.highlight_current_state_in_tree(state)
        
    def fsm_state_callback(self, msg):
        """
        Callback ROS2 para receber mensagens de mudança de estado da FSM.
        Esta função é automaticamente invocada quando uma nova mensagem é recebida no tópico ROS2.

        Args:
            msg (std_msgs.msg.String): A mensagem ROS2 contendo o novo estado da FSM.
        """
        # Registra a mensagem de estado recebida
        gui_log_info("FSMManager", f"Estado FSM recebido: {msg.data}")
        
        # Atualiza o estado da interface com o novo estado recebido.
        self.update_state(msg.data)
