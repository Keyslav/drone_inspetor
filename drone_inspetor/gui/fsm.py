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
        
        # Dados dinâmicos da missão (atualizados via update_state)
        self.current_mission_data = {
            "ponto_de_inspecao_indice_atual": 0,
            "total_pontos_de_inspecao": 0,
            "objeto_alvo": "",
            "tipos_anomalia": []
        }
        
        # Textos base para os itens (sem dados dinâmicos)
        self.base_texts = {
            "EXECUTANDO_INSPECIONANDO": "🔍 INSPECIONANDO",
            "EXECUTANDO_INSPECIONANDO_DETECTANDO": "🔄 DETECTANDO",
            "EXECUTANDO_INSPECIONANDO_ESCANEANDO": "🔭 ESCANEANDO",
            "EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO": "✅ ESCANEAMENTO FINALIZADO",
            "EXECUTANDO_INSPECIONANDO_FALHA": "⚠️ FALHA NA DETECÇÃO"
        }

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
        
        Os estados compostos (ex: EXECUTANDO_INSPECIONANDO_DETECTANDO) são exibidos
        como uma hierarquia:
        - EXECUTANDO
            - ARMANDO
            - DECOLANDO
            - INSPECIONANDO
                - DETECTANDO
                - CENTRALIZANDO
                - ESCANEANDO
                - FALHA
        
        Quando um estado composto está ativo, todos os níveis ancestrais são destacados.
        """
        # Limpa a árvore existente antes de recriar a estrutura.
        self.fsm_tree.clear()
        
        # Cria o item raiz da árvore.
        root = QTreeWidgetItem(self.fsm_tree)
        root.setText(0, "🤖 Sistema de Inspeção")
        root.setExpanded(True)
        
        # Dicionário principal: mapeia state_name completo para item
        self.fsm_states = {}
        
        # Dicionário auxiliar: mapeia state_name para lista de itens a destacar
        # (o próprio + todos os ancestrais)
        self.fsm_highlight_items = {}
        
        # ==================== ESTADOS DE NÍVEL 0 - SISTEMA ====================
        
        # Estado: DESATIVADO (0)
        desativado_item = QTreeWidgetItem(root)
        desativado_item.setText(0, "⛔ DESATIVADO")
        self.fsm_states["DESATIVADO"] = desativado_item
        self.fsm_highlight_items["DESATIVADO"] = [desativado_item]
        
        # Estado: PRONTO (1)
        pronto_item = QTreeWidgetItem(root)
        pronto_item.setText(0, "✅ PRONTO")
        self.fsm_states["PRONTO"] = pronto_item
        self.fsm_highlight_items["PRONTO"] = [pronto_item]
        
        # ==================== EXECUTANDO (NÍVEL 1) ====================
        
        # Nó pai: EXECUTANDO
        executando_item = QTreeWidgetItem(root)
        executando_item.setText(0, "🚀 EXECUTANDO")
        executando_item.setExpanded(True)
        
        # Sub-estado: ARMANDO
        armando_item = QTreeWidgetItem(executando_item)
        armando_item.setText(0, "⚙️ ARMANDO")
        self.fsm_states["EXECUTANDO_ARMANDO"] = armando_item
        self.fsm_highlight_items["EXECUTANDO_ARMANDO"] = [executando_item, armando_item]
        
        # Sub-estado: DECOLANDO
        decolando_item = QTreeWidgetItem(executando_item)
        decolando_item.setText(0, "⬆️ DECOLANDO")
        self.fsm_states["EXECUTANDO_DECOLANDO"] = decolando_item
        self.fsm_highlight_items["EXECUTANDO_DECOLANDO"] = [executando_item, decolando_item]
        
        # ==================== INSPECIONANDO (NÍVEL 2) ====================
        
        # Nó pai: INSPECIONANDO (filho de EXECUTANDO)
        inspecionando_item = QTreeWidgetItem(executando_item)
        inspecionando_item.setText(0, "🔍 INSPECIONANDO")
        inspecionando_item.setExpanded(True)
        self.fsm_states["EXECUTANDO_INSPECIONANDO"] = inspecionando_item
        self.fsm_highlight_items["EXECUTANDO_INSPECIONANDO"] = [executando_item, inspecionando_item]
        
        # Sub-sub-estado: DETECTANDO (20)
        detectando_item = QTreeWidgetItem(inspecionando_item)
        detectando_item.setText(0, "🔄 DETECTANDO")
        self.fsm_states["EXECUTANDO_INSPECIONANDO_DETECTANDO"] = detectando_item
        self.fsm_highlight_items["EXECUTANDO_INSPECIONANDO_DETECTANDO"] = [executando_item, inspecionando_item, detectando_item]
        
        # Sub-sub-estado: ESCANEANDO (22)
        escaneando_item = QTreeWidgetItem(inspecionando_item)
        escaneando_item.setText(0, "🔭 ESCANEANDO")
        self.fsm_states["EXECUTANDO_INSPECIONANDO_ESCANEANDO"] = escaneando_item
        self.fsm_highlight_items["EXECUTANDO_INSPECIONANDO_ESCANEANDO"] = [executando_item, inspecionando_item, escaneando_item]
        
        # Sub-sub-estado: ESCANEAMENTO_FINALIZADO (23)
        escaneamento_finalizado_item = QTreeWidgetItem(inspecionando_item)
        escaneamento_finalizado_item.setText(0, "✅ ESCANEAMENTO FINALIZADO")
        self.fsm_states["EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO"] = escaneamento_finalizado_item
        self.fsm_highlight_items["EXECUTANDO_INSPECIONANDO_ESCANEAMENTO_FINALIZADO"] = [executando_item, inspecionando_item, escaneamento_finalizado_item]
        
        # Sub-sub-estado: FALHA (25)
        falha_item = QTreeWidgetItem(inspecionando_item)
        falha_item.setText(0, "⚠️ FALHA NA DETECÇÃO")
        self.fsm_states["EXECUTANDO_INSPECIONANDO_FALHA"] = falha_item
        self.fsm_highlight_items["EXECUTANDO_INSPECIONANDO_FALHA"] = [executando_item, inspecionando_item, falha_item]
        
        # ==================== ESTADOS DE FINALIZAÇÃO ====================
        
        # Estado: INSPECAO_FINALIZADA (30)
        finalizada_item = QTreeWidgetItem(root)
        finalizada_item.setText(0, "✅ INSPECAO_FINALIZADA")
        self.fsm_states["INSPECAO_FINALIZADA"] = finalizada_item
        self.fsm_highlight_items["INSPECAO_FINALIZADA"] = [finalizada_item]
        
        # Estado: RETORNANDO (40)
        retornando_item = QTreeWidgetItem(root)
        retornando_item.setText(0, "🏠 RETORNANDO")
        self.fsm_states["RETORNANDO"] = retornando_item
        self.fsm_highlight_items["RETORNANDO"] = [retornando_item]
        
        # Expande todos os itens da árvore para que todos os estados sejam visíveis.
        self.fsm_tree.expandAll()

    def highlight_current_state_in_tree(self, state):
        """
        Destaca visualmente o estado atual na árvore de estados (QTreeWidget).
        Para estados compostos, destaca todos os níveis hierárquicos (ancestrais).
        
        Por exemplo, para EXECUTANDO_INSPECIONANDO_DETECTANDO, destaca:
        - EXECUTANDO (nível 1)
        - INSPECIONANDO (nível 2)
        - DETECTANDO (nível 3)

        Args:
            state (str): O nome do estado a ser destacado na árvore.
        """
        # Primeiro, remove o destaque de todos os itens
        for state_name, state_item in self.fsm_states.items():
            font = state_item.font(0)
            font.setBold(False)
            state_item.setFont(0, font)
            state_item.setSelected(False)
        
        # Obtém a lista de itens a destacar para este estado
        items_to_highlight = self.fsm_highlight_items.get(state, [])
        
        if items_to_highlight:
            # Limpa qualquer seleção anterior na árvore.
            self.fsm_tree.clearSelection()
            
            # Destaca todos os itens da hierarquia
            for item in items_to_highlight:
                font = item.font(0)
                font.setBold(True)
                item.setFont(0, font)
                item.setSelected(True)
            
            # Define o último item (mais específico) como o item atual
            self.fsm_tree.setCurrentItem(items_to_highlight[-1])

    def fsm_state_callback(self, msg):
        """
        Callback ROS2 para receber mensagens de mudança de estado da FSM.
        Esta função é automaticamente invocada quando uma nova mensagem é recebida no tópico ROS2.

        Args:
            msg: Mensagem ROS2 contendo o novo estado da FSM (dict via sinal PyQt).
        """
        # Registra a mensagem de estado recebida
        if isinstance(msg, dict):
            state_name = msg.get("state_name", "")
        else:
            state_name = str(msg)
        gui_log_info("FSMManager", f"Estado FSM recebido: {state_name}")
        
        # Atualiza o estado da interface com o novo estado recebido.
        self.update_state(msg)


    def update_state(self, state_data):
        """
        Atualiza a interface visual da FSM com dados do novo estado.
        Este método é chamado sempre que o estado do sistema muda.
        
        Aceita tanto dict (novo formato de FSMStateMSG) quanto str (legado).

        Args:
            state_data: Dados do estado - pode ser dict com campos de FSMStateMSG ou str com state_name
        """
        # Extrai state_name do dict ou usa string diretamente
        if isinstance(state_data, dict):
            state = state_data.get("state_name", "")
            
            # Atualiza dados dinâmicos da missão
            self.current_mission_data["ponto_de_inspecao_indice_atual"] = state_data.get("ponto_de_inspecao_indice_atual", 0)
            self.current_mission_data["total_pontos_de_inspecao"] = state_data.get("total_pontos_de_inspecao", 0)
            self.current_mission_data["objeto_alvo"] = state_data.get("objeto_alvo", "")
            self.current_mission_data["tipos_anomalia"] = state_data.get("tipos_anomalia", [])
        else:
            state = str(state_data)
        
        # Atualiza textos dinâmicos dos itens da árvore
        self._update_dynamic_labels()
        
        # Destaca o estado atual na árvore de estados.
        self.highlight_current_state_in_tree(state)
    
    def _update_dynamic_labels(self):
        """
        Atualiza os textos dinâmicos dos itens INSPECIONANDO, DETECTANDO e ESCANEANDO
        com informações da missão atual.
        """
        idx = self.current_mission_data["ponto_de_inspecao_indice_atual"]
        total = self.current_mission_data["total_pontos_de_inspecao"]
        objeto_alvo = self.current_mission_data["objeto_alvo"]
        tipos_anomalia = self.current_mission_data["tipos_anomalia"]
        
        # INSPECIONANDO: mostra índice atual / total (ex: "🔍 INSPECIONANDO (2/4)")
        inspecionando_item = self.fsm_states.get("EXECUTANDO_INSPECIONANDO")
        if inspecionando_item:
            if total > 0:
                # Índice é 0-based, exibimos como 1-based
                text = f"🔍 INSPECIONANDO ({idx + 1}/{total})"
            else:
                text = self.base_texts["EXECUTANDO_INSPECIONANDO"]
            inspecionando_item.setText(0, text)
        
        # DETECTANDO: mostra objeto alvo (ex: "🔄 DETECTANDO: Flare")
        detectando_item = self.fsm_states.get("EXECUTANDO_INSPECIONANDO_DETECTANDO")
        if detectando_item:
            if objeto_alvo:
                text = f"🔄 DETECTANDO: {objeto_alvo}"
            else:
                text = self.base_texts["EXECUTANDO_INSPECIONANDO_DETECTANDO"]
            detectando_item.setText(0, text)
        
        # ESCANEANDO: mostra tipos de anomalia (ex: "🔭 ESCANEANDO: corrosion, crack")
        escaneando_item = self.fsm_states.get("EXECUTANDO_INSPECIONANDO_ESCANEANDO")
        if escaneando_item:
            if tipos_anomalia:
                # Formata lista de anomalias (remove prefixos comuns para legibilidade)
                anomalias_formatadas = [a.replace("_", " ").replace("corrosion", "corr.") for a in tipos_anomalia]
                text = f"🔭 ESCANEANDO: {', '.join(anomalias_formatadas)}"
            else:
                text = self.base_texts["EXECUTANDO_INSPECIONANDO_ESCANEANDO"]
            escaneando_item.setText(0, text)
