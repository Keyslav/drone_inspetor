# fsm_node.py
# =================================================================================================
# NÓ DA MÁQUINA DE ESTADOS FINITA (O "CÉREBRO")
# =================================================================================================
# RESPONSABILIDADE PRINCIPAL:
# Gerenciar o fluxo lógico e sequencial da missão de inspeção. Implementa uma FSM hierárquica
# com estados e sub-estados para controlar o comportamento do drone durante
# toda a missão.
#
# =================================================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import time
import json # Para processar mensagens de detecção do cv_node
import math # Para cálculos de ângulos e distâncias
from enum import IntEnum


# ==================================================================================================
# ENUMS DE ESTADOS DA FSM
# ==================================================================================================

class FSMState(IntEnum):
    """
    Estados principais da FSM (nível 0).
    Representam o estado geral da missão de inspeção.
    """
    DESATIVADO = 0          # Sistema desativado, aguardando tópicos essenciais
    PRONTO = 1              # Pronto para iniciar missão
    EXECUTANDO_MISSAO = 2   # Executando missão de inspeção
    INSPECAO_FINALIZADA = 3 # Inspeção concluída, pausando antes de retornar
    RETORNO_HELIDECK = 4    # Retornando para base


class FSMSubState(IntEnum):
    """
    Sub-estados da FSM (nível 1).
    Detalham a fase dentro do estado principal.
    """
    NENHUM = 0              # Sem sub-estado ativo
    ARMANDO = 1             # Armando motores
    DECOLANDO = 2           # Decolando até altitude de missão
    VOANDO = 3              # Voando para ponto de inspeção
    INSPECIONANDO = 4       # Executando inspeção
    RETORNANDO = 5          # Retornando para base
    DESARMANDO = 6          # Desarmando motores


class FSMInspectionState(IntEnum):
    """
    Sub-estados de inspeção (nível 2).
    Detalham a fase dentro da inspeção.
    """
    NENHUM = 0              # Sem sub-estado ativo
    DETECTANDO = 1          # Procurando target (girando 360°)
    CENTRALIZANDO = 2       # Centralizando target no frame
    ESCANEANDO = 3          # Executando voo orbital de escaneamento
    FALHA_DE_DETECCAO = 4   # Falha na detecção do target


class FSMScanState(IntEnum):
    """
    Sub-estados de escaneamento (nível 3).
    Detalham a fase dentro do escaneamento orbital.
    """
    NENHUM = 0              # Sem sub-estado ativo
    SEM_ANOMALIA = 1        # Escaneando sem anomalia detectada
    ANOMALIA_DETECTADA = 2  # Anomalia detectada durante escaneamento
    FOCANDO_ANOMALIA = 3    # Focando na anomalia detectada


class FSMNode(Node):
    """
    Implementa a lógica de estados hierárquica da missão de inspeção.
    
    FLUXO DA MISSÃO:

    1. DESATIVADO: Falha em algum dos tópicos essenciais
    2. PRONTO: Drone pronto e aguardando comando "start_inspection" do dashboard
    3. EXECUTANDO_MISSAO/ARMANDO: Arma os motores
    4. EXECUTANDO_MISSAO/DECOLANDO: Decola para 20m
    5. EXECUTANDO_MISSAO/VOANDO: Navega para Ponto de Detecção
    6. EXECUTANDO_MISSAO/INSPECIONANDO/PONTO_DE_DETECCAO: Verifica estabilidade por 5s
    7. EXECUTANDO_MISSAO/INSPECIONANDO/DETECTANDO: Gira 360° procurando target
    8. EXECUTANDO_MISSAO/INSPECIONANDO/ESCANEANDO/SEM_ANOMALIA: Voo orbital ao redor do target
    9. EXECUTANDO_MISSAO/INSPECIONANDO/ESCANEANDO/ANOMALIA_DETECTADA: Detecção de anomalia durante escaneamento
    10. EXECUTANDO_MISSAO/INSPECIONANDO/ESCANEANDO/FOCANDO_ANOMALIA: Aproxima e foca na anomalia por 5s
    11. INSPECAO_FINALIZADA: Pausa por 3s
    12. RETORNO_HELIDECK: RTL e pousa
    """
    
    def __init__(self):
        # --- Inicialização do Nó ROS2 ---
        super().__init__("fsm_node")
        self.get_logger().info("================ INICIALIZANDO FSM NODE ===============")

        # QoS para comandos críticos: TRANSIENT_LOCAL + BEST_EFFORT (baixa latência, disponível para novos subscribers)
        qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS para status do sistema: TRANSIENT_LOCAL + BEST_EFFORT (estado atual disponível para novos subscribers)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- Definição de Estados Hierárquicos ---
        # Estado é uma lista de IntEnum representando a hierarquia: [nível0, nível1, nível2, nível3]
        # Usa IntEnum para comparações rápidas. Use .name para obter o nome como string.
        self.state = [FSMState.DESATIVADO, FSMSubState.NENHUM, FSMInspectionState.NENHUM, FSMScanState.NENHUM]
        self.state_anterior = [FSMState.DESATIVADO, FSMSubState.NENHUM, FSMInspectionState.NENHUM, FSMScanState.NENHUM]

        # --- Variáveis de Controle ---
        self.drone_offboard_mode = False      # Indica se o modo OFFBOARD está ativo no Drone (drone_node)
        self.drone_arming = False             # Indica se o drone está sendo armado
        self.drone_armed = False              # Indica se o drone está armado
        self.drone_landed = False             # Indica se o drone está pousado
        self.drone_on_trajectory = False      # Indica se o drone está seguindo uma trajetória
        self.drone_reached_waypoint = False   # Indica se o drone chegou ao ponto de inspeção
        self.drone_local_position_x = 0.0     # Posição local X (Norte) em metros
        self.drone_local_position_y = 0.0     # Posição local Y (Leste) em metros
        self.drone_local_position_z = 0.0     # Posição local Z (positivo = acima do ponto de partida, já invertido pelo drone_node)
        self.drone_global_position_lat = 0.0  # Latitude do drone
        self.drone_global_position_lon = 0.0  # Longitude do drone
        self.drone_global_position_alt = 0.0  # Altitude do drone
        self.drone_yaw = 0.0                  # Yaw do drone
        
        # --- Estado do Drone (do drone_node) ---
        self.drone_state = "POUSADO_DESARMADO" # Estado principal do drone (da máquina de estados)
        self.drone_sub_state = "NENHUM"        # Sub-estado do drone (da máquina de estados)
        self.drone_state_duration = 0.0        # Duração no estado atual (segundos)
        
        # --- Controle de Aguardo de Comando ---
        self.waiting_for_state = None          # Estado esperado após enviar comando
        self.command_timeout = 10.0            # Timeout em segundos para aguardar estado
        self.command_sent_time = 0.0           # Timestamp do envio do comando
        
        # --- Parâmetros da Missão ---
        self.mission_started = False            # Flag para iniciar a missão a partir do estado PRONTO
        self.inspection_type = "Flare"          # Tipo de inspeção selecionado (Flare, Tanques de GLP, etc.)
        self.takeoff_altitude = 20.0            # Altitude alvo para a decolagem (metros)
        self.inspection_distance = 5.0          # Distância ideal do drone ao target durante a inspeção (metros)
        self.anomaly_approach_distance = 2.0    # Quanto o drone se aproxima ao detectar uma anomalia (metros)
        self.waypoint_tolerance = 0.5           # Tolerância em metros para considerar que chegou ao waypoint
        self.inspection_point_lat = -22.634010  # Latitude do ponto de inspeção
        self.inspection_point_lon = -40.092463  # Longitude do ponto de inspeção
        self.inspection_point_alt = 99.0        # Altitude do ponto de inspeção
        self.target_lat = -22.633061            # Latitude do centro do target para órbita
        self.target_lon = -40.093330            # Longitude do centro do target para órbita
        self.target_alt = 99.0                  # Altitude do centro do target para órbita
        
        # --- Parâmetros do ESCANEAMENTO ---
        self.current_scanning_angle = 0.0     # Ângulo atual durante a inspeção circular
        self.anomaly_angles_list = []         # Lista de ângulos das anomalias detectadas (para critério de 30°)
        
        # --- Estados de Controle Interno ---
        self.target_detected = False          # Flag para indicar se o target foi detectado
        self.anomaly_detected = False         # Flag para indicar se uma anomalia foi detectada
        
        # --- Variáveis de Detecção de Flare ---
        self.flare_detected = False                     # Flag para indicar se o flare foi detectado
        self.flare_detection_start_yaw = None           # Yaw inicial quando começou a busca
        self.flare_detection_yaw_covered = 0.0          # Quanto já girou durante a busca (em graus)
        self.flare_detection_current_target_yaw = None  # Yaw alvo atual para busca incremental
        self.flare_detection_increment = 4.5            # Incremento de yaw para busca (graus)
        self.flare_detection_yaw_reached = False        # Flag para indicar se o yaw alvo foi alcançado
        
        # --- Variáveis de Centralização do Target ---
        self.target_centralized = False       # Flag para indicar se o target está centralizado
        self.target_bbox_center = None        # Centro do bounding box do target [x, y]
        self.target_yaw_offset = 0.0          # Offset de yaw necessário para centralizar o target
        self.centralization_tolerance = 2.0   # Tolerância em graus para considerar centralizado
        
        # --- Variáveis de Estabilidade e Inspeção ---
        self.is_stable = False                # Flag para indicar se o drone está estável
        self.stability_check_start_time = 0   # Timestamp do início da checagem de estabilidade
        self.inspection_completed = False     # Flag para indicar se a inspeção foi completada
        self.inspection_start_angle = 0.0     # Ângulo inicial da inspeção
        self.current_inspection_angle = 0.0    # Ângulo atual durante a inspeção
        self.last_anomaly_angle = 0.0         # Último ângulo onde uma anomalia foi detectada
        self.failure_pause_start_time = 0      # Timestamp do início da pausa após falha
        self.inspection_pause_start_time = 0   # Timestamp do início da pausa após inspeção

        # --- Verificação de Saúde dos Tópicos Essenciais ---
        # Dicionário com informações dos tópicos essenciais: {nome_tópico: {"last_received": timestamp}}
        self.essencial_topics = {
            "drone_status": {"last_received": 0.0, "description": "Status do drone_node"},
            #"cv_detections": {"last_received": 0.0, "description": "Detecções do cv_node"},
            # "dashboard_commands": {"last_received": 0.0, "description": "Comandos do dashboard"}
        }
        self.topic_health_timeout = 2.0  # Timeout de 2 segundos para considerar tópico inativo

        # ==================================================================
        # PUBLISHERS
        # ==================================================================
        # Publica o estado atual da FSM para o dashboard
        self.fsm_state_pub = self.create_publisher(String, "/drone_inspetor/interno/fsm_node/state", qos_status)
        
        # Envia comandos para o drone_node
        self.fsm_command_pub = self.create_publisher(String, "/drone_inspetor/interno/fsm_node/drone_commands", qos_commands)

        # ==================================================================
        # SUBSCRIBERS
        # ==================================================================
        # Recebe status do drone_node (armado, decolagem concluída, chegou ao destino, etc.)
        self.drone_status_sub = self.create_subscription(
            String, "/drone_inspetor/interno/drone_node/status", self.drone_status_callback, qos_status)
        
        # Recebe comandos do dashboard_node (start_inspection, abort_mission, etc.)
        self.dashboard_command_sub = self.create_subscription(
            String, "/drone_inspetor/interno/dashboard_node/mission_commands", self.dashboard_command_callback, qos_commands)
        
        # Recebe detecções do cv_node (flare detectado, anomalia detectada, etc.)
        # QoS padrão para dados de sensores: VOLATILE + BEST_EFFORT (alta frequência, não crítico perder algumas)
        qos_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cv_detection_sub = self.create_subscription(
            String, "/drone_inspetor/interno/cv_node/object_detections", self.cv_detection_callback, qos_sensor_data)

        # ==================================================================
        # TIMERS
        # ==================================================================
        # Timer principal da FSM - executa a lógica de estados a cada 100ms
        self.fsm_timer = self.create_timer(0.1, self.fsm_step)
        
        # Timer para verificação de saúde dos tópicos essenciais - executa a cada 2 segundos
        self.topic_health_timer = self.create_timer(2.0, self.check_essential_topics)
        
        self.get_logger().info("================ FSM NODE PRONTO ================")

    # ==================================================================
    # LÓGICA PRINCIPAL DA FSM
    # ==================================================================

    def fsm_step(self):
        """
        O coração da FSM, executado a cada 100ms.
        Implementa a máquina de estados hierárquica completa.
        """

        # =================================================
        # ESTADO: DESATIVADO
        # =================================================
        if self.state[0] == FSMState.DESATIVADO:

            # Verifica se o drone está em modo OFFBOARD
            if not self.drone_offboard_mode:
                self.get_logger().info(f">>> Aguardando mudança para Modo OFFBOARD!", throttle_duration_sec=2)
                return

            self.get_logger().info("Sistema no modo OFFBOARD. Transicionando para Estado PRONTO!")
            self.set_state(FSMState.PRONTO)
        
        # Verifica se o drone está em modo OFFBOARD
        if not self.drone_offboard_mode:
            # Se o drone não está em modo OFFBOARD, retorna para o estado DESATIVADO
            self.get_logger().error(f"Drone saiu do modo OFFBOARD. Resetando FSM...")
            self.reset_FSM()
            return

        # =================================================
        # ESTADO: PRONTO
        # =================================================
        if self.state[0] == FSMState.PRONTO:

            # Verifica se o comando para iniciar a missão foi recebido
            if self.mission_started:
                self.get_logger().info("\033[1m\033[31mINICIANDO MISSÃO: Iniciando sequência de missão!\033[0m")

                # Reseta a flag de início de missão
                self.mission_started = False

                # Imprime as informações da missão selecionada
                self.get_logger().info(f"INICIANDO MISSÃO: Tipo de inspeção selecionada: {self.inspection_type}")
                
                self.get_logger().info("INICIANDO MISSÃO: Transicionando para estado EXECUTANDO_MISSAO...")
                self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.ARMANDO)
                return
            
            # Se o comando para iniciar a missão não foi recebido, retorna para o estado PRONTO
            self.get_logger().info("Estado PRONTO: Comando para iniciar a missão não recebido. Aguardando comando...", throttle_duration_sec=2)
            return

        # =================================================
        # ESTADO: EXECUTANDO_MISSAO
        # =================================================
        if self.state[0] == FSMState.EXECUTANDO_MISSAO:
            
            # =================================================
            # ESTADO: EXECUTANDO_MISSAO - ARMANDO
            # =================================================
            if self.state[1] == FSMSubState.ARMANDO:
                
                # Verifica se está aguardando estado do drone
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando drone armar... Estado atual: {self.drone_state}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout ao armar! Abortando missão.")
                    self.set_state(FSMState.DESATIVADO)
                    return

                # Se o drone está ARMADO (confirmado pela máquina de estados), transiciona
                if self.drone_state == "POUSADO_ARMADO":
                    self.get_logger().info("Drone ARMADO confirmado. Transicionando para DECOLANDO...")
                    self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.DECOLANDO)
                    return

                # Se o drone está POUSADO, envia comando ARM e aguarda
                if self.drone_state == "POUSADO_DESARMADO":
                    self.get_logger().info("Enviando comando ARM e aguardando confirmação...")
                    self.send_command_and_wait({"command": "ARM"}, "POUSADO_ARMADO", timeout=10.0)
                    return
                
                # Se está em outro estado, loga aviso
                self.get_logger().warn(f"Estado inesperado do drone durante ARMANDO: {self.drone_state}", throttle_duration_sec=2)
                

            # =================================================
            # ESTADO: EXECUTANDO_MISSAO - DECOLANDO
            # =================================================
            if self.state[1] == FSMSubState.DECOLANDO:
                
                # Verifica se está aguardando estado do drone
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Aguardando decolagem... Estado: {self.drone_state}/{self.drone_sub_state}", throttle_duration_sec=2)
                    return
                elif wait_status == "timeout":
                    self.get_logger().error("Timeout na decolagem! Abortando missão.")
                    self.set_state(FSMState.DESATIVADO)
                    return

                # Se o drone está PRONTO (terminou decolagem e está em hover), transiciona
                if self.drone_state == "VOANDO_PRONTO":
                    self.get_logger().info("Decolagem concluída! Drone em VOANDO_PRONTO. Transicionando para VOANDO...")
                    self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.VOANDO)
                    return

                # Se o drone está ARMADO (no chão), envia TAKEOFF e aguarda PRONTO
                if self.drone_state == "POUSADO_ARMADO":
                    self.get_logger().info(f"Enviando comando TAKEOFF para {self.takeoff_altitude}m...")
                    self.send_command_and_wait(
                        {"command": "TAKEOFF", "alt": self.takeoff_altitude},
                        "VOANDO_PRONTO",
                        timeout=30.0
                    )
                    return
                
                # Se está EM_TRAJETORIA com sub-estado DECOLANDO, aguarda
                if self.drone_state == "VOANDO_EM_TRAJETORIA" and self.drone_sub_state == "DECOLANDO":
                    self.get_logger().info("Drone decolando...", throttle_duration_sec=2)
                    return
                
                # Se está em outro estado, loga aviso
                self.get_logger().warn(f"Estado inesperado durante DECOLANDO: {self.drone_state}/{self.drone_sub_state}", throttle_duration_sec=2)
                

            # =================================================
            # ESTADO: EXECUTANDO_MISSAO - VOANDO
            # =================================================
            if self.state[1] == FSMSubState.VOANDO:
                
                # Verifica se está aguardando estado do drone
                wait_status = self.check_waiting_state()
                if wait_status == "waiting":
                    self.get_logger().info(f"Voando para ponto de inspeção... Estado: {self.drone_state}/{self.drone_sub_state}", throttle_duration_sec=2)
                    return

                # Se o drone está PRONTO (hover), verifica se chegou ao destino
                if self.drone_state == "VOANDO_PRONTO":
                    # Verifica se chegou ao ponto de inspeção
                    self.drone_reached_waypoint = self.is_at_waypoint(
                        self.inspection_point_lat,
                        self.inspection_point_lon,
                        self.inspection_point_alt
                    )

                    if self.drone_reached_waypoint:
                        self.get_logger().info("Drone chegou ao ponto de inspeção. Transicionando para INSPECIONANDO...")
                        self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.DETECTANDO)
                        return

                    # Envia comando GOTO para o ponto de inspeção
                    self.get_logger().info(f"Navegando para ponto de inspeção ({self.inspection_point_lat:.6f}, {self.inspection_point_lon:.6f}, {self.inspection_point_alt:.1f})...")
                    self.send_command_and_wait(
                        {"command": "GOTO", "lat": self.inspection_point_lat, "lon": self.inspection_point_lon, "alt": self.inspection_point_alt},
                        "VOANDO_PRONTO",
                        timeout=60.0
                    )
                    return
                
                # Se está em trajetória, aguarda
                if self.drone_state == "VOANDO_EM_TRAJETORIA":
                    self.get_logger().info(f"Em trajetória... Sub-estado: {self.drone_sub_state}", throttle_duration_sec=2)
                    return
                
                # Se não está em estado esperado, loga aviso
                self.get_logger().warn(f"Estado inesperado durante VOANDO: {self.drone_state}", throttle_duration_sec=2)

            # =================================================
            # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO
            # =================================================
            if self.state[1] == FSMSubState.INSPECIONANDO:
            
                
                # ==================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - DETECTANDO
                # ==================================================================
                if self.state[2] == FSMInspectionState.DETECTANDO:

                    # Objetivo: Girar o drone em incrementos de 4.5° até detectar o flare
                    if not self.flare_detected:
                        
                        if self.flare_detection_start_yaw is None:
                            # Inicia a busca incremental
                            self.flare_detection_start_yaw = self.drone_yaw
                            self.flare_detection_current_target_yaw = self.drone_yaw
                            self.flare_detection_yaw_covered = 0.0
                            self.flare_detection_yaw_reached = False
                            self.get_logger().info(f"Iniciando busca incremental de flare a partir do yaw {self.flare_detection_start_yaw:.1f}°...")
                        
                        # Verifica se o yaw alvo atual foi alcançado
                        if self.flare_detection_current_target_yaw is not None:
                            yaw_diff_to_target = self.drone_yaw - self.flare_detection_current_target_yaw
                            # Normaliza para lidar com transição 360°->0°
                            if yaw_diff_to_target < -180:
                                yaw_diff_to_target += 360
                            if yaw_diff_to_target > 180:
                                yaw_diff_to_target -= 360
                            
                            # Verifica se alcançou o yaw alvo (tolerância de 1°)
                            if abs(yaw_diff_to_target) <= 1.0:
                                if not self.flare_detection_yaw_reached:
                                    self.flare_detection_yaw_reached = True
                                    self.get_logger().info(f"Yaw alvo {self.flare_detection_current_target_yaw:.1f}° alcançado. Aguardando detecção...")
                            else:
                                # Ainda não alcançou o yaw alvo, aguarda
                                self.get_logger().debug(f"Aguardando alcançar yaw {self.flare_detection_current_target_yaw:.1f}° (atual: {self.drone_yaw:.1f}°)", 
                                                       throttle_duration_sec=1)
                                return
                        
                        # Se o yaw alvo foi alcançado, calcula o próximo incremento
                        if self.flare_detection_yaw_reached:
                            # Calcula quanto já girou desde o início
                            yaw_diff = self.drone_yaw - self.flare_detection_start_yaw
                            # Normaliza para lidar com transição 360°->0°
                            if yaw_diff < -180:
                                yaw_diff += 360
                            if yaw_diff > 180:
                                yaw_diff -= 360
                            
                            self.flare_detection_yaw_covered = abs(yaw_diff)
                            
                            # Verifica se completou 360° sem detectar
                            if self.flare_detection_yaw_covered >= 360.0:
                                self.get_logger().warn("Flare não detectado após busca completa de 360°.")
                                self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.FALHA_DE_DETECCAO)
                                return
                            
                            # Calcula o próximo yaw alvo (incremento de 4.5°)
                            next_target_yaw = (self.flare_detection_current_target_yaw + self.flare_detection_increment) % 360.0
                            self.flare_detection_current_target_yaw = next_target_yaw
                            self.flare_detection_yaw_reached = False
                            
                            self.get_logger().info(f"Girando para próximo yaw alvo: {next_target_yaw:.1f}° (coberto: {self.flare_detection_yaw_covered:.1f}°)")
                            # Usa send_yaw_command para simular SET_YAW com GOTO
                            self.send_yaw_command(next_target_yaw)

                # ==================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - CENTRALIZANDO
                # ==================================================================
                if self.state[2] == FSMInspectionState.CENTRALIZANDO:
                    # Objetivo: Centralizar o drone apontando para o centro do bounding box do target
                    if not self.target_centralized:
                        # Verifica se o yaw alvo de centralização foi alcançado
                        if self.target_yaw_offset != 0.0:
                            target_yaw = (self.drone_yaw + self.target_yaw_offset) % 360.0
                            yaw_diff = self.drone_yaw - target_yaw
                            # Normaliza para lidar com transição 360°->0°
                            if yaw_diff < -180:
                                yaw_diff += 360
                            if yaw_diff > 180:
                                yaw_diff -= 360
                            
                            if abs(yaw_diff) <= self.centralization_tolerance:
                                # Target centralizado
                                self.target_centralized = True
                                self.get_logger().info("Target centralizado! Transicionando para próximo estado...")
                            else:
                                # Ainda não centralizado, usa send_yaw_command
                                self.get_logger().info(f"Centralizando target... Offset: {self.target_yaw_offset:.1f}° (atual: {self.drone_yaw:.1f}°, alvo: {target_yaw:.1f}°)", 
                                                      throttle_duration_sec=0.5)
                                self.send_yaw_command(target_yaw)
                        else:
                            # Não há offset calculado ainda, aguarda
                            self.get_logger().warn("Aguardando cálculo do offset de centralização...")
                    
                    # Se centralizado, transiciona para o próximo estado
                    if self.target_centralized:
                        self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.ESCANEANDO)

                # ==================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - ESCANEANDO
                # ==================================================================
                if self.state[2] == FSMInspectionState.ESCANEANDO:
                    # TODO: Voo orbital não implementado no drone_node ainda
                    # Por enquanto, marca inspeção como completa após detecção e centralização
                    
                    # Verifica se está no sub-estado SEM_ANOMALIA
                    if self.state[3] == FSMScanState.NENHUM or self.state[3] == FSMScanState.SEM_ANOMALIA:
                        # Inicia no sub-estado SEM_ANOMALIA se ainda não foi definido
                        if self.state[3] == FSMScanState.NENHUM:
                            self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.ESCANEANDO, FSMScanState.SEM_ANOMALIA)
                        
                        if not self.inspection_completed:
                            # TODO: Implementar voo orbital quando disponível no drone_node
                            # Por enquanto, finaliza inspeção após 5 segundos simulando escaneamento
                            if self.inspection_start_angle == 0.0:
                                self.inspection_start_angle = time.time()
                                self.get_logger().warn("Voo orbital não implementado. Simulando escaneamento por 5 segundos...")
                            
                            elapsed = time.time() - self.inspection_start_angle
                            if elapsed >= 5.0:
                                self.get_logger().info("Escaneamento simulado concluído. Finalizando inspeção.")
                                self.inspection_completed = True
                                self.set_state(FSMState.INSPECAO_FINALIZADA)
                            else:
                                self.get_logger().info(f"Escaneando... {5.0 - elapsed:.1f}s restantes", throttle_duration_sec=1)

                # ==================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - FALHA_DE_DETECCAO
                # ==================================================================
                if self.state[2] == FSMInspectionState.FALHA_DE_DETECCAO:
                    # Objetivo: Pausar por 3 segundos após falha e abortar inspeção
                    if self.failure_pause_start_time == 0:
                        self.failure_pause_start_time = time.time()
                        self.get_logger().error("Falha na detecção do flare. Pausando por 3 segundos antes de abortar.")
                        
                    if time.time() - self.failure_pause_start_time >= 3.0:
                        self.get_logger().info("Pausa de falha concluída. Abortando inspeção.")
                        self.set_state(FSMState.INSPECAO_FINALIZADA)

                # =================================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - ESCANEANDO - ANOMALIA_DETECTADA
                # =================================================================================
                if self.state[3] == FSMScanState.ANOMALIA_DETECTADA:
                    # Objetivo: Detectou anomalia durante escaneamento
                    self.get_logger().info("Anomalia detectada durante escaneamento!")
                    
                    # Transiciona para FOCANDO_ANOMALIA
                    self.anomaly_focus_start_time = time.time()
                    self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.ESCANEANDO, FSMScanState.FOCANDO_ANOMALIA)

                # =================================================================================
                # ESTADO: EXECUTANDO_MISSAO - INSPECIONANDO - ESCANEANDO - FOCANDO_ANOMALIA
                # =================================================================================
                if self.state[3] == FSMScanState.FOCANDO_ANOMALIA:
                    # Objetivo: Manter foco na anomalia por 5 segundos
                    elapsed_time = time.time() - self.anomaly_focus_start_time
                    remaining_time = 5.0 - elapsed_time
                    
                    self.get_logger().info(f"Focando anomalia... {remaining_time:.1f}s restantes", 
                                        throttle_duration_sec=1)
                    
                    if elapsed_time >= 5.0:
                        self.get_logger().info("5 segundos de foco concluídos. Retornando ao escaneamento.")
                        self.anomaly_detected = False
                        self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.ESCANEANDO, FSMScanState.SEM_ANOMALIA)


        # =================================================
        # ESTADO: INSPECAO_FINALIZADA
        # =================================================
        if self.state[0] == FSMState.INSPECAO_FINALIZADA:
            # Objetivo: Pausar por 3 segundos antes de retornar
            if self.inspection_pause_start_time == 0:
                self.inspection_pause_start_time = time.time()
                self.get_logger().info("Inspeção finalizada. Pausando por 3 segundos...")
            
            if time.time() - self.inspection_pause_start_time >= 3.0:
                self.get_logger().info("Pausa concluída. Iniciando retorno ao helideck.")
                self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.RETORNANDO)

        # =================================================
        # ESTADO: RETORNO_HELIDECK
        # =================================================
        if self.state[0] == FSMState.RETORNO_HELIDECK:
            if self.state[1] == FSMSubState.RETORNANDO:
                # Objetivo: Retornar para casa usando RTL
                # Verifica se já enviou o comando
                if not self.drone_state.startswith("POUSADO"):
                    self.get_logger().info("Enviando comando RTL...", throttle_duration_sec=2)
                    self.publish_drone_command({"command": "RTL"})
                else:
                    # Drone pousou, transiciona para DESARMANDO
                    self.get_logger().info("Drone pousou. Transicionando para DESARMANDO...")
                    self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.DESARMANDO)
                
            if self.state[1] == FSMSubState.DESARMANDO:
                # Objetivo: Desarmar motores e finalizar missão
                if self.drone_state.startswith("POUSADO"):
                    self.get_logger().info("Enviando comando DISARM...")
                    self.publish_drone_command({"command": "DISARM"})
                    self.get_logger().info("Missão concluída com sucesso. Reiniciando FSM para próxima missão.")
                    self.reset_FSM()


    # ==================================================================
    # CHECAGEM ESSENCIAIS DE TÓPICOS
    # ==================================================================

    def check_essential_topics(self):
        """
        Verifica a saúde dos tópicos essenciais.
        Executado por um timer a cada 2 segundos.
        
        Verifica se algum tópico essencial não foi recebido nos últimos 2 segundos.
        Se algum tópico estiver inativo e o estado atual não for DESATIVADO,
        muda para DESATIVADO e envia comando para Position mode.
        """
        # Verifica se algum tópico essencial não foi recebido nos últimos 2 segundos
        current_time = time.time()
        unhealthy_topics = []  # Variável local apenas para esta função
        
        for topic_name, topic_info in self.essencial_topics.items():
            # Verifica timeout para todos os tópicos ou se o tópico nunca foi recebido
            time_since_last = current_time - topic_info["last_received"]
            if time_since_last > self.topic_health_timeout or topic_info["last_received"] == 0.0:
                unhealthy_topics.append(topic_name)
        
        # Se algum tópico está inativo, reseta a FSM
        if len(unhealthy_topics) > 0:
            unhealthy_list = ", ".join([f"{name} ({self.essencial_topics[name]['description']})" for name in unhealthy_topics])
            self.get_logger().error(f"Tópicos essenciais inativos detectados: {unhealthy_list}. Resetando FSM....")
            self.reset_FSM()
        

    def set_state(self, level0, level1=None, level2=None, level3=None):
        """
        Função central para atualizar o estado da FSM usando IntEnum.
        
        Args:
            level0: Estado principal (FSMState)
            level1: Sub-estado nível 1 (FSMSubState) - opcional
            level2: Sub-estado nível 2 (FSMInspectionState) - opcional
            level3: Sub-estado nível 3 (FSMScanState) - opcional
        
        Exemplos:
            - set_state(FSMState.DESATIVADO)
            - set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.ARMANDO)
            - set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.DETECTANDO)
        """
        # Preenche níveis não especificados com NENHUM
        self.state = [
            level0,
            level1 if level1 is not None else FSMSubState.NENHUM,
            level2 if level2 is not None else FSMInspectionState.NENHUM,
            level3 if level3 is not None else FSMScanState.NENHUM
        ]
        
        # Limpa variáveis de aguardo ao mudar de estado
        self.waiting_for_state = None
        
        # Constrói string do estado para logging (usa .name para IntEnum)
        state_names = []
        for s in self.state:
            if hasattr(s, 'name') and s != 0:  # IntEnum com valor != 0
                state_names.append(s.name)
        
        state_str = " -> ".join(state_names) if state_names else "DESATIVADO"
        self.get_logger().info(f"NOVO ESTADO FSM: {state_str}")
        
        # Publica estado para o dashboard
        msg = String()
        msg.data = state_str
        self.fsm_state_pub.publish(msg)


    def get_state_name(self, level=0):
        """
        Retorna o nome do estado em um nível específico da hierarquia.
        Útil para logging e debug.
        
        Args:
            level: Índice do nível (0 = estado principal, 1 = sub-estado, etc.)
            
        Returns:
            str: Nome do estado no nível especificado, ou "NENHUM" se não existir
        """
        if level < len(self.state):
            state = self.state[level]
            return state.name if hasattr(state, 'name') else str(state)
        return "NENHUM"


    def publish_drone_command(self, command_dict):
        """
        Publica um comando para o drone_node em formato JSON, evitando repetições consecutivas do mesmo comando.
        
        Args:
            command_dict: Dicionário com o comando a ser enviado. Deve conter pelo menos a chave "command".
                         Exemplos:
                         - {"command": "ARM"}
                         - {"command": "GOTO", "lat": 80.0, "lon": 0.0, "alt": 99.0}
                         - {"command": "GOTO", "lat": 80.0, "lon": 0.0, "alt": 99.0, "yaw": 45.0}  # Com yaw alvo opcional
        """
        # Converte dicionário para JSON string
        command_json = json.dumps(command_dict)
        
        msg = String()
        msg.data = command_json

        self.get_logger().info(f"--> Comando para DroneNode (JSON): {command_json}")
        self.fsm_command_pub.publish(msg)

    def send_command_and_wait(self, command_dict, expected_state, timeout=10.0):
        """
        Envia comando para o drone_node e configura aguardo de estado.
        Usado para comandos críticos onde a FSM precisa aguardar confirmação.
        
        Args:
            command_dict: Dicionário do comando (ex: {"command": "ARM"})
            expected_state: Estado esperado do drone após o comando (string, ex: "POUSADO_ARMADO")
            timeout: Timeout em segundos para aguardar o estado
        
        Returns:
            bool: True se o comando foi enviado, False se já está aguardando outro estado
        """
        # Se já está aguardando um estado, não envia novo comando
        if self.waiting_for_state is not None:
            self.get_logger().warn(f"Já está aguardando estado {self.waiting_for_state}. Comando ignorado.")
            return False
        
        self.publish_drone_command(command_dict)
        self.waiting_for_state = expected_state
        self.command_sent_time = time.time()
        self.command_timeout = timeout
        self.get_logger().info(f"Comando enviado. Aguardando estado: {expected_state} (timeout: {timeout}s)")
        return True

    def check_waiting_state(self):
        """
        Verifica se o estado esperado foi alcançado após enviar um comando.
        Deve ser chamado no loop principal da FSM para estados que aguardam confirmação.
        
        Returns:
            str: "waiting" se ainda aguardando, "success" se alcançou o estado, "timeout" se expirou
        """
        if self.waiting_for_state is None:
            return "success"
        
        # Verifica se alcançou o estado esperado
        if self.drone_state == self.waiting_for_state:
            self.get_logger().info(f"Estado {self.waiting_for_state} alcançado!")
            self.waiting_for_state = None
            return "success"
        
        # Verifica timeout
        elapsed = time.time() - self.command_sent_time
        if elapsed > self.command_timeout:
            self.get_logger().error(f"Timeout ({self.command_timeout}s) aguardando estado {self.waiting_for_state}. Estado atual: {self.drone_state}")
            self.waiting_for_state = None
            return "timeout"
        
        return "waiting"

    def send_yaw_command(self, target_yaw):
        """
        Simula comando SET_YAW usando GOTO na posição atual com yaw específico.
        Só funciona se o drone estiver no estado VOANDO_PRONTO.
        
        Args:
            target_yaw: Yaw alvo em graus (0-360)
        
        Returns:
            bool: True se o comando foi enviado, False se não foi possível
        """
        if self.drone_state != "VOANDO_PRONTO":
            self.get_logger().warn(f"Não é possível girar: drone não está VOANDO_PRONTO (estado: {self.drone_state})")
            return False
        
        self.publish_drone_command({
            "command": "GOTO",
            "lat": self.drone_global_position_lat,
            "lon": self.drone_global_position_lon,
            "alt": self.drone_global_position_alt,
            "yaw": target_yaw
        })
        return True

    # ==================================================================
    # CALLBACKS DE EVENTOS
    # ==================================================================

    def dashboard_command_callback(self, msg):
        """
        Recebe comandos do dashboard para controlar a missão.
        
        Formato esperado: JSON contendo:
        {
            "command": "<nome_do_comando>",
            ... outras variáveis específicas do comando ...
        }
        
        COMANDOS SUPORTADOS:
        - "start_inspection": Inicia a missão de inspeção (apenas no estado PRONTO)
            Variáveis opcionais:
                - "inspection_type" (str): Tipo de inspeção (ex: "Flare", "Tanques de GLP", etc.)
        - "stop_inspection": Pausa a missão atual
        - "cancel_inspection": Cancela completamente a missão
        - "return_to_base": Comanda o drone a retornar à base
        - "abort_mission": Aborta a missão e retorna para casa
        - "emergency_land": Pousa imediatamente
        """
        try:
            # Tenta decodificar como JSON
            command_data = json.loads(msg.data)
            command = command_data.get("command", "")
            
            self.get_logger().info(f"<-- Comando do Dashboard recebido (JSON): {msg.data}")
            
            # Processa o comando start_inspection
            if command == "start_inspection":
                inspection_type = command_data.get("inspection_type", "Flare")
                self.get_logger().info(f"Comando 'start_inspection' recebido do Dashboard. Tipo: {inspection_type}. Verificando condições...")
                if self.state[0] == FSMState.PRONTO and self.can_execute_mission():
                    self.mission_started = True
                    # Armazena o tipo de inspeção se necessário
                    self.inspection_type = inspection_type
                    self.get_logger().info(f"COMANDO 'START_INSPECTION' ACEITO. Tipo: {inspection_type}. INICIANDO MISSÃO.")
                else:
                    state_name = self.state[0].name if hasattr(self.state[0], 'name') else str(self.state[0])
                    self.get_logger().warn(f"COMANDO 'START_INSPECTION' RECUSADO. Estado atual: {state_name}. Condições de execução: {self.can_execute_mission()}")
            
            # Processa o comando stop_inspection
            elif command == "stop_inspection":
                self.get_logger().warn("COMANDO 'STOP_INSPECTION' RECEBIDO. PAUSANDO MISSÃO.")
                # TODO: Implementar lógica de pausa se necessário
                # Por enquanto, apenas loga o comando
            
            # Processa o comando cancel_inspection
            elif command == "cancel_inspection":
                self.get_logger().warn("COMANDO 'CANCEL_INSPECTION' RECEBIDO. CANCELANDO MISSÃO.")
                self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.RETORNANDO)
            
            # Processa o comando return_to_base
            elif command == "return_to_base":
                self.get_logger().warn("COMANDO 'RETURN_TO_BASE' RECEBIDO. RETORNANDO À BASE.")
                self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.RETORNANDO)
            
            # Processa o comando abort_mission
            elif command == "abort_mission":
                self.get_logger().warn("COMANDO 'ABORT_MISSION' RECEBIDO. ABORTANDO MISSÃO.")
                self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.RETORNANDO)
            
            # Processa o comando emergency_land
            elif command == "emergency_land":
                self.get_logger().error("COMANDO 'EMERGENCY_LAND' RECEBIDO. POUSANDO IMEDIATAMENTE.")
                self.publish_drone_command({"command": "LAND"})
            
            else:
                self.get_logger().warn(f"Comando desconhecido recebido: {command}")
                
        except json.JSONDecodeError:
            # Fallback para compatibilidade com formato antigo (string simples)
            command_str = msg.data
            self.get_logger().warn(f"Comando recebido não é JSON válido, tentando processar como string: {command_str}")
            
            # Processa comandos no formato antigo para compatibilidade
            if command_str.startswith("start_inspection"):
                parts = command_str.split(":", 1)
                inspection_type = parts[1] if len(parts) > 1 else "Flare"
                if self.state[0] == FSMState.PRONTO and self.can_execute_mission():
                    self.mission_started = True
                    self.inspection_type = inspection_type
                    self.get_logger().info(f"COMANDO 'START_INSPECTION' ACEITO (formato antigo). Tipo: {inspection_type}.")
            elif command_str == "abort_mission":
                self.set_state(FSMState.RETORNO_HELIDECK, FSMSubState.RETORNANDO)
            elif command_str == "emergency_land":
                self.publish_drone_command({"command": "LAND"})

    def drone_status_callback(self, msg):
        """
        Processa feedback do drone_node para transitar entre estados.
        Recebe dados como JSON contendo todas as variáveis de status do drone.
        
        CAMPOS JSON RECEBIDOS DO DRONE_NODE:
        - "offboard_mode_control" (bool): Indica se o modo OFFBOARD está ativo
        - "drone_armed" (bool): Indica se os motores estão armados
        - "drone_arming" (bool): Indica se os motores estão sendo armados
        - "drone_on_trajectory" (bool): Indica se o drone está seguindo uma trajetória
        - "drone_local_position_x/y/z" (float): Posição local (metros, coordenadas NED)
        - "drone_global_position_lat/lon/alt" (float): Posição global (GPS)
        - "drone_yaw_deg" (float): Yaw atual em graus (0-360)
        - "drone_state" (str): Estado da máquina de estados do drone (POUSADO_*, VOANDO_*, EMERGENCIA)
        - "drone_sub_state" (str): Sub-estado da trajetória (NENHUM, DECOLANDO, A_CAMINHO, etc.)
        - "state_duration_sec" (float): Duração no estado atual em segundos
        
        A FSM usa drone_state para aguardar confirmação de comandos enviados ao drone_node.
        """
        # Atualiza timestamp do tópico essencial para verificação de saúde
        self.essencial_topics["drone_status"]["last_received"] = time.time()

        # Decodifica mensagem JSON recebida do drone_node
        try:
            status_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Status recebido não é JSON válido: {msg.data}")
            return
        
        # ==================================================================
        # ATUALIZAÇÃO DE VARIÁVEIS DE STATUS DIRETAS DO DRONE_NODE
        # ==================================================================
        # Lê campos JSON publicados pelo drone_node e atualiza variáveis internas
        
        # Status de controle e armamento
        self.drone_offboard_mode = status_data.get("offboard_mode_control", False)
        self.drone_armed = status_data.get("drone_armed", False)
        self.drone_arming = status_data.get("drone_arming", False)
        self.drone_landed = status_data.get("drone_landed", False)
        self.drone_on_trajectory = status_data.get("drone_on_trajectory", False)
        
        # Posição local (coordenadas NED em metros, relativa ao ponto de partida)
        self.drone_local_position_x = status_data.get("drone_local_position_x", 0.0)
        self.drone_local_position_y = status_data.get("drone_local_position_y", 0.0)
        self.drone_local_position_z = status_data.get("drone_local_position_z", 0.0)
        
        # Posição global (coordenadas GPS: latitude, longitude em graus, altitude em metros)
        self.drone_global_position_lat = status_data.get("drone_global_position_lat", 0.0)
        self.drone_global_position_lon = status_data.get("drone_global_position_lon", 0.0)
        self.drone_global_position_alt = status_data.get("drone_global_position_alt", 0.0)
        
        # Orientação (yaw em graus, normalizado para 0-360)
        self.drone_yaw = status_data.get("drone_yaw_deg", 0.0)
        
        # Estado do drone (novos campos da máquina de estados do drone_node)
        self.drone_state = status_data.get("drone_state", "POUSADO_DESARMADO")
        self.drone_sub_state = status_data.get("drone_sub_state", "NENHUM")
        self.drone_state_duration = status_data.get("state_duration_sec", 0.0)


    def cv_detection_callback(self, msg):
        """
        Processa detecções do cv_node durante a inspeção.
        
        FORMATO ESPERADO DO JSON:
        {
            "timestamp": "ISO timestamp",
            "detections": [
                {
                    "object_type": "flare" | "anomalia",
                    "class": "nome_da_classe",
                    "confidence": 0.0-1.0,
                    "bbox": [x1, y1, x2, y2],
                    "bbox_center": [cx, cy]
                }
            ],
            "count": número_de_deteccoes
        }
        """
        # Atualiza timestamp do tópico essencial
        #self.essencial_topics["cv_detections"]["last_received"] = time.time()
        
        # Só processa detecções durante estados relevantes de inspeção
        if self.state[2] not in [FSMInspectionState.DETECTANDO, FSMInspectionState.CENTRALIZANDO, FSMInspectionState.ESCANEANDO]:
            return
            
        try:
            detection_data = json.loads(msg.data)
            # Extrai a lista de detecções do JSON
            detections = detection_data.get("detections", [])
            flare_detected_in_frame = False
            
            for detection in detections:
                object_type = detection.get("object_type", "")
                confidence = detection.get("confidence", 0.0)
                bbox_center = detection.get("bbox_center", None)
                
                # Processa detecção de flare durante busca
                if object_type == "flare" and self.state[2] == FSMInspectionState.DETECTANDO:
                    flare_detected_in_frame = True
                    self.flare_detected = True
                    self.get_logger().info(f"!!! FLARE DETECTADO !!! (confiança: {confidence:.2f})")
                    
                    # Armazena o centro do bounding box
                    if bbox_center:
                        self.target_bbox_center = bbox_center
                        # Calcula o offset de yaw necessário para centralizar o target
                        image_center_x = 320.0  # Assumindo imagem de 640px de largura
                        pixel_offset = bbox_center[0] - image_center_x
                        
                        # Converte offset em pixels para offset em graus
                        fov_horizontal = 60.0  # graus
                        image_width = 640.0  # pixels
                        degrees_per_pixel = fov_horizontal / image_width
                        self.target_yaw_offset = pixel_offset * degrees_per_pixel
                        
                        self.get_logger().info(f"Centro do bounding box: {bbox_center}, Offset de yaw: {self.target_yaw_offset:.1f}°")
                    
                    # Transiciona para centralização
                    self.target_centralized = False
                    self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.CENTRALIZANDO)
                    break
                
                # Atualiza centralização durante estado CENTRALIZANDO
                if object_type == "flare" and self.state[2] == FSMInspectionState.CENTRALIZANDO and bbox_center:
                    # Atualiza continuamente o offset de yaw baseado na posição atual do bounding box
                    image_center_x = 320.0
                    pixel_offset = bbox_center[0] - image_center_x
                    fov_horizontal = 60.0
                    image_width = 640.0
                    degrees_per_pixel = fov_horizontal / image_width
                    self.target_yaw_offset = pixel_offset * degrees_per_pixel
                    self.target_bbox_center = bbox_center
                    
                    # Se o offset for muito pequeno, considera centralizado
                    if abs(self.target_yaw_offset) <= self.centralization_tolerance:
                        self.target_centralized = True
                        self.get_logger().info(f"Target centralizado! Offset: {self.target_yaw_offset:.1f}°")
                
                # Processa detecção de anomalia durante escaneamento
                if object_type == "anomalia" and self.state[2] == FSMInspectionState.ESCANEANDO:
                    # Verifica critério de 30° para evitar detecção repetida da mesma anomalia
                    angle_diff = abs(self.current_inspection_angle - self.last_anomaly_angle)
                    if angle_diff > 30 or self.last_anomaly_angle == 0.0:
                        self.anomaly_detected = True
                        self.last_anomaly_angle = self.current_inspection_angle
                        self.get_logger().info(f"!!! ANOMALIA DETECTADA !!! Ângulo: {self.current_inspection_angle:.1f}° (confiança: {confidence:.2f})")
                        self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.ESCANEANDO, FSMScanState.ANOMALIA_DETECTADA)
                        break
            
            # Verifica se completou giro de 360° sem detectar flare
            if (self.state[2] == FSMInspectionState.DETECTANDO and 
                not flare_detected_in_frame and 
                self.flare_detection_yaw_covered >= 360.0):
                self.get_logger().warn("Flare não detectado após giro completo de 360°.")
                self.set_state(FSMState.EXECUTANDO_MISSAO, FSMSubState.INSPECIONANDO, FSMInspectionState.FALHA_DE_DETECCAO)

        except json.JSONDecodeError:
            self.get_logger().error("Erro ao decodificar JSON de detecção do cv_node.")


    def reset_FSM(self):
        """
        Reseta todas as variáveis de estado para uma nova missão.
        Chamado após completar uma missão para preparar a próxima.
        """
        # --- Reset dos Estados (usando IntEnum) ---
        self.state = [FSMState.DESATIVADO, FSMSubState.NENHUM, FSMInspectionState.NENHUM, FSMScanState.NENHUM]
        self.state_anterior = [FSMState.DESATIVADO, FSMSubState.NENHUM, FSMInspectionState.NENHUM, FSMScanState.NENHUM]
        
        # --- Controle de Aguardo de Comando ---
        self.waiting_for_state = None
        self.command_sent_time = 0.0
        self.command_timeout = 10.0

        # --- Variáveis de Controle (drone_node) ---
        self.drone_offboard_mode = False
        self.drone_mode = None
        self.drone_arming = False
        self.drone_armed = False
        self.drone_landed = False
        self.drone_on_trajectory = False
        self.drone_reached_waypoint = False
        self.drone_local_position_x = 0.0
        self.drone_local_position_y = 0.0
        self.drone_local_position_z = 0.0
        self.drone_global_position_lat = 0.0
        self.drone_global_position_lon = 0.0
        self.drone_global_position_alt = 0.0
        self.drone_yaw = 0.0
        
        # --- Estado do Drone (da máquina de estados do drone_node) ---
        self.drone_state = "POUSADO_DESARMADO"
        self.drone_sub_state = "NENHUM"
        self.drone_state_duration = 0.0
        
        # --- Parâmetros da Missão ---
        self.mission_started = False
        self.inspection_type = "Flare"
        self.inspection_point_lat = 80.0
        self.inspection_point_lon = 0.0
        self.inspection_point_alt = 99.0
        self.target_lat = 92.0
        self.target_lon = 0.0
        self.target_alt = 99.0
        
        # --- Parâmetros do ESCANEAMENTO ---
        self.current_scanning_angle = 0.0
        self.anomaly_angles_list = []
        
        # --- Estados de Controle Interno ---
        self.target_detected = False
        self.anomaly_detected = False
        
        # --- Variáveis de Detecção de Flare ---
        self.flare_detected = False
        self.flare_detection_start_yaw = None
        self.flare_detection_yaw_covered = 0.0
        self.flare_detection_current_target_yaw = None
        self.flare_detection_yaw_reached = False
        
        # --- Variáveis de Centralização do Target ---
        self.target_centralized = False
        self.target_bbox_center = None
        self.target_yaw_offset = 0.0
        
        # --- Variáveis de Estabilidade e Inspeção ---
        self.is_stable = False
        self.stability_check_start_time = 0
        self.inspection_completed = False
        self.inspection_start_angle = 0.0
        self.current_inspection_angle = 0.0
        self.last_anomaly_angle = 0.0
        self.failure_pause_start_time = 0
        self.inspection_pause_start_time = 0
        
        self.get_logger().info("\033[1m\033[93mMáquina de Estados (FSM) resetada!\033[0m")

    def calculate_yaw_to_point(self, target_lat, target_lon):
        """
        Calcula o yaw necessário para apontar para um ponto GPS.
        
        Args:
            target_lat: Latitude do alvo
            target_lon: Longitude do alvo
            
        Returns:
            float: Ângulo de yaw em graus
            
        Nota: Esta é uma implementação simplificada. Em um sistema real,
        seria necessário usar a posição atual do drone para calcular o bearing correto.
        """
        # Implementação placeholder - em sistema real usaria posição atual do drone
        # e cálculo de bearing geodésico
        return 90.0  # Aponta para o Norte como exemplo

    def is_at_waypoint(self, waypoint_lat, waypoint_lon, waypoint_alt):
        """
        Verifica se o drone chegou ao waypoint especificado comparando posições GPS.
        
        Args:
            waypoint_lat: Latitude do waypoint (graus)
            waypoint_lon: Longitude do waypoint (graus)
            waypoint_alt: Altitude do waypoint (metros)
        
        Returns:
            bool: True se o drone está dentro da tolerância do waypoint e não está mais em trajetória, False caso contrário
        """
        # Verifica se as posições são válidas
        if (self.drone_global_position_lat == 0.0 or self.drone_global_position_lon == 0.0 or 
            waypoint_lat == 0.0 or waypoint_lon == 0.0):
            return False
        
        # Converte diferença de latitude/longitude para metros
        # Aproximação: 1 grau de latitude ≈ 111000 metros
        lat_diff = abs(self.drone_global_position_lat - waypoint_lat) * 111000  # metros
        lon_diff = abs(self.drone_global_position_lon - waypoint_lon) * 111000 * math.cos(math.radians(self.drone_global_position_lat))
        
        # Calcula distância 2D (horizontal) e 3D (incluindo altitude)
        distance_2d = math.sqrt(lat_diff**2 + lon_diff**2)
        alt_diff = abs(self.drone_global_position_alt - waypoint_alt)
        distance_3d = math.sqrt(distance_2d**2 + alt_diff**2)
        
        # Considera que chegou ao destino se está dentro da tolerância e não está mais em trajetória
        return distance_3d <= self.waypoint_tolerance and not self.drone_on_trajectory

    def can_execute_mission(self):
        """
        Verifica se todas as condições necessárias para iniciar a missão são atendidas.
        """
        # Verifica se os tópicos essenciais foram confirmados (estão saudáveis)
        current_time = time.time()
        topics_confirmed = True
        
        for topic_name, topic_info in self.essencial_topics.items():
            # Verifica timeout para todos os tópicos ou se o tópico nunca foi recebido
            time_since_last = current_time - topic_info["last_received"]
            if time_since_last > self.topic_health_timeout or topic_info["last_received"] == 0.0:
                topics_confirmed = False
                break
        
        # Adicione outras condições aqui conforme necessário (ex: drone em posição segura, bateria ok)
        # Por exemplo: self.is_drone_safe and self.battery_level > 0.2
        
        return topics_confirmed


def main(args=None):
    """Função principal do nó."""
    rclpy.init(args=args)
    fsm_node = FSMNode()
    
    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

