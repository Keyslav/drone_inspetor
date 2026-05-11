# drone_node.py — Manual de Inicialização

> **Papel no sistema:** Interface exclusiva com a controladora de voo PX4. Traduz comandos de alto nível da FSM em mensagens PX4 e republica telemetria simplificada para o restante do sistema. Não contém lógica de missão.

---

## Sumário

1. [Arquitetura da classe](#1-arquitetura-da-classe)
2. [Função `main()` — ponto de entrada](#2-função-main--ponto-de-entrada)
3. [Sequência de inicialização (`__init__`)](#3-sequência-de-inicialização-__init__)
   - [3.1 Inicialização do nó ROS2](#31-inicialização-do-nó-ros2)
   - [3.2 Configuração de QoS](#32-configuração-de-qos)
   - [3.3 Subscribers — escutando o PX4 e o LiDAR](#33-subscribers--escutando-o-px4-e-o-lidar)
   - [3.4 Action Server — canal de comando da FSM](#34-action-server--canal-de-comando-da-fsm)
   - [3.5 Publishers — enviando para o PX4 e para o sistema](#35-publishers--enviando-para-o-px4-e-para-o-sistema)
   - [3.6 Máquina de estados (`DroneFSMContext`)](#36-máquina-de-estados-dronefsmcontext)
   - [3.7 Timers — ações periódicas](#37-timers--ações-periódicas)
4. [Estado inicial do sistema ao finalizar `__init__`](#4-estado-inicial-do-sistema-ao-finalizar-__init__)
5. [Fluxo de dados em operação normal](#5-fluxo-de-dados-em-operação-normal)
6. [Shutdown](#6-shutdown)
7. [Lógica de recepção de comandos do Mission Node](#7-lógica-de-recepção-de-comandos-do-mission-node)
   - [7.1 Ciclo de vida de um comando](#71-ciclo-de-vida-de-um-comando)
   - [7.2 Fluxo detalhado por comando](#72-fluxo-detalhado-por-comando)
   - [7.3 Feedback e conclusão](#73-feedback-e-conclusão)
8. [Sistema de desvio de obstáculos](#8-sistema-de-desvio-de-obstáculos)
   - [8.1 Fontes de detecção](#81-fontes-de-detecção)
   - [8.2 Influência na velocidade (TrajectoryProfile)](#82-influência-na-velocidade-trajectoryprofile)
   - [8.3 Sub-FSM de desvio — 5 fases](#83-sub-fsm-de-desvio--5-fases)
   - [8.4 Proteção anti-loop](#84-proteção-anti-loop)
9. [Verificações globais pré-ciclo da FSM](#9-verificações-globais-pré-ciclo-da-fsm)
10. [Perfil trapezoidal de velocidade](#10-perfil-trapezoidal-de-velocidade)

---

## 1. Arquitetura da classe

`DroneNode` usa herança múltipla para manter o código organizado em responsabilidades separadas:

```
DroneNode
├── DroneTrajectoryMixin    — cálculo de setpoints de trajetória e conversão GPS↔local
├── DroneActionServerMixin  — callbacks do ActionServer (goal, cancel, execute, feedback)
├── DronePX4CommandsMixin   — métodos de alto nível (arm, takeoff, goto, land, rtl, stop)
└── Node                    — classe base ROS2
```

Os mixins compartilham estado via `self.drone_context` (instância de `DroneFSMContext`) e `self.get_logger()` / `self.get_clock()` do `Node`.

**Arquivos envolvidos:**

| Arquivo | Responsabilidade |
|---|---|
| `drone_node.py` | Classe principal: subscribers, publishers, timers, callbacks de telemetria |
| `fsm/machine.py` / `fsm/context.py` | `DroneFSM` / `DroneFSMContext` — transições automáticas, validação de comandos, container de dados |
| `drone_px4_state.py` | `DroneStatePX4` — container de toda telemetria recebida do PX4 |
| `drone_obstacles.py` | `DroneObstacles` — flags LiDAR e cálculo de desvio de trajetória |
| `drone_trajectory.py` | `DroneTrajectoryMixin` — geração de `TrajectorySetpoint` e conversão GPS |
| `drone_action_server.py` | `DroneActionServerMixin` — lógica completa do ActionServer |
| `drone_px4_commands.py` | `DronePX4CommandsMixin` — publicação de `VehicleCommand` PX4 |

---

## 2. Função `main()` — ponto de entrada

```python
def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    executor = MultiThreadedExecutor()
    executor.add_node(drone_node)
    executor.spin()
```

**Por que `MultiThreadedExecutor`?**
O ActionServer executa um loop de feedback bloqueante (`while not _is_command_complete`) na thread da action. Se usasse `SingleThreadedExecutor`, esse loop impediria os callbacks de telemetria PX4 de serem processados, paralisando todo o sistema. Com `MultiThreadedExecutor` + `ReentrantCallbackGroup` no ActionServer, a execução da action roda em uma thread e os callbacks de telemetria rodam em outras.

---

## 3. Sequência de inicialização (`__init__`)

### 3.1 Inicialização do nó ROS2

```python
super().__init__("drone_node")
self._is_drone_node_shutting_down = False
```

- Registra o nó no grafo ROS2 com o nome `"drone_node"`.
- `_is_drone_node_shutting_down` é uma flag de segurança usada para ignorar erros durante o encerramento do processo (ex: erros de publicação de feedback após o contexto ROS2 já ter sido destruído).

---

### 3.2 Configuração de QoS

QoS não é configurado manualmente — vem amarrado no `TopicSpec` de cada tópico (em `drone_inspetor/ros_interfaces/`). Os call-sites usam helpers que já leem o QoS correto do spec, eliminando o risco de pub/sub divergirem.

Perfis usados em `drone_node.py` (todos definidos em `drone_inspetor/ros_interfaces/qos.py`):

| Perfil | `Reliability` | `Durability` | `depth` | Usado em |
|---|---|---|---|---|
| `QoSProfiles.px4()` | BEST_EFFORT | TRANSIENT_LOCAL | 1 | Toda telemetria PX4 (`/fmu/out/*`, `/fmu/in/*`) — necessário para compatibilidade com uXRCE-DDS |
| `QoSProfiles.status()` | BEST_EFFORT | TRANSIENT_LOCAL | 1 | `DRONE_STATE`, `DRONE_BATTERY_STATUS` — estado interno |
| `QoSProfiles.commands_volatile(depth=10)` | RELIABLE | VOLATILE | 10 | `LIDAR_OBSTACLE_DETECTIONS` — comandos confiáveis sem persistência |

---

### 3.3 Subscribers — escutando o PX4 e o LiDAR

Todos os subscribers são criados via `create_subscription_from(self, Topics.<Categoria>.<NOME>, callback)` — o tipo da mensagem e o QoS vêm do spec, garantindo que pub/sub não possam divergir.

#### Telemetria PX4 (`/fmu/out/...`)

| Spec | Mensagem | Callback | O que extrai |
|---|---|---|---|
| `Topics.PX4.VEHICLE_STATUS` | `VehicleStatus` | `vehicle_status_callback` | Estado de armamento (`arming_state`), modo de navegação (`nav_state`). Detecta transição OFFBOARD ON/OFF. |
| `Topics.PX4.VEHICLE_COMMAND_ACK` | `VehicleCommandAck` | `command_ack_callback` | Confirmação de comandos enviados. Só loga ACKs de comandos no conjunto `pending_commands`. |
| `Topics.PX4.VEHICLE_LOCAL_POSITION` | `VehicleLocalPosition` | `vehicle_local_position_callback` | Posição NED (x, y, z em metros), velocidade (vx, vy, vz) e aceleração (ax, ay, az). |
| `Topics.PX4.VEHICLE_GLOBAL_POSITION` | `VehicleGlobalPosition` | `vehicle_global_position_callback` | Posição GPS (lat, lon, alt). |
| `Topics.PX4.HOME_POSITION` | `HomePosition` | `home_position_callback` | Posição de home: GPS (lat, lon, alt), local (x, y, z) e yaw (rad). Calcula e armazena yaw em três formatos: rad, graus 0-360 e graus -180/180. |
| `Topics.PX4.VEHICLE_ATTITUDE` | `VehicleAttitude` | `vehicle_attitude_callback` | Quaternião (q[0]=w, q[1]=x, q[2]=y, q[3]=z). Converte para yaw em rad, graus 0-360 e graus -180/180. |
| `Topics.PX4.VEHICLE_LAND_DETECTED` | `VehicleLandDetected` | `land_detected_callback` | Flag booleana `landed` — indica se o drone tocou o solo. |
| `Topics.PX4.BATTERY_STATUS` | `BatteryStatus` | `battery_status_callback` | Estado da bateria. Republica diretamente para `Topics.Interno.DRONE_BATTERY_STATUS`. |

#### LiDAR interno

| Spec | Mensagem | Callback | O que faz |
|---|---|---|---|
| `Topics.Interno.LIDAR_OBSTACLE_DETECTIONS` | `ObstaclesMSG` | `lidar_obstacles_callback` | Chama `drone_context.obstacles.update_from_lidar(msg)` — alimenta o buffer da fonte 'lidar'. |
| `Topics.Interno.DEPTH_OBSTACLE_DETECTIONS` | `ObstaclesMSG` | `depth_obstacles_callback` | Chama `drone_context.obstacles.update_from_depth(msg)` — alimenta o buffer da fonte 'depth' (cobertura frontal). As flags públicas em `DroneObstacles` retornam OR entre as fontes. |

---

### 3.4 Action Server — canal de comando do Mission Node

```python
self._action_callback_group = ReentrantCallbackGroup()
self._action_server = make_action_server(
    self,
    Topics.Action.DRONE_COMMAND,           # /drone_inspetor/action/drone_command
    execute_callback=self.execute_drone_command_callback,
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback,
    callback_group=self._action_callback_group,
)
self._current_goal_handle = None
self._action_cancelled = False
```

`make_action_server` é o helper de `drone_inspetor.ros_interfaces` que recebe o `ActionSpec` e instancia o `ActionServer` com tipo + nome corretos.

O ActionServer substitui o subscriber simples de tópico para permitir:
- **Feedback periódico** durante a execução (estado atual, distância ao alvo, progresso em %).
- **Resultado final** com `success`, `message` e `final_state`.
- **Cancelamento** via `cancel_callback` — executa `stop()` e aceita o cancelamento.

**Comandos suportados pela action `DroneCommand`:**

| Comando | Parâmetros | Estado de conclusão |
|---|---|---|
| `ARM` | — | `POUSADO_ARMADO` |
| `TAKEOFF` | `altitude` (metros) | `VOANDO_PRONTO` |
| `GOTO` (sem foco) | `lat`, `lon`, `alt`, `yaw` (todos opcionais via NaN) | `VOANDO_PRONTO` com flag limpa |
| `GOTO` (com foco) | `lat`, `lon`, `alt`, `use_focus=true`, `focus_lat`, `focus_lon` | `VOANDO_PRONTO` com flag limpa |
| `LAND` | — | `POUSADO_ARMADO` ou `POUSADO_DESARMADO` |
| `RTL` | — | `POUSADO_DESARMADO` |
| `STOP` | — | `VOANDO_PRONTO` |

O comando `GOTO` é único: a flag booleana `use_focus` no goal seleciona o subgrafo da FSM. Quando `use_focus=true`, o yaw é mantido apontando ao ponto `focus_lat`/`focus_lon` durante toda a trajetória (campo `yaw` é ignorado). Quando `use_focus=false` (padrão), o `yaw` é o ângulo final no destino.

**Timeouts por comando:** ARM=10s, STOP=5s, TAKEOFF/LAND=60s, GOTO=120s, RTL=180s.

---

### 3.5 Publishers — enviando para o PX4 e para o sistema

Todos criados via `create_publisher_from(self, Topics.<Categoria>.<NOME>)`. Tipo + QoS vêm amarrados no spec.

| Variável | Spec | Mensagem | Propósito |
|---|---|---|---|
| `vehicle_command_pub` | `Topics.PX4.VEHICLE_COMMAND` | `VehicleCommand` | Envia comandos diretos ao PX4 (ARM, TAKEOFF, LAND, RTL, SET_MODE...) |
| `offboard_control_mode_pub` | `Topics.PX4.OFFBOARD_CONTROL_MODE` | `OffboardControlMode` | Mantém o modo Offboard ativo (deve ser enviado a 50 Hz) |
| `trajectory_setpoint_pub` | `Topics.PX4.TRAJECTORY_SETPOINT` | `TrajectorySetpoint` | Envia posição/yaw alvo para o controlador PX4 em modo Offboard |
| `drone_state_pub` | `Topics.Interno.DRONE_STATE` | `DroneStateMSG` | Estado completo do drone para Mission Node e Dashboard (a 2 Hz) |
| `battery_status_pub` | `Topics.Interno.DRONE_BATTERY_STATUS` | `BatteryStatus` | Republica status da bateria para o Dashboard |

---

### 3.6 Máquina de estados (`DroneFSMContext`)

```python
self.declare_parameter("step_distance", 5.0)
step_distance = self.get_parameter("step_distance").get_parameter_value().double_value
self.drone_context = DroneFSMContext(self, step_distance=step_distance)
self.pending_commands = set()
```

**`step_distance`** (padrão 5.0m, configurável em `config/param_ros.yaml`) define o tamanho do passo incremental de trajetória — o quanto o drone avança em cada ciclo antes de recalcular.

A instância `DroneFSMContext` é o **núcleo de estado** do nó. Dentro do seu `__init__`, ela cria:

#### `DroneStatePX4` (container de telemetria)
Inicializado com todos os valores a `None` ou `False`. Atualizado pelos callbacks de telemetria:
- `nav_state`, `is_armed`, `is_landed` — estado do veículo
- `local_position`, `global_position`, `home_position` — navegação
- `current_yaw_rad/deg/deg_normalized` — orientação calculada dos quaterniões
- `current_velocity_x/y/z`, `current_acceleration_x/y/z` — cinemática
- `home_global_lat/lon/alt`, `home_local_position`, `home_yaw_*` — referência HOME

#### `DroneObstacles` (detecção de obstáculos)
Flags inicializadas em `False`. Atualizadas pelo `lidar_obstacles_callback`:
- `have_obstacles_1/2/3/5/8` — obstáculos dentro de 1m, 2m, 3m, 5m, 8m
- `have_obstacles_front/right/back/left_90` — obstáculos por quadrante de 90°
- `have_down_obstacles_1/0.5` — obstáculos abaixo do drone

#### Estado inicial da FSM interna do drone

O estado inicial é `DroneStateDescription.OFFBOARD_DESATIVADO` (valor 2). O `drone_state_timer` verifica periodicamente a telemetria PX4 e transiciona automaticamente para outros estados conforme o modo de navegação muda.

**Tabela de estados do drone:**

| Estado | Valor | Descrição |
|---|---|---|
| `POUSADO_DESARMADO` | 0 | No chão, desarmado |
| `POUSADO_ARMADO` | 1 | No chão, armado |
| `OFFBOARD_DESATIVADO` | 2 | Modo manual/POSCTL (não Offboard) |
| `VOANDO_PRONTO` | 10 | Hover estável, aguardando comando |
| `VOANDO_DECOLANDO` | 20 | Subindo para altitude de decolagem |
| `VOANDO_GIRANDO_INICIO` | 21 | Girando para apontar ao destino (GOTO) |
| `VOANDO_A_CAMINHO` | 22 | Voando em direção ao destino (GOTO) |
| `VOANDO_GIRANDO_FIM` | 23 | Girando para yaw final (GOTO) |
| `VOANDO_GIRANDO_COM_FOCO` | 25 | Girando para apontar ao foco (GOTO com use_focus=true) |
| `VOANDO_A_CAMINHO_COM_FOCO` | 26 | Voando apontando ao foco (GOTO com use_focus=true) |
| `RETORNANDO_GIRANDO_INICIO` | 30 | Girando para direção do HOME (RTL) |
| `RETORNANDO_A_CAMINHO` | 31 | Voando para HOME (RTL) |
| `RETORNANDO_GIRANDO_FIM` | 32 | Girando para yaw final do HOME (RTL) |
| `POUSANDO` | 40 | Pouso em progresso (LAND/RTL) |
| `EMERGENCIA` | 99 | Failsafe/emergência ativa |

---

### 3.7 Timers — ações periódicas

Três timers são criados no final do `__init__`, iniciando imediatamente após o nó entrar em spin:

#### Timer 1: `offboard_control_mode_timer` — 50 Hz (0.02s)
**Callback:** `publish_offboard_control_mode`

Publica uma mensagem `OffboardControlMode` com `position=True` e todos os outros campos `False`. Isso mantém o PX4 ciente de que existe um controlador externo ativo em modo de posição.

> **Condição de guarda:** só publica se `nav_state == NAVIGATION_STATE_OFFBOARD`. Se o modo não for Offboard, não publica e não desperdiça banda.

O PX4 exige que este tópico seja publicado continuamente a pelo menos 2 Hz para manter o modo Offboard ativo. Publicar a 50 Hz garante margem suficiente.

#### Timer 2: `trajectory_setpoint_timer` — 50 Hz (0.02s)
**Callback:** `publish_trajectory_setpoint`

Publica setpoints de posição para o controlador PX4. Só executa se:
1. `nav_state == NAVIGATION_STATE_OFFBOARD`
2. Posição global e local já foram recebidas (`global_position is not None` e `local_position is not None`)

Se `drone_context.on_trajectory == True` e há `target_local_position` definido, chama `create_moving_trajectory_setpoint()` (que calcula o próximo passo incremental baseado no estado atual da trajetória).

Se `on_trajectory == False` (hover), nenhum setpoint é enviado — o PX4 mantém a última posição recebida automaticamente.

#### Timer 3: `drone_state_timer` — 2 Hz (0.5s)
**Callback:** `update_and_publish_drone_state`

Executa duas ações em sequência:
1. `drone_fsm.tick()` — verifica condições para transição de estado (ex: chegou ao destino? o yaw está alinhado? pousou?) e transiciona automaticamente.
2. `publish_drone_status()` — monta e publica a mensagem `DroneStateMSG` completa com toda a telemetria atual, estado, posições (atual, alvo, home, estática) e flags. Essa mensagem é consumida pela FSM e pelo Dashboard.

---

## 4. Estado inicial do sistema ao finalizar `__init__`

Ao fim do `__init__`, o sistema está assim:

```
drone_node
│
├── Estado: OFFBOARD_DESATIVADO
│
├── DroneStatePX4
│   ├── nav_state: None
│   ├── is_armed: False
│   ├── is_landed: False
│   ├── local_position: None      ← aguardando primeiro callback PX4
│   ├── global_position: None     ← aguardando primeiro callback PX4
│   └── home_position: None       ← aguardando primeiro callback PX4
│
├── DroneObstacles
│   └── todos os flags: False
│
├── Timers ativos (mas com guards — só publicam quando PX4 está em Offboard)
│   ├── offboard_control_mode_timer @ 50 Hz
│   ├── trajectory_setpoint_timer @ 50 Hz
│   └── drone_state_timer @ 2 Hz
│
└── ActionServer aguardando goals da FSM
```

O log final que confirma o fim da inicialização é:
```
[INFO] [drone_node]: ================ DRONE NODE PRONTO ================
```

---

## 5. Fluxo de dados em operação normal

```
PX4 (firmware)
│
│ /fmu/out/vehicle_status_v1       → vehicle_status_callback()
│ /fmu/out/vehicle_local_position  → vehicle_local_position_callback()
│ /fmu/out/vehicle_global_position → vehicle_global_position_callback()
│ /fmu/out/vehicle_attitude        → vehicle_attitude_callback()
│ /fmu/out/home_position           → home_position_callback()
│ /fmu/out/vehicle_land_detected   → land_detected_callback()
│ /fmu/out/battery_status          → battery_status_callback()
│ /fmu/out/vehicle_command_ack     → command_ack_callback()
▼
DroneStatePX4 (container atualizado continuamente)
│
├── drone_state_timer (2 Hz)
│   ├── verifica_mudanca_de_estado_drone() → transições automáticas de estado
│   └── publish_drone_status() → DroneStateMSG → /drone_inspetor/interno/drone_node/drone_state
│
├── offboard_control_mode_timer (50 Hz)
│   └── OffboardControlMode → /fmu/in/offboard_control_mode
│
└── trajectory_setpoint_timer (50 Hz)
    └── TrajectorySetpoint → /fmu/in/trajectory_setpoint
                                                        ▲
FSM                                                     │
│                                                       │
│ DroneCommand Action → goal_callback()                 │
│                    → execute_drone_command_callback() │
│                        → arm() / takeoff() / goto()   │
│                          goto(use_focus=...) / land() │
│                          / rtl()                      │
│                          → define flags em DroneFSMContext ┘
│                          → verifica_mudanca_de_estado_drone() reage às flags
│
└── Feedback loop (0.1s): estado, distância, progresso → FSM

lidar_node / depth_node
├── ObstaclesMSG (lidar 360° + abaixo) → lidar_obstacles_callback()
│                                      → DroneObstacles.update_from_lidar()
└── ObstaclesMSG (depth frontal)       → depth_obstacles_callback()
                                       → DroneObstacles.update_from_depth()
   → flags agregadas (OR entre fontes) usadas em create_moving_trajectory_setpoint() para desviar
```

---

## 6. Shutdown

```python
def destroy_node(self):
    self._is_drone_node_shutting_down = True
    super().destroy_node()
```

A flag `_is_drone_node_shutting_down = True` é verificada no loop de feedback do ActionServer antes de publicar feedback. Isso evita erros de "contexto inválido" quando o nó é destruído enquanto uma action ainda está em execução.

O `main()` encapsula o `executor.spin()` em `try/except` genérico e chama `drone_node.destroy_node()` + `rclpy.try_shutdown()` no bloco `finally`, garantindo cleanup mesmo em caso de erro inesperado.

---

## 7. Lógica de recepção de comandos do Mission Node

O `mission_node` é o único cliente que envia comandos ao `drone_node`. A comunicação ocorre exclusivamente via **ROS2 Action** (`DroneCommand`), nunca por tópicos simples.

### 7.1 Ciclo de vida de um comando

Cada comando segue 4 etapas obrigatórias:

```
Mission Node                         drone_node
    │                                     │
    │─── send_goal(DroneCommand) ────────►│
    │                                     ├── goal_callback()
    │                                     │   ├── verifica_validade_do_comando()
    │                                     │   │   ├── modo OFFBOARD ativo?
    │                                     │   │   ├── posição local disponível?
    │                                     │   │   └── estado FSM permite este comando?
    │                                     │   └── ACCEPT ou REJECT
    │                                     │
    │                                     ├── execute_drone_command_callback()
    │                                     │   ├── _execute_command(request)
    │                                     │   │   └── chama arm()/takeoff()/goto()/land()/rtl()
    │                                     │   │       └── define pending_command no contexto
    │                                     │   │
    │◄──── feedback (a cada 0.1s) ───────│   ├── loop de feedback
    │      (state, distance, progress)    │   │   ├── verifica timeout
    │                                     │   │   ├── verifica cancelamento
    │                                     │   │   └── _is_command_complete()?
    │                                     │   │
    │◄──── result (success/fail) ────────│   └── goal_handle.succeed() / abort()
    │                                     │
```

**Validação (`verifica_validade_do_comando`):**

| Comando | Estados permitidos |
|---|---|
| `ARM` | `POUSADO_DESARMADO` |
| `TAKEOFF` | `POUSADO_ARMADO` |
| `GOTO` | `VOANDO_PRONTO` |
| `LAND` | `VOANDO_PRONTO` |
| `RTL` | `VOANDO_PRONTO` |

Se o drone não estiver em modo OFFBOARD ou a posição local for desconhecida, **qualquer** comando é rejeitado.

### 7.2 Fluxo detalhado por comando

#### ARM

1. `arm()` publica `VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM` com `param1=1.0`.
2. O PX4 arma os motores e publica `VehicleStatus` com `arming_state=ARMED`.
3. O callback `px4_vehicle_status_callback` atualiza `state_px4.is_armed = True`.
4. No próximo tick da FSM (2 Hz), `PousadoDesarmadoState.on_step()` detecta `is_armed=True` e transiciona para `POUSADO_ARMADO`.
5. `_is_command_complete("ARM")` retorna `True` quando `state == POUSADO_ARMADO`.

#### TAKEOFF

1. `takeoff(altitude)` calcula `target_local_position` como `[current_x, current_y, home_z - altitude]` (NED: Z negativo = para cima).
2. Define `pending_command = "TAKEOFF"`.
3. No próximo tick, `PousadoArmadoState.on_step()` detecta o pending command, ativa `on_trajectory=True` e transiciona para `VOANDO_DECOLANDO`.
4. O timer de trajetória (50 Hz) publica setpoints com `position=[x, y, target_z]` — o drone sobe verticalmente mantendo X/Y e yaw do home.
5. `VoandoDecolandoState.on_step()` compara `|current_alt - target_alt|` com `position_tolerance` (0.15m). Quando dentro da tolerância, transiciona para `VOANDO_PRONTO`.
6. `_is_command_complete("TAKEOFF")` retorna `True` quando `state == VOANDO_PRONTO`.

#### GOTO (sem foco)

Sequência de 3 fases na FSM:

```
VOANDO_PRONTO ──(pending_command="GOTO")──► VOANDO_GIRANDO_INICIO
                                                    │
                                          yaw alinhado + 3s estabilização
                                                    │
                                                    ▼
                                            VOANDO_A_CAMINHO
                                                    │
                                          distância ≤ 0.15m (tolerância)
                                                    │
                                                    ▼
                                            VOANDO_GIRANDO_FIM
                                                    │
                                    yaw final alinhado + 3s estabilização
                                    (ou sem yaw final → direto para PRONTO)
                                                    │
                                                    ▼
                                            VOANDO_PRONTO
```

**Detalhes de cada fase:**

1. **`goto()`** converte lat/lon/alt para coordenadas locais via `global_to_local_position()`, calcula `target_direction_yaw` (ângulo para o destino), empilha um `Waypoint` na `WaypointStack` e define `pending_command="GOTO"`.

2. **`VOANDO_PRONTO`** consome o pending command, ativa o waypoint da pilha via `apply_waypoint()`, liga `on_trajectory=True` e transiciona para `VOANDO_GIRANDO_INICIO`.

3. **`VOANDO_GIRANDO_INICIO`** — o timer de trajetória envia setpoints com `_calculate_incremental_yaw()` (passos de `yaw_step_deg=15°`). A FSM verifica `|yaw_diff| ≤ yaw_tolerance_deg` (2°). Quando alinhado, aguarda `yaw_stabilization_delay` (3s) antes de transicionar.

4. **`VOANDO_A_CAMINHO`** — o timer delega ao `TrajectoryProfile` que gera setpoints com perfil trapezoidal (aceleração → cruzeiro → frenagem). A FSM verifica distância 3D ao alvo. Quando `≤ position_tolerance`, marca o waypoint como concluído e transiciona para `VOANDO_GIRANDO_FIM`.

5. **`VOANDO_GIRANDO_FIM`** — gira para `target_final_yaw_deg` (se definido). Ao concluir, chama `reset_trajectory_vars()` e transiciona para `VOANDO_PRONTO`.

**Casos especiais com NaN:**
- `lat/lon=NaN, alt válido`: pula rotação inicial, apenas muda altitude.
- `lat/lon/alt=NaN, yaw válido`: vai direto para rotação final.
- Todos NaN: comando ignorado.

**Completude:** `_is_command_complete("GOTO")` exige 3 condições simultâneas: `state == VOANDO_PRONTO` + `waypoint_stack.is_empty` + `pending_command is None`.

#### GOTO (com foco, `use_focus=True`)

```
VOANDO_PRONTO ──► VOANDO_GIRANDO_COM_FOCO ──► VOANDO_A_CAMINHO_COM_FOCO ──► VOANDO_PRONTO
```

Diferenças em relação ao GOTO padrão:
- **Rotação inicial**: gira para apontar ao ponto de foco (não à direção de movimento).
- **Em trânsito**: o yaw é recalculado **a cada tick** (50 Hz) via `_calculate_focus_yaw()` para manter a câmera apontada ao foco enquanto o drone se desloca.
- **Sem rotação final**: ao chegar ao destino, aguarda estabilização de 3s e vai para `VOANDO_PRONTO`.
- O parâmetro `yaw` é ignorado; `lat/lon/alt` e `focus_lat/focus_lon` são obrigatórios.

#### LAND

1. `land()` publica `VehicleCommand.VEHICLE_CMD_NAV_LAND` — o PX4 muda temporariamente para modo `AUTO_LAND` e executa pouso autônomo.
2. Define `pending_command = "LAND"`.
3. `VoandoProntoState` consome o pending e transiciona para `POUSANDO`.
4. `PousandoState.on_step()` apenas monitora `is_landed` — o controle de descida é do PX4.
5. Quando `is_landed=True`, o PX4 desarma automaticamente (parâmetro `COM_DISARM_LAND`).
6. O desarmamento é capturado pela **verificação global de desarmamento** na FSM (`tick()`), que transiciona para `POUSADO_DESARMADO`.

#### RTL (Return to Launch)

Executa o retorno via Offboard (não delega ao PX4):

```
VOANDO_PRONTO ──► RETORNANDO_GIRANDO_INICIO ──► RETORNANDO_A_CAMINHO ──► RETORNANDO_GIRANDO_FIM ──► POUSANDO ──► POUSADO_DESARMADO
```

1. `rtl()` calcula destino como `[home_x, home_y, home_z - rtl_altitude]` (sobe até 30m relativo ao home), empilha waypoint e define `pending_command="RTL"`.
2. Gira para apontar ao home, voa até lá (com perfil trapezoidal), gira para `home_yaw`.
3. `RetornandoGirandoFimState` chama `land()` (pouso nativo PX4) e transiciona para `POUSANDO`.
4. Após pouso + desarme automático → `POUSADO_DESARMADO`.

#### STOP

Ação especial — não é um comando do Action Server, mas executado via `cancel_callback`:
1. `stop()` chama `reset_trajectory_vars()` (limpa pilha, targets, flags).
2. Armazena posição atual como `last_static_position` para hover estável.
3. Transiciona para `VOANDO_PRONTO`.

### 7.3 Feedback e conclusão

O loop de feedback em `execute_drone_command_callback` publica a cada **0.1s**:

| Campo | Origem |
|---|---|
| `current_state` | `drone_context.state` (IntEnum) |
| `state_name` | `drone_context.state.name` (string) |
| `distance_to_target` | Distância 3D euclidiana ao `target_local_position` |
| `progress_percent` | Para GOTO/RTL: `(1 - dist_atual/dist_inicial) × 100` |

**Condições de saída do loop:**
- **Sucesso**: `_is_command_complete()` retorna `True` → `goal_handle.succeed()`.
- **Timeout**: ARM=10s, TAKEOFF/LAND=60s, GOTO=120s, RTL=180s → `goal_handle.abort()`.
- **Cancelamento**: `goal_handle.is_cancel_requested` → `goal_handle.canceled()`.
- **Shutdown**: `_is_drone_node_shutting_down=True` → retorna Result vazio.

---

## 8. Sistema de desvio de obstáculos

### 8.1 Fontes de detecção

O drone recebe dados de obstáculos de **duas fontes independentes** via mensagem `ObstaclesMSG`:

| Fonte | Subscriber | Cobertura | Callback |
|---|---|---|---|
| `lidar_node` | `Topics.Interno.LIDAR_OBSTACLE_DETECTIONS` | 360° horizontal + 1D abaixo | `lidar_obstacles_callback()` → `obstacles.update_from_lidar()` |
| `depth_node` | `Topics.Interno.DEPTH_OBSTACLE_DETECTIONS` | Frontal (câmera profundidade) | `depth_obstacles_callback()` → `obstacles.update_from_depth()` |

Cada fonte mantém seu próprio buffer (`_ObstacleFlags`). As **propriedades públicas** de `DroneObstacles` fazem **OR** entre as fontes — basta **um** sensor detectar para o desvio ser acionado.

**Flags disponíveis:**

| Flag | Significado |
|---|---|
| `have_obstacles_1/2/3/5/8` | Obstáculos dentro de 1m, 2m, 3m, 5m, 8m |
| `have_obstacles_front/right/back/left_90` | Obstáculos por quadrante de 90° |
| `have_down_obstacles_1/0.5` | Obstáculos abaixo do drone |

### 8.2 Influência na velocidade (TrajectoryProfile)

Durante estados de deslocamento (`VOANDO_A_CAMINHO`, `VOANDO_A_CAMINHO_COM_FOCO`, `RETORNANDO_A_CAMINHO`), os obstáculos aplicam um **cap de velocidade** ao perfil trapezoidal via `get_velocity_cap_from_obstacles()`:

| Distância do obstáculo | Cap de velocidade |
|---|---|
| > 8m | `vc` (sem cap — velocidade de cruzeiro nominal) |
| ≤ 8m | 3.0 m/s |
| ≤ 5m | 2.0 m/s |
| ≤ 3m | 1.0 m/s |
| ≤ 2m | 0.5 m/s |
| ≤ 1m | 0.0 m/s (parada total) |

Quando `v_des > v_target_obs`, o `TrajectoryProfile` entra na fase `OBSTACLE_BRAKE` e desacelera com `-ao` (obstacle_deceleration, padrão 2.0 m/s²) até a velocidade ficar dentro do cap.

### 8.3 Sub-FSM de desvio — 5 fases

Quando o drone detecta obstáculo **no quadrante da direção de movimento** durante os estados `*_A_CAMINHO`, a FSM entra em uma **sub-FSM de desvio** com 5 fases. Existem **3 variantes** desta sub-FSM (uma por fluxo de voo):

| Fluxo normal | Sub-FSM de desvio (5 estados) |
|---|---|
| `VOANDO_A_CAMINHO` | `VOANDO_A_CAMINHO_OBSTACULO → _GIRANDO_INICIO → _DESVIANDO → _GIRANDO_FIM → _DESVIADO` |
| `VOANDO_A_CAMINHO_COM_FOCO` | `VOANDO_A_CAMINHO_COM_FOCO_OBSTACULO → ... → _DESVIADO` |
| `RETORNANDO_A_CAMINHO` | `RETORNANDO_A_CAMINHO_OBSTACULO → ... → _DESVIADO` |

Todas as variantes herdam das 5 classes base em `_obstaculo_base.py`:

```
Estado *_A_CAMINHO
  │
  │  obstáculo detectado em has_obstacle_in_sector(direction_yaw)
  ▼
Fase 1: *_OBSTACULO (BaseObstaculoState)
  │  • Salva destino original (save_original_target) — apenas na primeira vez
  │  • Registra ponto de parada (register_obstacle_stop_point)
  │  • Calcula coordenada de desvio via adjust_trajectory_XY():
  │    - Obstáculo à frente + livre à direita → desvia 1m à direita
  │    - Obstáculo à frente + livre à esquerda → desvia 1m à esquerda
  │    - Sem saída lateral (3 quadrantes bloqueados) → sobe 2m (NED: z -= 2.0)
  │  • Valida anti-loop (is_detour_loop)
  │  • Atualiza target_local_position para o desvio
  ▼
Fase 2: *_OBSTACULO_GIRANDO_INICIO (BaseObstaculoGirandoInicioState)
  │  • Gira para apontar na direção do desvio
  │  • Aguarda estabilização (3s)
  ▼
Fase 3: *_OBSTACULO_DESVIANDO (BaseObstaculoDesviandoState)
  │  • Voa até a coordenada de desvio
  │  • Se NOVO obstáculo detectado durante o desvio → volta à Fase 1 (recursão)
  │  • Quando chega ao desvio (≤ 0.15m) → avança
  ▼
Fase 4: *_OBSTACULO_GIRANDO_FIM (BaseObstaculoGirandoFimState)
  │  • Calcula yaw para apontar ao destino ORIGINAL (saved_target_local_position)
  │  • Gira e aguarda estabilização (3s)
  ▼
Fase 5: *_OBSTACULO_DESVIADO (BaseObstaculoDesviadoState)
  │  • Verifica se o caminho para o destino original está livre
  │  • Se LIVRE:
  │    - Restaura destino original (restore_original_target)
  │    - Limpa pilhas e snapshot (clear_obstacle_avoidance_state)
  │    - Retorna ao estado normal (*_A_CAMINHO)
  │  • Se OBSTÁCULO PERSISTE: volta à Fase 1 (recalcula)
  ▼
Estado *_A_CAMINHO (retomado com destino original)
```

**Recursividade:** durante a Fase 3 (`*_DESVIANDO`), se um novo obstáculo for detectado, a sub-FSM **reentra na Fase 1**. O destino original NÃO é sobrescrito (o `save_original_target()` só grava na primeira vez).

### 8.4 Proteção anti-loop

O sistema mantém **duas pilhas** no `DroneFSMContext`:

1. **`obstacle_stop_points`**: coordenadas `(x, y, z, yaw)` onde o drone parou por obstáculo.
2. **`detour_calculated_points`**: coordenadas `(x, y, z)` calculadas como desvio.

Antes de aceitar um desvio candidato, `is_detour_loop()` verifica se ele está a menos de `obstacle_loop_threshold` (1.0m) de **qualquer** ponto em ambas as pilhas. Se sim, o desvio é **abortado** e o drone vai para `VOANDO_PRONTO` via `reset_trajectory_vars()`.

Todas as pilhas são limpas em: `reset_trajectory_vars()` (STOP, perda de offboard, emergência), `clear_obstacle_avoidance_state()` (desvio concluído com sucesso).

---

## 9. Verificações globais pré-ciclo da FSM

A cada tick da FSM (2 Hz), o método `DroneFSM.tick()` executa **3 verificações globais** em ordem de prioridade **antes** de processar o estado atual. Se qualquer verificação dispara uma transição, o `on_step()` do estado atual **não é executado** naquele ciclo.

### Verificação 1: Perda de modo Offboard (prioridade máxima)

```
SE nav_state ≠ OFFBOARD
   E nav_state ≠ AUTO_LAND
   E estado_atual ≠ OFFBOARD_DESATIVADO
   E estado_atual ≠ POUSANDO
ENTÃO:
   reset_trajectory_vars()
   transiciona → OFFBOARD_DESATIVADO
```

**Razão:** se o piloto mudou o modo via rádio ou o PX4 caiu para failsafe, o drone_node não deve tentar publicar setpoints. Os estados `POUSANDO` e `AUTO_LAND` são isentos porque o pouso nativo do PX4 muda temporariamente o `nav_state`.

### Verificação 2: Emergência (bateria crítica)

```
SE estado_atual ≠ OFFBOARD_DESATIVADO
   E emergency_active = False
   E bateria < 10%
ENTÃO:
   SE drone pousado e desarmado → apenas loga
   SE drone pousado e armado → disarm()
   SE drone em voo → rtl_native() (delega ao autopilot PX4)
   
   emergency_active = True  (impede reenvio a cada tick)
   reset_trajectory_vars()
   transiciona → OFFBOARD_DESATIVADO
```

**Nota:** `rtl_native()` é diferente de `rtl()` — publica `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` e **delega todo o controle ao PX4**, sem usar Offboard.

### Verificação 3: Desarmamento inesperado

```
SE nenhuma verificação anterior disparou transição
   E is_armed = False
   E estado_atual ≠ POUSADO_DESARMADO
   E estado_atual ≠ OFFBOARD_DESATIVADO
ENTÃO:
   reset_trajectory_vars()
   transiciona → POUSADO_DESARMADO
```

**Razão:** se os motores foram desligados inesperadamente (ex: failsafe de hardware, perda de sinal), a FSM deve refletir essa realidade imediatamente.

### Diagrama de prioridade

```
tick() chamado (2 Hz)
  │
  ├── [1] Offboard perdido? ─── SIM ──► OFFBOARD_DESATIVADO (pula on_step)
  │                              NÃO
  │                               │
  ├── [2] Emergência?      ─── SIM ──► OFFBOARD_DESATIVADO (pula on_step)
  │                              NÃO
  │                               │
  ├── [3] Desarmado?       ─── SIM ──► POUSADO_DESARMADO (pula on_step)
  │                              NÃO
  │                               │
  └── Nenhuma verificação disparou ──► executa super().tick() → on_step() do estado atual
```

---

## 10. Perfil trapezoidal de velocidade

O `TrajectoryProfile` gera setpoints de posição, velocidade e aceleração para estados de deslocamento linear. Encapsula uma FSM interna de 5 fases:

```
  IDLE ──start_segment()──► ACEL ──v ≥ vc──► CRUISE ──s_remaining ≤ s_brake──► BRAKE ──v ≈ 0──► DONE
                              │                                                   ▲
                              └── s_remaining ≤ s_brake (perfil triangular) ──────┘
                              
  Em qualquer fase ativa:  v_des > v_target_obs ──► OBSTACLE_BRAKE (desacelera com -ao)
```

**Parâmetros (configuráveis em `param_ros.yaml`):**

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `cruise_velocity` (vc) | 3.0 m/s | Velocidade máxima de cruzeiro |
| `travel_acceleration` (ad) | 1.0 m/s² | Aceleração e desaceleração nominal |
| `obstacle_deceleration` (ao) | 2.0 m/s² | Desaceleração por obstáculo (mais agressiva) |
| `arrival_position_tol` | 0.2 m | Tolerância de chegada |
| `arrival_velocity_tol` | 0.1 m/s | Velocidade para considerar parado |

**Integração com o timer de trajetória (50 Hz):**

1. `_advance_profile()` detecta novo segmento comparando `profile.target` com `target_local_position`.
2. Se novo segmento, chama `profile.start_segment()` com velocidade inicial projetada na nova direção (emenda suave).
3. Consulta `obstacles.get_velocity_cap_from_obstacles()` para obter `v_target_obs`.
4. Chama `profile.tick(dt=0.02, current_pos, v_target_obs)` que retorna `(pos, vel, acc)`.
5. O setpoint é publicado com position + velocity (feedforward) + acceleration (feedforward) para o PX4.

**Modo Offboard do PX4:** o `OffboardControlMode` é publicado com `position=True, velocity=True, acceleration=True`. O PX4 usa posição como referência primária e velocity/acceleration como feedforward, tornando o perfil trapezoidal previsível para o controlador interno.
