# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Visão geral

Pacote ROS2 (ament_python) para controle e monitoramento autônomo de um drone de inspeção industrial. Integra PX4 (via `px4_msgs`), simulador Gazebo (via `ros_gz_bridge`), visão computacional YOLO/Ultralytics, sensores (LiDAR 2D + 1D, depth camera) e GUI PyQt6.

Código, nomes de estados e comentários estão em **português (pt-BR)** — preserve esse idioma ao escrever novo código no pacote.

## Build e execução

```bash
cd ~/ros2_ws
colcon build --packages-select drone_inspetor
source install/setup.bash

# Launch principal (Gazebo bridges + drone_node ativo; demais nós comentados)
ros2 launch drone_inspetor dashboard_launch.py

# Launch alternativo que sobe todos os 7 nós sem bridges Gazebo
ros2 launch drone_inspetor drone_inspetor_launch.py
```

Em `dashboard_launch.py` a maioria dos nós está **comentada** no `LaunchDescription` — descomente conforme o subsistema em teste. Para depuração de um nó, descomente o `arguments=["--ros-args", "--log-level", "DEBUG"]` já preparado em cada `Node(...)`.

### Scripts de teste e simulação

```bash
# Teste do drone_node sozinho (envia ARM → TAKEOFF → GOTO → GOTO → LAND via Action)
ros2 run drone_inspetor teste_drone_node

# Ciclo end-to-end de comandos do drone (bash, requer drone_node rodando)
./drone_inspetor/scripts/teste_critico.sh

# Disparar missão pelo mission_node (precisa do mission_node ativo + missão definida em missions.json)
./drone_inspetor/scripts/teste_iniciar_missao.sh <nome_da_missao>   # default: flare

# Lançar Gazebo + modelos PX4 (helper)
python3 drone_inspetor/scripts/simulation-gazebo.py --world <world>
```

Não há `pytest`/`colcon test` configurado. Os "testes" são scripts manuais que exercitam Actions/tópicos contra um drone (real ou simulado) já em execução.

## Arquitetura

```
Gazebo  ──ros_gz_bridge──┐                                ┌──► Dashboard (PyQt6)
                         ▼                                │
PX4 ◄──px4_msgs──► drone_node ◄──Action DroneCommand──► mission_node ◄──Service──► cv_node
                         │                                │
                         └──► camera_node / depth_node / lidar_node ─► Dashboard
```

- **drone_node** ("músculos"): única interface com PX4. Recebe comandos de alto nível via Action `DroneCommand`, traduz para `OffboardControlMode`/`VehicleCommand`, mantém FSM interna de estados de voo.
- **mission_node** ("cérebro"): FSM hierárquica de missão. Lê `missions.json`, orquestra `drone_node` via Action, controla pipeline de `cv_node` via Service.
- **dashboard_node**: ponte GUI↔ROS2. Roda o executor `rclpy` em thread separada; o loop PyQt fica no main thread. Sinais PyQt propagam dados dos subscribers para os widgets.
- **camera/cv/depth/lidar_node**: cada um normaliza dados crus do sensor e publica em tópicos internos.

### Layout do pacote (`drone_inspetor/drone_inspetor/`)

```
common/           # Cross-cutting: enums, constantes, base StateMachine, log helpers, math, param_utils
ros_interfaces/   # Fonte única de verdade dos canais (Topics, Services, Actions, QoSProfiles + helpers)
nodes/            # Cada nó é um SUBPACOTE, não um arquivo:
  drone_node/       fsm/{machine,context,states/...}, action_server.py, px4_commands.py,
                    trajectory.py, trajectory_profile.py, obstacles.py, waypoint_stack.py
  mission_node/     fsm/{machine,context,states/...}, drone_state_data.py
  dashboard_node/   dashboard_node.py
  camera_node/, cv_node/, depth_node/, lidar_node/
gui/              # PyQt6: dashboard_gui, controles, mission, mapa (Leaflet), *_screen.py, log_analise
publishers/ subscribers/ signals/   # GUI ↔ ROS bridges, um arquivo por sensor
config/           # param_ros.yaml (parâmetros ROS2 por nó), ros_gz_bridges.yaml
missions/         # missions.json (waypoints, alvos, anomalias)
redes_treinadas/  # *.pt YOLO + models.json (registro de modelos)
launch/           # dashboard_launch.py (produção), drone_inspetor_launch.py (todos os nós), simulation_launch.py
scripts/          # teste_drone_node.py (entry point), teste_*.sh, simulation-gazebo.py, legados/
```

### Convenções importantes (não óbvias)

**1. `ros_interfaces/` é fonte única de verdade.** Nunca declare nome de tópico/serviço/action como string solta em um nó. Em vez disso:

```python
from drone_inspetor.ros_interfaces import Topics, create_publisher_from, create_subscription_from

create_subscription_from(self, Topics.Interno.DRONE_STATE, self.callback)
create_publisher_from(self, Topics.PX4.VEHICLE_COMMAND)
```

Cada spec amarra `nome + tipo + QoS`, então pub/sub nunca divergem em runtime. Antes de adicionar um tópico novo, adicione a spec em `ros_interfaces/{internal,external,px4,dashboard,services,actions}.py`.

**2. Namespaces de tópicos.**
- `/drone_inspetor/externo/*` — vindos do simulador/drone (sensores brutos, telemetria PX4)
- `/drone_inspetor/interno/<nome_do_no>/*` — produzidos pelos nós internos
- `/drone_inspetor/action/*` — Actions (ex.: `drone_command`)
- `/fmu/{in,out}/*` — comunicação direta com PX4 (definido em `ros_interfaces/px4.py`)

**3. FSMs.** `common/state.py` define a `StateMachine` base. Cada nó com FSM (`drone_node`, `mission_node`) tem `fsm/machine.py` (subclasse), `fsm/context.py` (estado compartilhado) e `fsm/states/<nome_do_estado>.py` (um arquivo por estado). Os enums em `common/enums.py` (`DroneStateDescription`, `MissionStateDescription`) são a lista canônica de estados — sempre adicione lá primeiro, depois crie o arquivo correspondente em `fsm/states/`.

`DroneFSM` executa **verificações globais por ciclo** (perda de offboard, emergência, target inalcançável, desarmamento) **antes** de delegar ao estado atual — isso vive em `drone_node/fsm/machine.py`, não nos estados.

**4. `drone_node` usa mixins.** A classe `DroneNode` compõe `DroneTrajectoryMixin`, `DroneActionServerMixin`, `DronePX4CommandsMixin` (definidos em `nodes/drone_node/*.py`). Adicione métodos relacionados a trajetória/PX4/action no mixin correspondente, não no `drone_node.py` principal.

**5. Padrão GUI ↔ ROS2 (assíncrono).**
- `subscribers/dashboard_<sensor>_subscriber.py` recebe tópico ROS e **emite sinal PyQt** (`signals/dashboard_signals.py`).
- `publishers/dashboard_<sensor>_publisher.py` publica comandos do dashboard para os nós.
- O `dashboard_node` registra ambos e roda `rclpy.spin` em uma thread; o PyQt event loop fica no thread principal.
- Nunca chame `rclpy` direto de um slot Qt — vá pelos publishers do diretório `publishers/`.

**6. Comandos do drone vão via Action, não tópico.** Toda interação `mission_node → drone_node` usa a Action `drone_inspetor_msgs/action/DroneCommand` (campos `command`, `lat/lon/alt`, `yaw`, `use_focus`, `focus_lat/lon`). Comandos válidos: `ARM`, `TAKEOFF`, `GOTO`, `RTL`, `LAND`. Ver `scripts/teste_critico.sh` para uma sequência completa de payloads.

**7. Parâmetros.** Carregue parâmetros ROS2 via `drone_inspetor.common.param_utils.load_param(self, name, default)` — esse helper já lida com declaração + get. Valores default ficam em `config/param_ros.yaml`.

## Dependências externas

- `drone_inspetor_msgs` (pacote separado, mensagens/services/actions customizados)
- `px4_msgs` (interface PX4)
- `ros_gz_bridge`, `ros_gz_image` (bridges Gazebo)
- PyQt6 + PyQt6-WebEngine (mapa Leaflet via WebEngine)
- Ultralytics/YOLO (`*.pt` em `redes_treinadas/`)
