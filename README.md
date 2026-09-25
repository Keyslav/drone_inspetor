# Drone Inspetor — v2.0

Ambiente local de simulação: [referência inicial do X500 UERJ](INIT_SIMULACAO.md)
(caminhos, comandos, massas, motores e inconsistências conhecidas).

Aplicação ROS 2 para inspeção com PX4, processamento de sensores e dashboard PyQt6.
Os pacotes `drone_inspetor` e `drone_inspetor_msgs` devem usar **a mesma linha v2.0**.
A v1 possui nomes e campos incompatíveis; mudar apenas o checkout não recompila as interfaces.

## Ambiente e instalação

A base de desenvolvimento é Ubuntu 24.04, ROS 2 Jazzy e Python 3.12. O pacote usa
**ament_python**; `setup.py` é a única fonte de instalação. Bibliotecas ROS/Qt/OpenCV
de sistema são resolvidas por `package.xml`; `requirements-runtime.txt` fixa o gerador
Ruckig e a combinação de inferência Python. O `px4_msgs` usado no desenvolvimento é
`57e96edf0772fb548cbc352b332e67e76fc64756` (manifesto 2.0.1). O firmware PX4 precisa
usar mensagens compatíveis com esse checkout; isso não declara compatibilidade de
voo com qualquer release PX4.

Com ROS Jazzy instalado e os três repositórios em `~/ros2_ws/src`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
# Na primeira instalação do rosdep: sudo rosdep init
rosdep update
rosdep install --from-paths src/drone_inspetor src/drone_inspetor_msgs src/px4_msgs \
  --ignore-src --rosdistro jazzy -r -y

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
# CPU; para CUDA selecione previamente a distribuição Torch compatível com a GPU.
python -m pip install torch==2.8.0 torchvision==0.23.0 \
  --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r src/drone_inspetor/requirements-runtime.txt
python -m colcon build --symlink-install \
  --packages-select px4_msgs drone_inspetor_msgs drone_inspetor
source install/setup.bash
```

Use o mesmo Python no build e na execução. A restrição `numpy<2` preserva a ABI do
`cv_bridge` instalado pelo ROS Jazzy; não atualize NumPy/OpenCV isoladamente nesse
ambiente. O Ruckig 0.19.4 é instalado antes do voo e calcula localmente os perfis
ponto a ponto. Não são utilizados waypoints intermediários nem a API cloud do Ruckig.

Para comparar branches sem aproveitar interfaces v1 já compiladas, use um build novo:

```bash
cd ~/ros2_ws
python -m colcon --log-base /tmp/drone-v2/log build \
  --build-base /tmp/drone-v2/build --install-base /tmp/drone-v2/install \
  --packages-select px4_msgs drone_inspetor_msgs drone_inspetor
source /tmp/drone-v2/install/setup.bash
python -c 'from drone_inspetor_msgs.msg import DashboardMissionCommandMSG'
```

Os pesos YOLO (`drone_inspetor/redes_treinadas/*.pt`) e modelos Gazebo locais são
artefatos externos, ignorados pelo Git. O catálogo `models.json` descreve os arquivos
esperados; instalar o pacote não baixa pesos automaticamente. Disponibilize os pesos
antes de habilitar CV. Gazebo, PX4 SITL e Micro XRCE-DDS Agent são processos externos.

Use **Redes CV** para escolher equipamentos/anomalias em um popup independente.
A imagem ampliada fica dedicada ao vídeo. O parâmetro `models_directory` permite
armazenar pesos fora do build; veja [seleção e armazenamento de redes](docs/MODELOS_CV.md).

### Atualização de um workspace existente para v2

Trocar a branch não recompila as mensagens ROS. Atualize **os dois pacotes** na
instalação que será usada para executar o dashboard, mesmo usando `--symlink-install`:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
# Ative o mesmo ambiente Python preparado acima, se estiver usando venv.
python3 -c 'import ruckig'  # deve estar instalado nesse Python
colcon build --symlink-install --packages-select drone_inspetor_msgs drone_inspetor
source install/setup.bash
python3 -c 'from drone_inspetor_msgs.msg import DashboardMissionCommandMSG'
ros2 launch drone_inspetor dashboard_launch.py
```

Se as bridges já estiverem rodando, acrescente `bridges:=false` ao launch para
não duplicá-las. O launch verifica as interfaces e o Ruckig antes de iniciar os
nós; um build isolado em outro diretório não atualiza `~/ros2_ws/install`.

## Execução por contexto

O dashboard possui resumo de telemetria, painéis adaptáveis e radar Qt nativo.
Veja [layout, radar e responsividade](docs/DASHBOARD_V2.md) e o
[guia de leitura do código](docs/GUIA_LEITURA_CODIGO.md).

Cada missão grava automaticamente `events.jsonl` em sua pasta de sessão
(por padrão, `~/Drone_Inspetor_Missoes/mission_.../`). O diário reúne estados,
telemetria resumida, comandos/resultados e logs de drone/mission/CV. Veja
[diagnóstico da Flare e formato do diário](docs/DIAGNOSTICO_MISSAO_FLARE.md).

Todos os launchers recebem o YAML de parâmetros e aceitam argumentos, sem precisar
comentar nós no código:

```bash
# Aplicação completa, relógio real, sensores ROS externos (sem bridges Gazebo)
ros2 launch drone_inspetor drone_inspetor_launch.py

# Aplicação completa + dashboard e bridges de uma simulação já iniciada
ros2 launch drone_inspetor dashboard_launch.py

# Somente bridges para uma simulação já iniciada
ros2 launch drone_inspetor simulation_launch.py

# Exemplo headless sem inferência
ros2 launch drone_inspetor dashboard_launch.py with_dashboard:=false with_cv:=false

# Catálogo customizado compartilhado pela missão e pela GUI
ros2 launch drone_inspetor drone_inspetor_launch.py \
  missions_file:=/caminho/missoes.json params_file:=/caminho/parametros.yaml
```

| Argumento | Comportamento |
|---|---|
| `use_sim_time` | `true` no dashboard/simulation; `false` no contexto real |
| `bridges` | Habilita `ros_gz_bridge` e `ros_gz_image` |
| `bridges_file` | YAML de configuração do `ros_gz_bridge` |
| `with_camera`, `with_cv`, `with_depth`, `with_lidar` | Seleção dos processadores de sensores |
| `with_drone`, `with_mission`, `with_dashboard` | Seleção de controle, missão e GUI |
| `params_file` | YAML aplicado a todos os nós |
| `missions_file` | Caminho absoluto, ou relativo a `share/drone_inspetor/missions` |
| `log_level` | Nível ROS, padrão `info` |

Use `bridges_file:=/caminho/bridges.yaml` para fornecer tópicos próprios do mundo e da instância; o padrão é `share/drone_inspetor/config/ros_gz_bridges.yaml`.

Fechar o dashboard encerra esse launch. Para manter controle/missão independentes da
janela, execute o launch com `with_dashboard:=false` e abra `dashboard_node` separadamente.
O launch não arma o drone nem inicia uma missão.

## Monitor de estados

Abra **Monitor do drone** no topo do dashboard, ou use **Ctrl+M**. A tela reúne
estados do drone/PX4/missão, bateria, movimento e disponibilidade de sensores.
A aba **Tópicos e campos** mostra valores, idade da última recepção e frequência,
com filtro por campo e identificação de dados desatualizados.

Para abrir só o monitor, sem os painéis de câmera:

```bash
ros2 run drone_inspetor monitor_node
```

É uma interface de leitura; não inicia controle, missão ou simulação. Veja
[fontes, unidades e remaps](docs/MONITOR_DRONE.md).

## Responsabilidades do código

O controle usa NED (Norte/Leste/Abaixo); tópicos PX4 mantêm esse frame no ROS 2.
O `DroneStateMSG` preserva posições locais legadas com Z para cima, mas derivadas
NED. Consulte o [contrato de coordenadas e altitudes](docs/COORDENADAS.md) antes
de integrar Gazebo, sensores ou novos consumidores.

| Área | Responsabilidade |
|---|---|
| `nodes/drone_node` | Action `DroneCommand`, telemetria PX4, FSMs de voo/deslocamento e setpoints |
| `navigation` | Cálculo puro do perfil de movimento e decisões de obstáculos |
| `missions` | Modelos tipados e repositório de catálogos, sem ROS/Qt e sem criar sessões |
| `nodes/mission_node` | Orquestração da missão, clientes de voo/CV e arquivos da sessão |
| `nodes/camera_node`, `nodes/cv_node`, `nodes/depth_node`, `nodes/lidar_node` | Adaptadores de sensores e processamento |
| `media` | Propriedade e sincronização de gravação de fotos/vídeos |
| `gui/presentation` | Snapshots imutáveis e formatação, sem ROS/Qt |
| `gui/widgets`, `gui/theme.py` | Widgets compartilhados, conversão de imagem e tema |
| `signals`, `publishers`, `subscribers` | Fronteira assíncrona GUI↔ROS; nomes preservados |
| `ros_interfaces` | Especificações de canais, tipos e QoS |
| `base_classes` | Contrato de ciclo de vida das máquinas de estados |

O caminho de dados de detecção é mensagem ROS → `DetectionFrame` imutável → sinal Qt
→ tela. JSON permanece somente onde faz parte do contrato ROS ou de arquivo. O catálogo
de missões é validado pela mesma implementação no dashboard e na missão. A GUI roda
na thread principal, e o executor ROS roda separadamente; snapshots não compartilham
listas mutáveis com callbacks.

A navegação produz referências de **posição, velocidade e aceleração**, com limites de
velocidade de cruzeiro, aceleração e jerk. Velocidade/aceleração são feedforward para
os controladores internos do PX4; não substituem a sintonia desses controladores.
O tratamento de obstáculos deve reduzir a referência de velocidade usando a distância
livre e a capacidade de frenagem, considerando validade/idade dos sensores. Limites e
referenciais ficam explícitos nos tipos do módulo `navigation` e no YAML. A confirmação
de chegada depende também da telemetria; terminar o perfil não prova chegada física.

Antes do destino, a referência reserva um trecho de aproximação lenta para acomodar
a resposta do controlador. `navigation.arrival_approach_velocity` define o limite
desse trecho (0,6 m/s) e `navigation.arrival_settling_time` sua reserva de tempo
(2 s). O perfil mantém o alvo original e termina com velocidade/aceleração zero;
um obstáculo pode exigir parada antes. Esses valores precisam ser validados na
dinâmica do modelo selecionado.

O desvio considera a passagem até o ponto lateral e a extensão observada da saída
em direção ao destino. A saída serve para escolher entre candidatos, sem pressupor
que a região oculta esteja livre. Após o giro, um novo segmento só começa quando
posição e velocidade estiverem nas tolerâncias de chegada; alinhar yaw não basta.
Quando há obstáculo observado, a próxima perna pode ser escolhida ainda em hover,
antes do giro, evitando alinhar primeiro para um rumo que será descartado. Sem
cobertura na direção desejada, o drone pode girar para observá-la antes de avançar.

## Contrato dos comandos

A preparação inicial de OFFBOARD acontece desarmado. Quando o veículo armado
sai de OFFBOARD, o nó interrompe seus setpoints para preservar a autoridade de
LAND/RTL e dos demais modos PX4. Reentrada em OFFBOARD durante o voo ainda exige
um protocolo explícito de transferência; não é preparada automaticamente.

A action admite uma operação de cada vez, reservada na aceitação do goal. `ARM`,
`DISARM`, `TAKEOFF`, `GOTO`, `STOP`, `LAND` e `RTL` são explícitos; comandos desconhecidos
são rejeitados. `DISARM` exige solo confirmado. `GOTO` usa altitude absoluta e converte
para NED incluindo o deslocamento local do HOME; NaN mantém individualmente cada eixo.

O cancelamento de movimento remove os destinos e espera confirmação de frenagem.
`STOP` também espera a velocidade medida e o perfil pararem. Um novo comando de voo
não preempta outro: o cliente deve cancelar/aguardar a operação anterior. `LAND`/`RTL`
transferem autoridade ao PX4 e rejeitam cancelamento pela action. O resultado depende
de pouso/desarme observado; timeout não retoma OFFBOARD automaticamente. Os prazos
operacionais usam relógio monotônico, mesmo com `/clock` simulado pausado.

## Verificação

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/.venv/bin/activate
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/drone_inspetor
QT_QPA_PLATFORM=offscreen python -m pytest test -m 'not linter' -q
python -m flake8 drone_inspetor test setup.py --select E9,F63,F7,F82 --show-source
```

Há testes de contratos ROS, missão, navegação, percepção, mídia e GUI. Os testes de
GUI usam `offscreen`, verificam snapshots, conversão/cópia de pixels, seleção de
modelos e abertura/fechamento de janelas CV, sem tela física. Os testes de launch
avaliam argumentos e condições sem iniciar nós. Esses testes **não certificam voo**;
SITL deve verificar trajetórias, obstáculos, sensores indisponíveis, cancelamento e
transferência para LAND/RTL antes de operação real.

O [guia dos ensaios SIH](docs/VALIDACAO_SIH.md) registra o cenário, comandos,
métricas e limitações da validação com PX4. A política de retentativa do retorno
da missão está em [Retorno da missão](docs/MISSAO.md).

A CI em `.github/workflows/ci.yml` compila as interfaces, executa a suíte funcional e
verifica erros estáticos de Python. O legado ainda possui dívida de formatação e
docstrings; os testes originais `test_flake8.py`/`test_pep257.py` continuam disponíveis
com `pytest test -m linter`, e seus relatórios são publicados pela CI. Eles não são
silenciosamente removidos nem equivalem aos gates funcionais.
