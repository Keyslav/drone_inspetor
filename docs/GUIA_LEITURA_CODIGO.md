# Guia de leitura do código v2

Este mapa indica onde investigar cada comportamento. Comentários no código
explicam decisões e contratos; parâmetros e regras executáveis continuam sendo
a fonte de verdade. Os caminhos abaixo são relativos à raiz de `drone_inspetor`.

## Comece pelo fluxo que deseja entender

| Pergunta | Ponto de entrada e sequência |
| --- | --- |
| Como os nós são iniciados? | `drone_inspetor/launch/dashboard_launch.py` → arquivos de configuração carregados pelo launch → `setup.py` para executáveis e recursos instalados. |
| Onde estão nomes de tópicos e QoS? | `drone_inspetor/ros_interfaces/`: `external.py` para sensores, `px4.py` para autopiloto, `internal.py` para dados da aplicação; `specs.py` e `helpers.py` constroem interfaces. |
| O que acontece ao iniciar uma missão? | `nodes/mission_node/mission_node.py` valida o comando, resolve a definição pelo repositório, cria a pasta da sessão e inicia o diário. `fsm/mission/machine.py` é a autoridade do estado. |
| Quem envia e confirma cada manobra? | `nodes/mission_node/action_client.py` cria uma operação identificada. `nodes/drone_node/action_server.py` reserva o comando, despacha e confirma o resultado por telemetria. |
| Como o drone se move suavemente? | `nodes/drone_node/trajectory.py` decide a referência e limites; `navigation/motion.py` gera posição, velocidade e aceleração coerentes com Ruckig; o PX4 fecha as malhas de controle. |
| Como os obstáculos limitam o movimento? | `nodes/drone_node/navigation_sensors.py` valida scans/pose/idade; `navigation/obstacles.py` calcula corredor observado e desvios; `trajectory.py` reduz velocidade e freia. |
| Como são processadas imagens? | `nodes/cv_node/cv_node.py` adapta ROS; `inference.py` executa a análise; `model_registry.py` gerencia pesos; `detection_buffer.py` controla validade das consultas; `media/` grava artefatos. |
| Como os dados chegam à tela? | `nodes/dashboard_node/dashboard_node.py` e `subscribers/` adaptam ROS para os sinais em `signals/`; `gui/` apresenta os dados. A GUI não calcula setpoints. |

Os caminhos da tabela iniciados por `nodes/`, `navigation/`, `media/`, `subscribers/`,
`signals/` e `gui/` ficam dentro de `drone_inspetor/`.

## Missão: acompanhe o resultado, não apenas o estado visível

O caminho normal passa por preparar a sessão, armar, decolar, navegar para um
waypoint, detectar o equipamento e escanear. Waypoints sem inspeção avançam após
a chegada. Os arquivos de cada etapa estão em
`nodes/mission_node/fsm/mission/states/`.

Aceitar uma Action significa apenas que o servidor reservou o comando. A missão
só avança após um resultado de sucesso. Por exemplo, `executando_decolando.py`
aguarda o resultado do TAKEOFF; uma falha registra o motivo e encaminha para
RETORNANDO, sem enviar o primeiro GOTO. `retornando.py` espera a operação anterior
liberar sua reserva e solicita RTL; falhas de aceitação incerta não são reenviadas
indefinidamente.

Há três máquinas distintas: a da missão decide a sequência; DroneFSM acompanha
o ciclo do veículo; DeslocamentoFSM organiza fases das manobras em voo. A subida
do TAKEOFF é gerada diretamente por `Trajectory.compute_vertical_takeoff`, não
pela fase DESLOCANDO. LAND e RTL entregam o movimento ao modo nativo do PX4.

## Convenções que não devem ser misturadas

- Navegação interna e `TrajectorySetpoint`: NED, em metros; Z cresce para baixo.
  Velocidade está em m/s e aceleração em m/s². Yaw é rumo NED em radianos no PX4.
- Scans ROS: ângulos relativos ao sensor, zero à frente e positivo à esquerda
  (FLU). A pose na aquisição e o deslocamento de montagem transformam o scan
  para o mapa NED; um ângulo do radar não é um rumo geográfico.
- `DroneStateMSG` preserva posições locais NEU por compatibilidade, mas mantém
  velocidade/aceleração NED. A conversão está em `nodes/drone_node/telemetry.py`.
- `DroneCommand.GOTO.alt`: altitude AMSL; `TAKEOFF.altitude`: altura acima do HOME.
  Campos opcionais usam NaN explicitamente; o zero gerado pela interface é real.
- `ObstaclesMSG` é um resumo de apresentação. False também pode significar dado
  desconhecido; a navegação usa scans e não as flags booleanas de proximidade.

Consulte [COORDENADAS.md](COORDENADAS.md) para a convenção completa, incluindo
Gazebo, origem local do estimador e HOME.

## Relógios, concorrência e diagnóstico

O relógio ROS acompanha `/clock`: timestamps e permanência de inspeção pausam
com a simulação. Prazos de operações, validade de telemetria e espera de serviços
usam tempo monotônico, que continua avançando. Não compare diretamente valores
dos dois relógios. Os adaptadores de sensores descontam a idade ROS antes de
registrar a recepção monotônica.

No DroneNode, um lock protege despacho de comandos, snapshots e transições;
esperas do servidor de ações ficam fora dele para permitir novos setpoints. No
CVNode, imagem e controles compartilham um grupo exclusivo; a consulta de detecção
espera em outro grupo e recebe snapshots por `DetectionBuffer`. Trocar missão ou
modelos invalida os resultados pendentes.

Cada missão cria `events.jsonl` na pasta de sessão configurada. Comece procurando
`state_change`, `failure_reason` e eventos `rosout` com comandos/resultados. Os
snapshots estáveis são limitados a 1 Hz; eventos de transição não aguardam esse
intervalo. `elapsed_s` é tempo real monotônico, `ros_time_s` é tempo da simulação
e `utc` permite cruzar logs. Esse diário ajuda a explicar o fluxo, mas não contém
todas as amostras como um rosbag ou o ULog do PX4.

Para diagnóstico de decolagem seguida de retorno, veja também
[DIAGNOSTICO_MISSAO_FLARE.md](DIAGNOSTICO_MISSAO_FLARE.md). Testes por componente
estão em `test/`; cenários de voo isolado ficam em `tools/validate_gazebo.py` e
[REPRODUZIR_TESTES_GAZEBO.md](REPRODUZIR_TESTES_GAZEBO.md).
