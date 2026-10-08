# Reproduzir a validação Gazebo

Este roteiro serve para repetir ensaios automatizados e registrar evidências de
sensores, controle e missão. O script cria seus próprios processos Gazebo/PX4,
executa o cenário e os encerra. Para a montagem do workspace, siga o
[README](../README.md); para abrir e operar o projeto no dia a dia, use
[EXECUCAO.md](EXECUCAO.md).

O ambiente ensaiado em 22/09/2026 foi ROS 2 Jazzy, Gazebo 8.15/gz-sensors 8.2.2,
PX4 local com airframe 4030 e recursos em `/home/keyslav/.simulation-gazebo`.
Os resultados e limitações históricos estão em
[VALIDACAO_GAZEBO.md](VALIDACAO_GAZEBO.md).

## Pré-requisitos

- Workspace compilado conforme o README, incluindo `px4_msgs`,
  `drone_inspetor_msgs` e `drone_inspetor`, com interfaces v2 compatíveis e
  Ruckig 0.19.4 no mesmo Python usado no build.
- PX4 SITL já compilado em `build/px4_sitl_default`, incluindo `etc/`,
  `rootfs/parameters.bson` e, para a correção abaixo, `compile_commands.json`,
  bibliotecas, auxiliares e ferramentas de compilação (`ninja`, `ar`, compilador).
- Recursos locais `models/x500_uerj/model.sdf`, `plugins/`, `server.config` e
  `worlds/Plataforma_UERJ.sdf`, conforme [INIT_SIMULACAO.md](../INIT_SIMULACAO.md).
- `gz`, `ros2`, `ros_gz_bridge`, Micro XRCE-DDS Agent e `nvidia-run` disponíveis.
  O script chama `nvidia-run gz sim` diretamente; o wrapper de GPU precisa existir
  no `PATH` mesmo quando a GUI não é usada.

Os comandos abaixo usam o venv preparado pelo README e o workspace normal.
`pymavlink` é uma dependência adicional para `--gcs-heartbeat`; `pyulog` é
necessário apenas para analisar ULogs com `tools/inspect_magnetic_log.py`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source .venv/bin/activate
source install/setup.bash
python -m pip install pymavlink
python -c 'import rclpy, ruckig, pymavlink; from drone_inspetor_msgs.action import DroneCommand'
```

A pasta `.drone-v2-validation` abaixo guarda saídas dos ensaios. Não é necessário
carregar o antigo overlay privado `install/` ou `python/` dessa pasta.

## 1. Preparar uma cópia corrigida do PX4

O firmware original apresentou erro de yaw de aproximadamente 46°. A correção
combina campo mundial ENU no plugin e campo corporal FLU→FRD no bridge. Os dois
arquivos gerados devem ser usados juntos. Não basta modificar o projeto ROS.

O comando abaixo exige o build SITL já existente e suas dependências de
compilação. Recompila um objeto e religa uma cópia do executável, sem escrever
no checkout/build original. Não é um procedimento de instalação limpa do PX4.

```bash
cd ~/ros2_ws
python src/drone_inspetor/tools/build_gazebo_magnetic_fix.py \
  --px4-root /home/keyslav/PX4-Autopilot \
  --server-config /home/keyslav/.simulation-gazebo/server.config \
  --output "$PWD/src/.drone-v2-validation/magnetic-local"
```

A pasta de saída deve ser nova. Ela conterá `px4`, auxiliares de inicialização,
`server.config`, comandos de compilação e hashes. O script rejeita um bridge
que não corresponda à transformação legada conhecida. Validado também com a
saída `magnetic-repro`; isso não aprova automaticamente outras versões Gazebo.

## 2. Executar um cenário

Com o ambiente dos pré-requisitos carregado, use os caminhos locais abaixo
(ajuste-os se o PX4, os recursos ou o Agent estiverem em outro diretório):

```bash
cd ~/ros2_ws
python src/drone_inspetor/tools/validate_gazebo.py \
  --px4-root /home/keyslav/PX4-Autopilot \
  --model-store /home/keyslav/.simulation-gazebo \
  --world-sdf /home/keyslav/.simulation-gazebo/worlds/Plataforma_UERJ.sdf \
  --px4-binary "$PWD/src/.drone-v2-validation/magnetic-local/px4" \
  --server-config "$PWD/src/.drone-v2-validation/magnetic-local/server.config" \
  --agent /usr/local/bin/MicroXRCEAgent --gcs-heartbeat \
  --scenario land --output "$PWD/src/.drone-v2-validation/meu-teste-01"
```

A saída deve ser um diretório novo em cada rodada. `--preflight-only` encerra
depois de conferir telemetria, HOME, LiDARs e yaw, antes de OFFBOARD/ARM. Sem essa
opção, o comando acima arma e executa voo na simulação. Para diagnosticar outra
orientação, acrescente `--preflight-only --yaw-enu 1.5707963267948966` (90° ENU);
rotação diferente de zero é aceita somente com `--preflight-only`.

Escolhas de `--scenario` (padrão: `land`):

| Cenário | Sequência |
| --- | --- |
| `land` | ARM, subida de 3 m, pouso |
| `cruise` | Subida, ida/volta de 40 m Norte, pouso |
| `obstacle` | Cilindro físico temporário, GOTO de 12 m com desvio, cancelamento, retorno ao HOME, pouso |
| `rtl` | Subida, afastamento de 12 m, RTL nativo até pouso e desarme |
| `mission` | Missão por MissionNode/CVNode reais; exige `--mission-image`, detecta flare em imagem controlada, grava, retorna e desarma |

O cenário `mission` precisa de uma imagem existente e decodificável em
`--mission-image /caminho/imagem.png`, além dos pesos
`flare_yolov8n_detection_300ep.pt` e `corrosion_yolo8n_detection.pt` no diretório
de modelos usado pelo CVNode. Consulte [MODELOS_CV.md](MODELOS_CV.md). A imagem é
republicada pelo ensaio; esse cenário não verifica a câmera ao vivo do Gazebo.

`--px4-binary` e `--server-config` são opcionais no script: sem eles, são usados
o executável do build original e `server.config` do `--model-store`. Isso não
aplica a correção magnética. `--agent` pode ser omitido se `MicroXRCEAgent`
estiver no `PATH`. `--gcs-heartbeat` inicia uma GCS mínima que apenas envia
heartbeat para esta instância; ela usa o mesmo Python do ensaio.

O ensaio utiliza instância 27, DDS 173, Agent UDP 18889 e GCS localhost 18597.
Execute uma rodada por vez. O wrapper encerra seus processos; não abre GUI ou
QGroundControl. `flight.json` registra comandos, medições e sucesso/falha;
`manifest.json` identifica entradas e processos. Veja também os logs individuais
quando uma falha ocorrer antes da criação de `flight.json`.

O wrapper copia `parameters.bson` para a pasta do ensaio e registra hashes das
entradas. Ele não carrega o dashboard nem usa o iniciador gráfico da execução
cotidiana. A comparação de yaw com o Gazebo fica em `frame-check.json`; o limite
de erro antes de armar é 5°.

Os cenários aprovados não certificam toda a plataforma ou qualquer obstáculo.
O mapa LiDAR é local e horizontal; não prova espaço livre acima. LAND termina
pousado, podendo ainda estar armado; o cenário RTL verifica desarme.
