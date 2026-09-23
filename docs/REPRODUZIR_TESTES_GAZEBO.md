# Reproduzir a validação Gazebo

Ambiente validado: ROS 2 Jazzy, Gazebo 8.15/gz-sensors 8.2.2, PX4 local com
airframe 4030 e recursos em `/home/keyslav/.simulation-gazebo`. Os resultados
estão em [VALIDACAO_GAZEBO.md](VALIDACAO_GAZEBO.md).

## 1. Preparar uma cópia corrigida do PX4

O firmware original apresentou erro de yaw de aproximadamente 46°. A correção
combina campo mundial ENU no plugin e campo corporal FLU→FRD no bridge. Os dois
arquivos gerados devem ser usados juntos. Não basta modificar o projeto ROS.

O comando abaixo exige o build SITL já existente e suas dependências de
compilação. Recompila um objeto e religa uma cópia do executável, sem escrever
no checkout/build original. Não é um procedimento de instalação limpa do PX4.

```bash
cd /home/keyslav/ros2_ws/src
python3 drone_inspetor/tools/build_gazebo_magnetic_fix.py \
  --px4-root /home/keyslav/PX4-Autopilot \
  --server-config /home/keyslav/.simulation-gazebo/server.config \
  --output /home/keyslav/ros2_ws/src/.drone-v2-validation/magnetic-local
```

A pasta de saída deve ser nova. Ela conterá `px4`, auxiliares de inicialização,
`server.config`, comandos de compilação e hashes. O script rejeita um bridge
que não corresponda à transformação legada conhecida. Validado também com a
saída `magnetic-repro`; isso não aprova automaticamente outras versões Gazebo.

## 2. Executar um cenário

Use o overlay construído com **os dois pacotes v2** e Python com Ruckig 0.19.4.
O heartbeat GCS requer `pymavlink`; a análise posterior de ULog usa `pyulog`.
No ambiente de validação desta máquina:

```bash
source /opt/ros/jazzy/setup.bash
source /home/keyslav/ros2_ws/src/.drone-v2-validation/install/local_setup.bash
cd /home/keyslav/ros2_ws/src
export PYTHONPATH="$PWD/.drone-v2-validation/python:$PWD/drone_inspetor:$PYTHONPATH"
python3 drone_inspetor/tools/validate_gazebo.py \
  --px4-root /home/keyslav/PX4-Autopilot \
  --model-store /home/keyslav/.simulation-gazebo \
  --world-sdf /home/keyslav/.simulation-gazebo/worlds/Plataforma_UERJ.sdf \
  --px4-binary "$PWD/.drone-v2-validation/magnetic-local/px4" \
  --server-config "$PWD/.drone-v2-validation/magnetic-local/server.config" \
  --agent /usr/local/bin/MicroXRCEAgent --gcs-heartbeat \
  --scenario land --output "$PWD/.drone-v2-validation/meu-teste-01"
```

`--preflight-only` encerra antes de OFFBOARD/ARM. Sem essa opção, há comandos de
voo na simulação. Escolhas de `--scenario`:

| Cenário | Sequência |
| --- | --- |
| `land` | ARM, subida de 3 m, pouso |
| `cruise` | Subida, ida/volta de 40 m Norte, pouso |
| `obstacle` | Cilindro físico temporário, GOTO de 12 m com desvio, cancelamento, retorno ao HOME, pouso |
| `rtl` | Subida, afastamento de 12 m, RTL nativo até pouso e desarme |
| `mission` | Missão por MissionNode/CVNode reais; exige `--mission-image`, detecta flare, grava, retorna e desarma |

O ensaio utiliza instância 27, DDS 173, Agent UDP 18889 e GCS localhost 18597.
Execute uma rodada por vez. O wrapper encerra seus processos; não abre GUI ou
QGroundControl. `flight.json` registra comandos, medições e sucesso/falha;
`manifest.json` identifica entradas e processos. Veja também os logs individuais
quando uma falha ocorrer antes da criação de `flight.json`.

Os cenários aprovados não certificam toda a plataforma ou qualquer obstáculo.
O mapa LiDAR é local e horizontal; não prova espaço livre acima. LAND termina
pousado, podendo ainda estar armado; o cenário RTL verifica desarme.
