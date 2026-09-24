# Flare: decolagem seguida de retorno — 23/09/2026

Sessão: `~/Drone_Inspetor_Missoes/mission_20260923_215448_zxqkh7vl`.
Os horários abaixo são locais (America/Sao_Paulo).

| Horário | Evento confirmado nos logs ROS |
| --- | --- |
| 21:54:48 | Início da sessão Flare |
| 21:54:50 | ARM concluído |
| 21:54:51 | TAKEOFF solicitado, altitude relativa ao HOME de 20 m |
| 21:55:15 | DroneFSM passou de DECOLANDO para EM_VOO |
| 21:55:19 | Action TAKEOFF falhou: `Telemetria local expirada` |
| 21:55:20 | MissionFSM passou de EXECUTANDO_DECOLANDO para RETORNANDO |
| 21:55:21 | RTL nativo solicitado ao PX4 |
| 21:55:51 | RTL concluído; drone desarmado |

O primeiro GOTO não foi enviado. Estar em EM_VOO não basta para o resultado
TAKEOFF: a action também verifica parada física e falhas de telemetria. A missão
retorna ao HOME quando essa action falha; não foi um pouso por conclusão da Flare.

## Origem do diagnóstico

- ROS: `~/.ros/log/python3_1523253_1790209741668.log` (mission_node) e
  `python3_1523252_1790209741669.log` (drone_node).
- PX4: `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-24/00_11_08.ulg`.
  Leitura dos tópicos vehicle_local_position/vehicle_status com pyulog:
  282.257 posições, maior intervalo entre timestamps registrados de 0,032 s.
  Flags de posição/velocidade inválidas somente na inicialização, até 4,36 s.
  Armado entre 2215,54 e 2258,40 s; OFFBOARD em 2209,54 s e RTL em 2238,05 s.

Logo, o ULog não mostra invalidação da estimativa no voo. O drone_node detectou
mais de 0,5 s **monotônico** sem processar posição válida. Isso aponta para
entrega/processamento ROS (DDS, escalonamento, carga etc.), mas não identifica
qual deles causou o intervalo. Timestamps de simulação no ULog não comprovam
regularidade em tempo real no ROS. O log antigo não permite separar essas causas.
Não se alterou o limite nem se desativou a proteção para ocultar a falha.

## Diário automático para próximas missões

Cada nova sessão cria `events.jsonl` em sua própria pasta. Contém:

- definição validada da missão e configuração do mission_node;
- horário UTC, tempo monotônico decorrido e horário ROS em cada evento;
- mudanças de estado observadas e snapshots das mensagens drone/mission,
  no máximo uma amostra periódica por segundo, além de mudanças imediatas;
- logs ROS de drone_node, mission_node e cv_node: transições internas,
  comandos enviados/resultados e motivos de falha;
- pedidos de cancelamento, fim da sessão e encerramento do nó.

Snapshots cessam ao limpar a sessão. O arquivo permanece aberto para receber
os últimos logs em trânsito; fecha no próximo início ou no encerramento do nó.
Escrita por linha; erro de disco é informado e não interrompe o controle.
O diário é diagnóstico, não uma gravação completa de tópicos via rosbag.

Em uma nova falha de telemetria, a mensagem registra idade da última posição
válida, idade da última mensagem recebida, última rejeição/flags, limite e
intervalo do timer da trajetória. Isso distingue mensagens inválidas de
ausência de callbacks. Não grava imagens ou nuvens nesse arquivo.

## Validação desta alteração

348 testes passaram, 1 ignorado (estilo legado separado). Regressão reproduz
TAKEOFF falhando por telemetria e confirma ARM → TAKEOFF → RTL, sem GOTO.
Teste ROS com MissionNode real em domínio 177 confirmou criação do diário,
recepção de rosout do drone e preservação do motivo de retorno. Nenhum comando
de voo enviado; essa validação não afirma que a interrupção DDS foi corrigida.
