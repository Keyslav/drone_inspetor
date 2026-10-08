# Monitor do drone

Guia da janela de telemetria: como acessá-la e interpretar seus dados. Para
iniciar os produtores de tópicos e escolher um perfil, consulte
[EXECUCAO.md](EXECUCAO.md); montagem e build ficam no [README](../README.md).

Na tela inicial do dashboard, clique em **Monitor do drone** ou pressione
**Ctrl+M**. A janela pode ficar em outro monitor; fechar e reabrir conserva a
seleção do tópico e o filtro. Fechar o dashboard também fecha essa janela.

Para abrir somente o monitor, após compilar e carregar o workspace:

```bash
ros2 run drone_inspetor monitor_node
```

Para escolher explicitamente o relógio ROS ao acompanhar uma simulação:

```bash
ros2 run drone_inspetor monitor_node --ros-args -p use_sim_time:=true
```

Isso não inicia `/clock`. A idade e frequência mostradas na tela continuam
baseadas na recepção monotônica local, conforme explicado abaixo.

O executável inicia apenas a interface e as assinaturas de telemetria. Não
inicia drone_node, mission_node, câmeras, simulador nem publica comandos de voo.
Os produtores dos tópicos devem estar rodando no mesmo domínio ROS.

## Visão geral

- Estado do drone, modo PX4, armamento, pouso, failsafe, trajetória e desvio.
- Missão, etapa, ponto de inspeção, objeto alvo e pedido de cancelamento.
- Bateria: carga, tensão, corrente e aviso informado pelo PX4.
- Posição e velocidade medidas, validade da estimativa e referência publicada.
- Gráfico dos últimos 60 segundos de velocidade medida e referência.
- Disponibilidade de LiDAR horizontal, inferior e profundidade, com menor
  retorno finito válido. Ausência de retorno não significa espaço livre.

Posição e vetores do PX4 usam NED: Norte, Leste, Abaixo. Altura sobre a origem
local e velocidade de subida são exibidas positivas para cima. A altura sobre
a origem não é distância ao solo. Referências publicadas são comandos
observados no tópico; não comprovam aceitação ou execução pelo PX4.

## Tópicos e campos

A segunda aba apresenta as nove fontes, nomes ROS completos, frequência e idade
da última recepção. Selecionar uma linha abre seus campos; o filtro busca pelo
nome do campo, como `state`, `velocity` ou `failsafe`. Mensagens de estado mantêm
todos os campos nativos; scans mostram apenas um resumo para manter a GUI leve.

| Fonte | Tópico padrão | Expiração visual |
| --- | --- | --- |
| Drone | `/drone_inspetor/interno/drone_node/drone_state` | 2 s |
| Missão | `/drone_inspetor/interno/mission_node/mission_state` | 3 s |
| Estado PX4 | `/fmu/out/vehicle_status_v1` | 1 s |
| Posição local | `/fmu/out/vehicle_local_position` | 1 s |
| Bateria | `/fmu/out/battery_status` | 3 s |
| Referência | `/fmu/in/trajectory_setpoint` | 0,5 s |
| LiDAR horizontal | `/drone_inspetor/externo/lidar/scan` | 1 s |
| LiDAR inferior | `/drone_inspetor/externo/lidar_down/scan` | 1 s |
| Profundidade | `/drone_inspetor/interno/depth_node/scan` | 1 s |

Os nomes exibidos respeitam remaps ROS. Por exemplo:

```bash
ros2 run drone_inspetor monitor_node --ros-args \
  -r /fmu/out/vehicle_status_v1:=/px4_27/fmu/out/vehicle_status_v1 \
  -r /fmu/out/vehicle_local_position:=/px4_27/fmu/out/vehicle_local_position \
  -r /fmu/out/battery_status:=/px4_27/fmu/out/battery_status \
  -r /fmu/in/trajectory_setpoint:=/px4_27/fmu/in/trajectory_setpoint
```

**Sem dados** significa que nenhuma mensagem chegou. **Desatualizado** significa
que o intervalo de recepção excedeu o limite da tabela. Nessa situação, os
cartões deixam de mostrar valores atuais e os últimos dados ficam disponíveis,
identificados como antigos, na inspeção de campos. NaN, estimativas inválidas e
carga de bateria desconhecida aparecem como `—`, não como zero.

Idade e Hz usam recepção local e relógio monotônico; não medem latência de rede
nem garantem que a amostra original seja recente. Se um produtor republicar um
valor antigo, a recepção será atual. Esses limites são de visualização e não
alteram nenhum timeout do controlador. Sensores opcionais desligados podem
permanecer sem dados. A GUI atualiza a 5 Hz, com buffers limitados.

## Validação

Testes cobrem contratos ROS reais, dados ausentes/expirados, NaN, unidades NED,
imutabilidade, concorrência, remaps, filtro e abertura/fechamento da janela.
Imagens de apresentação geradas durante o desenvolvimento usam dados de exemplo.
