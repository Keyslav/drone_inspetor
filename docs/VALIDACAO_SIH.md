# Ensaios de navegação no PX4 SIH

Receita de ensaio e registro das rodadas históricas. Para montagem do workspace,
use o [README](../README.md); operação diária fica em [EXECUCAO.md](EXECUCAO.md).
Os comandos de preparação abaixo foram atualizados em 28/09/2026; as medições
das rodadas não foram repetidas nesta revisão.

`tools/validate_sitl.py` inicia uma instância própria do PX4 SIH: instância 27,
system ID 28, namespace `px4_27`, domínio DDS 173 e agente UDP na porta 18889.
O cenário usa a dinâmica SIH e os controladores PX4. O lidar e os obstáculos
circulares são sintéticos: o ensaio mede afastamento geométrico, não colisão
física. Não representa o modelo `x500_uerj`/autostart 4030 do Gazebo.

## Executar no ambiente de validação local

Com workspace v2 compilado, PX4 SITL disponível e Ruckig no Python da aplicação:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source .venv/bin/activate
source install/setup.bash
python src/drone_inspetor/tools/validate_sitl.py \
  --px4-root /home/keyslav/PX4-Autopilot \
  --agent /usr/local/bin/MicroXRCEAgent \
  --output "$PWD/src/.drone-v2-validation/sitl-novo-ensaio"
```

Use uma pasta de saída nova por rodada. O script guarda `flight.json`, logs ROS,
log PX4, ULog e parâmetros da instância, e encerra seus processos ao sair.
`--deceleration 1.0` aplica um override experimental ROS de 1 m/s², tanto ao
perfil quanto ao envelope do planejador. Não altera parâmetros/arquivos do PX4
nem o YAML do projeto. Sem essa opção, vale o padrão do nó: 2 m/s².

Para investigar pausas, acrescente `--diagnose-timing`. Essa opção instrumenta
somente o ensaio: `callback-timing.json` registra duração de parede/CPU e maior
intervalo entre entradas dos callbacks; `executor-stacks.txt` captura pilhas quando
o timer de controle demora mais de 250 ms para voltar a executar. O watchdog
não altera os limites de reação/telemetria e não registra valores de variáveis
locais. Duração de callback e atraso entre callbacks são medidas distintas.

O cenário inclui ARM, decolagem de 3 m, GOTO livre de 40 m, volta, GOTO de 12 m
com obstáculo a 6 m, cancelamento em movimento e LAND. `SITL_PASS` exige os
comandos concluídos, parada confirmada no cancelamento, pelo menos 2 s contínuos
de cruzeiro no primeiro GOTO (referência e velocidade medida dentro de 10% de
3 m/s) e afastamento da superfície do obstáculo de ao menos raio do veículo +
margem. Lacunas de amostragem maiores que 150 ms não comprovam cruzeiro contínuo.

`--scenario obstacle` omite os dois trechos livres de 40 m e a exigência de
cruzeiro; executa ARM, TAKEOFF, desvio, cancelamento e LAND. A saída identifica
`SITL_PASS obstacle` ou `SITL_PASS full`: o cenário reduzido não aprova cruzeiro.
`--scenario land` executa apenas ARM, TAKEOFF e LAND; serve para verificar a
transferência ao pouso nativo sem repetir os percursos de navegação.
O prazo de observação do cliente inclui o prazo de execução do servidor, seus
30 s máximos de frenagem e 10 s de transporte. Isso não amplia o prazo de GOTO
no servidor (120 s). Falhas de transporte também preservam o intervalo do comando
e suas métricas no JSON, em vez de desaparecerem do relatório.

Os relatórios incluem velocidade máxima medida/referenciada, erro máximo de
seguimento, duração de cruzeiro e afastamento mínimo dos obstáculos. São
calculados também em ensaios que falham. Limites de jerk/continuidade são
verificados separadamente nos testes de navegação; as amostras de 20 Hz deste
relatório não provam esses limites em todos os setpoints. RTL ainda precisa
de cenário próprio.

Novas amostras identificam se a referência ROS está ativa (OFFBOARD). Erro de
seguimento e velocidade de referência só consideram esse intervalo. Em modo
nativo, o último setpoint ROS não é a referência do controlador PX4; sem amostras
ativas, essas métricas são `null`. Relatórios antigos de LAND podem mostrar um
erro de vários metros contra o hover antigo: isso não mede seguimento do pouso.

## Aproximação terminal

Após a rodada 08, foi acrescentado um trecho lento antes da parada final. O
planejador usa a menor distância restante entre posição medida e referência,
projetada no eixo do segmento. Reserva distância para acomodação em velocidade
baixa e para a frenagem final, aplicando o envelope limitado por jerk ao trecho
restante. A aceleração usada nesse envelope é a da referência: isso não equivale
a identificar a resposta física do veículo pela velocidade/aceleração medidas.

`arrival_approach_velocity=0.6` m/s e `arrival_settling_time=2.0` s ficam no grupo
`navigation` do YAML. O limite terminal só diminui durante cada segmento. Seu
piso positivo mantém Ruckig em controle de posição e evita aproximação assintótica;
o perfil ainda termina no alvo com velocidade e aceleração zero. O limite zero de
um obstáculo ou do controle de seguimento continua prevalecendo. A mesma regra
de aproximação é usada na decolagem, respeitando o limite vertical.

Uma planta reduzida com massa/arrasto/ganhos SIH reproduziu a falha anterior:
erro de 1,004 m sem aproximação terminal. Com a aproximação, erro máximo de
0,870 m, ultrapassagem de 0,518 m e chegada confirmada dentro de 0,2 m e 0,1 m/s.
O teste também verifica cruzeiro e jerk. É uma regressão do mecanismo observado;
não inclui o EKF e a atitude completos. O ensaio SIH continua necessário.

## Rodada 07: frenagem na chegada

Com desaceleração de 2 m/s², ARM e TAKEOFF passaram. O primeiro GOTO de 40 m
abortou ao ultrapassar o limite de seguimento de 1 m; os demais passos não foram
executados. A referência chegou ao destino e parou, mas o veículo o ultrapassou.

A inspeção de `sitl-flight-07/rootfs/log/2026-09-22/00_12_16.ulg` encontrou:

- Sem saturação aparente: comando máximo dos motores menor que 0,596; inclinação
  aproximada de 19,5°, para limite de 45°.
- SIH com massa de 1 kg e arrasto linear de coeficiente 1. A 3 m/s, isso exige
  cerca de 3 m/s² de compensação de arrasto. A compensação acumulada pelo
  controlador demora a diminuir quando a referência freia.
- Primeiro erro de seguimento registrado no JSON: 1,01273 m. Perto do aborto,
  velocidade X estimada de 0,049 m/s versus 0,314 m/s no groundtruth; ultrapassagem
  X estimada de 1,013 m versus 1,384 m real no simulador.
- A diferença entre derivada da posição estimada e velocidade estimada também
  aparece no ULog; não foi criada pela conversão ROS. Sem resets XY/velocidade
  durante o GOTO.

A hipótese de trabalho combina a resposta do controlador ao arrasto com erro
de estimativa no final. A rodada seguinte testa desaceleração de 1 m/s² com o
mesmo limite de seguimento, sem assumir que a sintonia SIH vale para o Gazebo.
O limite de 1 m permanece ativo; não foi ampliado para fazer o ensaio passar.

## Rodada 08: desaceleração experimental de 1 m/s²

ARM e TAKEOFF passaram. No GOTO de 40 m, a referência atingiu 3,000 m/s e a
velocidade medida atingiu 3,165 m/s. Houve **4,95 s contínuos de cruzeiro** dentro
da tolerância de 10%. A chegada ainda falhou: depois que a referência parou,
o erro ultrapassou 1 m (máximo amostrado de 1,014 m).

No primeiro erro, a referência X era 39,787 m e a posição estimada X era
40,781 m; a velocidade X estimada ainda era 0,112 m/s. O comando terminou com
falha depois de confirmar parada. Volta, obstáculo, cancelamento e LAND não
foram executados nessa rodada. Dados em `.drone-v2-validation/sitl-flight-08/`.

**Conclusão:** reduzir a desaceleração isoladamente não resolveu a chegada.
O padrão de 2 m/s² e o limite de seguimento de 1 m do projeto foram preservados.
A próxima investigação deve confrontar o estado medido no início da frenagem,
o envelope de parada e a resposta do controlador; novas rodadas não devem apenas
variar constantes até obter um resultado verde. A dinâmica do `x500_uerj` segue
sem validação por estes ensaios SIH.

## Rodadas 09 e 10: erro transversal e temporização

Na rodada 09, já com aproximação terminal e desaceleração padrão de 2 m/s²,
ARM e TAKEOFF passaram. O GOTO abortou **antes da aproximação final**, com a
posição X em 22,174 m, para destino a aproximadamente 40 m. No primeiro erro,
a diferença para a referência tinha 0,606 m longitudinal, 0,820 m lateral e
0,181 m vertical. A rodada não validou a chegada terminal.

O limitador de avanço considerava somente a projeção longitudinal. Foi corrigido
para descontar o erro transversal do orçamento `reference_lead_limit`: a margem
longitudinal disponível passa a ser a raiz de `max(0, limite² - erro_transversal²)`.
Assim, o avanço desacelera quando a correção lateral consome a margem. O limite
tridimensional de 1 m continua interrompendo a navegação. Dois testes adicionais
cobrem esse comportamento em rotas Norte e Leste.

A rodada 10 incluiu a correção, mas falhou durante TAKEOFF: intervalo de trajetória
de **631 ms**, acima do orçamento de reação de 350 ms, seguido de telemetria local
expirada. Não chegou a executar GOTO. A causa da pausa do executor ainda não foi
identificada; a rodada não prova nem refuta a correção lateral/terminal. Os processos
dos dois ensaios foram encerrados e os dados estão nas pastas `sitl-flight-09/10`.

Estado após integração: **308 testes funcionais passaram, 1 ignorado**, build dos
dois pacotes e lint fatal passaram. A validação de chegada/desvio/cancelamento/pouso
em SIH permanece pendente; não houve aprovação do cenário completo.

## Rodada 11: chegada confirmada e estabilização antes do segundo desvio

Com as correções lateral/terminal e diagnóstico de temporização, ARM, TAKEOFF,
GOTO livre de 40 m e volta passaram. Ida e volta duraram aproximadamente 47 s
cada. Na ida, o cruzeiro dentro da faixa de 10% durou apenas 0,15 s contínuos;
na volta, 2,15 s. Portanto, o critério de cruzeiro do primeiro trecho ainda não
foi satisfeito, mesmo com chegada confirmada nos dois sentidos.

O primeiro desvio foi concluído e o destino original retomado. O drone permaneceu
parado na referência enquanto aguardava a velocidade medida estabilizar para outro
desvio. O prazo de 15 s expirou. A distância mínima estimada à superfície do
obstáculo foi 2,612 m; cancelamento e LAND não foram executados.

Uma reprodução geométrica dos raios com as poses registradas retornou
`braking_for_detour`; ao fornecer velocidade zero, encontrou um desvio observado.
Essa reprodução usa a pose amostrada, não substitui o histórico exato de aquisição
do sensor. O diagnóstico genérico de sensor/corredor indisponível foi separado em
sensor inválido, ausência de corredor, parada não estabilizada e posição não
estabilizada. Novos ensaios também registram `last_decision` em cada amostra.

A pausa de 631 ms da rodada 10 não reapareceu. Medições da rodada 11:

| Callback | Maior duração | Maior intervalo entre entradas |
| --- | ---: | ---: |
| Trajetória/setpoint | 8,59 ms | 121,47 ms |
| Posição local PX4 | 0,77 ms | 126,12 ms |
| Lidar horizontal | 3,08 ms | 131,24 ms |

Não houve captura de pilha pelo watchdog de 250 ms. Esses dados não identificam
a causa da pausa anterior, mas não sustentam tratá-la como custo habitual do
gerador de trajetória. Fontes: `sitl-flight-11/flight.json`, `callback-timing.json`
e `executor-stacks.txt`.

Foi corrigido ainda o valor reportado de referência em hover: após limpar o perfil,
o diagnóstico retornava a posição medida, embora o setpoint mantivesse a posição
estática. As métricas antigas nesse intervalo podiam indicar erro zero indevidamente.
Os setpoints de hover foram preservados; o novo teste compara a referência reportada
com a posição efetivamente comandada.

Após essas correções: **309 testes funcionais passaram, 1 ignorado**; build dos
dois pacotes, lint fatal e `git diff --check` passaram. O cenário completo continua
pendente, principalmente cruzeiro na ida e estabilização antes de novo desvio.

## Rodadas 12 e 13: admissão e estabilização física

Na rodada 12, ARM passou, mas TAKEOFF foi rejeitado quando a velocidade estimada
no solo oscilou ligeiramente acima de 0,1 m/s. A admissão agora exige que a
referência anterior tenha terminado a frenagem; a decolagem efetiva continua
aguardando a parada medida. Enquanto TAKEOFF estiver pendente, uma oscilação do
indicador de pouso não pode promover o estado diretamente para EM_VOO. Dois
testes cobrem a espera e a rejeição enquanto a referência ainda está em movimento.

A rodada 13 usou `--scenario obstacle`. ARM e TAKEOFF passaram; houve dois
desvios completos, mas GOTO não terminou em 120 s. O cliente antigo encerrou
antes de observar o resultado final da frenagem. Cancelamento/LAND não rodaram.
A superfície do obstáculo permaneceu a pelo menos **1,690 m** da posição estimada;
erro de seguimento máximo amostrado **0,964 m**. Esses dois números foram
recalculados das amostras com obstáculo, pois o relatório antigo omitia GOTO
quando o cliente expirava.

As esperas em GIRANDO_INICIO foram de **22,50 s e 24,15 s**. Yaw alinhou em
aproximadamente 2,8 s; a espera restante veio de posição/velocidade ainda fora das
tolerâncias. Comparação com groundtruth SIH no ULog `17_17_36.ulg`:

| Medida durante a espera | Primeira | Segunda |
| --- | ---: | ---: |
| Velocidade mediana estimada | 0,272 m/s | 0,241 m/s |
| Velocidade mediana groundtruth | 0,267 m/s | 0,218 m/s |
| Erro vetorial mediano da estimativa de velocidade | 0,296 m/s | 0,310 m/s |
| Erro mediano de posição estimada para a referência | 0,323 m | 0,284 m |

Há movimento real e erro de estimativa, não apenas conversão incorreta no ROS.
Após o giro, o erro yaw estimado/setpoint RMS foi 0,24°/0,17°. A referência
translacional estava fixa, com feedforward de velocidade/aceleração zero; não
houve indício de saturação. Esses dados não demonstram windup nem justificam
alterar ganhos. O SIH injeta ruído de aceleração em voo com sigma
[0,5; 1,7; 1,4] m/s² (`simulator_sih/sih.cpp`); sua influência no EKF é hipótese
para investigação, não causa isolada demonstrada por este registro.

O callback de trajetória durou no máximo 23,36 ms, com intervalo máximo entre
entradas de 107,43 ms. O watchdog não capturou pausa acima de 250 ms.

## Mudanças depois desses ensaios

- Candidatos a desvio consideram a extensão observada da saída rumo à missão,
  além da primeira perna livre. Saída parcialmente oculta permite progresso
  gradual; não autoriza viajar por espaço desconhecido.
- Depois de girar, a translação aguarda posição e velocidade estabilizadas.
- Em PLANANDO, um obstáculo já observado pode selecionar a próxima perna antes
  do giro. Isso evita alinhar ao destino original e, logo depois, girar novamente
  para outro desvio. Setor fora do FOV, sem retorno bloqueando a rota, ainda permite
  girar para observar. A translação revalida cobertura em cada ciclo.
- Avaliação e aceitação de desvios são compartilhadas entre preparação e movimento;
  preservam missão, limite de desvios e detecção de repetição.
- O harness preserva falhas e espera a resposta do servidor após a frenagem.

Após a integração: **319 testes passaram, 1 ignorado**, build dos dois pacotes,
lint fatal e `git diff --check` passaram. A rodada 14 testa o planejamento antes
do giro. A aprovação do cenário completo e do modelo Gazebo 4030 permanece pendente.

## Rodada 14: desvio/cancelamento concluídos, conflito de autoridade em LAND

Com planejamento antes do giro, ARM, TAKEOFF e GOTO com obstáculo passaram.
GOTO concluiu dois desvios em **111,39 s**; distância mínima estimada à superfície
**1,662 m**, erro máximo de seguimento **0,871 m**. O cancelamento de um novo GOTO
em movimento confirmou parada. O cenário completo reduzido ainda falhou:
**LAND expirou em 120 s**.

O callback ROS publicava hover mesmo após sair de OFFBOARD. No PX4 local,
`MulticopterPositionControl.cpp` lê `trajectory_setpoint` também em modos nativos
com controle de posição habilitado. O ULog `17_31_41.ulg` confirmou setpoints
alternando entre a descida nativa e a referência ROS antiga. Groundtruth chegou
ao solo, mas a conclusão de pouso não foi confirmada pela action. Não basta
observar a mudança de modo ou o ACK para considerar LAND concluído.

A publicação agora para quando o veículo armado não está em OFFBOARD, incluindo
AUTO_LAND, AUTO_RTL, AUTO_MISSION e POSCTL. Durante a espera pela mudança efetiva
de modo, o stream OFFBOARD continua. Desarmado, a pré-publicação permite iniciar
o próximo ciclo. Seis testes dos callbacks verificam esses casos. A entrada
inicial em OFFBOARD é preparada no solo; reentrada em voo a partir de outro modo
exige um protocolo explícito de transferência, ainda não implementado.

## Rodada 15: pouso nativo confirmado

Ensaio focado `--scenario land`: **ARM, TAKEOFF e LAND passaram**. TAKEOFF levou
18,82 s e LAND **11,08 s**, com estado físico POUSADO_ARMADO observado antes de
concluir a action. LAND não promete auto-desarme; RTL, pelo contrato atual, exige
pouso e desarme e continua sem ensaio próprio. O harness encerrou seus processos.

Evidências em `.drone-v2-validation/sitl-flight-14-pre-turn/` e
`.drone-v2-validation/sitl-flight-15-land/`. **325 testes funcionais passaram,
1 ignorado**, build dos dois pacotes e lint fatal passaram após o ajuste de
autoridade. A correção posterior das métricas de referência inativa passou nos
5 testes específicos do harness. O cenário completo, cruzeiro na ida, RTL e o
Gazebo 4030 permanecem pendentes.
