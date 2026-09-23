# Validação no Gazebo — Plataforma UERJ

Registro de 22/09/2026. Complementa `INIT_SIMULACAO.md` e `VALIDACAO_SIH.md`.
Aqui são usados o mundo e o modelo do usuário, com sensores físicos simulados
pelo Gazebo. O servidor roda headless com NVIDIA; GUI e QGroundControl não foram
abertos nesta etapa. Nenhuma aeronave real participa.

## Isolamento e configuração

- Recursos: `/home/keyslav/.simulation-gazebo/{models,plugins,server.config}`.
- Mundo original: `worlds/Plataforma_UERJ.sdf`; modelo `x500_uerj`, autostart 4030.
- Pose inicial ENU: `-70,-27,57`; instância 27, modelo `x500_uerj_27`, system ID 28.
- Partição Gazebo própria; ROS_DOMAIN_ID 173; MicroXRCEAgent UDP 18889, localhost.
- Parâmetros persistidos copiados do rootfs do usuário para rootfs separado.
  No ensaio, `UXRCE_DDS_PTCFG=1` restringe a descoberta DDS a localhost;
  `UXRCE_DDS_SYNCT=0` mantém timestamps de simulação, como no arquivo do usuário.
- Modelos, plugins e parâmetros de voo originais não foram editados. Ganhos,
  geometria dos rotores e hover são os do 4030 e sua herança.

O bridge inferior original fixa `x500_uerj_0`. A validação usa um YAML próprio
apontando para `x500_uerj_27`. O launch agora aceita `bridges_file:=/caminho/arquivo.yaml`,
permitindo selecionar mundo/instância sem editar o arquivo instalado.

## Rodada 01 — mundo original, sem armar

O Gazebo carregou a plataforma e o PX4 criou o drone. Um observador ROS somente
leitura coletou 15 s de dados reais. Clock e sensores chegaram pelos bridges:

| Fonte | Frequência observada de parede | Último timestamp simulado |
| --- | ---: | ---: |
| Clock | 125,00 Hz | 241,048 s |
| Posição local PX4 | 100,00 Hz | 241,032 s |
| LiDAR horizontal | 30,32 Hz | 241,032 s |
| LiDAR inferior | 50,02 Hz | 241,040 s |

O LiDAR horizontal publicou 1080 feixes, alcance 0,4–12 m, sem retornos finitos
nesse ponto. O inferior mediu 0,177 m. Posição e velocidade PX4 estavam válidas;
estado físico era pousado/desarmado. O status em AUTO_LOITER indicava preflight
incompleto e o console apontava falta de GCS; não foi feita tentativa de ARM para
concluir se isso também impediria OFFBOARD.

Depois de acomodar no piso, a origem do modelo estava em ENU
`(-70,000001; -26,999999; 56,084590) m`. O PX4 estava próximo de `(0,0,0)` local,
com `ref_alt≈56,346 m AMSL`. Isso confirma que coordenadas absolutas do mundo e
coordenadas locais PX4 têm origens distintas.

### Divergência de yaw observada

OdometryPublisher informou quaternion ENU aproximadamente `(0,0,0,1)`: frente
para Leste, equivalente a **90° NED**. O PX4 estimou **43,83° NED**, diferença
de cerca de **46°**. Não é a conversão ENU/NED do software ROS: a divergência
já existe entre groundtruth e estimativa PX4.

O mundo original declara campo magnético ENU `0 0 5e-5` T. A hipótese inicial
era que sua ausência de componente horizontal explicaria a divergência.
**A rodada 02 não confirmou essa hipótese**: alterar somente esse elemento
não corrigiu o yaw; não se deve tratar o campo declarado como o campo efetivo.
Todos os processos da rodada 01 foram encerrados sem armar o veículo.

Evidências: `.drone-v2-validation/gazebo-01/{preflight.json,odometry.txt,px4.log,
gazebo.log,bridge.log,topics.txt}`. O observador foi preservado como
`preflight_probe.py` nessa pasta. Ele usa QoS de sensores; por isso o HOME já
publicado antes da conexão não apareceu nessa coleta. O console confirmou HOME
válido; o nó de produção usa TRANSIENT_LOCAL para telemetria PX4.

## Rodada 02 — cópia com campo magnético coerente com este PX4

A cópia `.drone-v2-validation/gazebo-02/Plataforma_UERJ.sdf` altera somente o
elemento ativo `magnetic_field`. A tabela local
`PX4-Autopilot/src/lib/world_magnetic_model/geo_magnetic_tables.hpp` usa WMM-2020
com data de geração 2024,41257. Sua interpolação bilinear em
`(-22,633890; -40,093330)` fornece:

- Declinação: −23,501513°; inclinação: −44,585411°.
- Intensidade total: 23600,943 nT.
- Vetor ENU em tesla: **`-6.702862487e-6 1.541441500e-5 1.656719454e-5`**.

O cálculo parte de NED: `N=B·cos(I)·cos(D)`, `E=B·cos(I)·sin(D)`,
`Down=B·sin(I)`, depois aplica ENU=`(E,N,-Down)`. Essa escolha busca coerência
com a tabela usada pelo firmware; não é um levantamento geofísico atualizado.
`magnetic-field.json` guarda os valores e hashes das fontes. `gz sdf -k` confirmou
que a cópia é válida.

O ensaio automatizado em `gazebo-02-flight` mediu yaw físico **89,999973° NED**
e estimado **43,778395° NED**: erro **−46,221578°**. O verificador rejeitou o
ensaio antes de ARM (limite 5°). `flight.json` tem lista de comandos vazia;
`frame-check.json` registra a comparação. **Não houve voo no Gazebo.**

## Diagnóstico delimitado do magnetômetro

Pacotes instalados conferidos: `libgz-sim8` 8.15.0 e `libgz-sensors8` 8.2.2.
No [código oficial da versão 8.15.0](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/magnetometer/Magnetometer.cc),
o plugin substitui o campo inicial pelo calculado nas coordenadas geográficas.
Os padrões são `use_units_gauss=true` e `use_earth_frame_ned=true`; o
`server.config` local não os sobrescreve. Isso explica por que editar apenas
`magnetic_field` não é um experimento suficiente para corrigir a orientação.

O bridge local `PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp`,
linhas 231–254, converte a medida por `(x,y,z) → (−y,−x,z)`.
Essa transformação tem determinante −1: é uma reflexão, não uma rotação entre
referenciais destros. O próprio comentário do firmware a apresenta como
compensação de comportamento legado do Gazebo. É um ponto concreto para
investigar, mas a leitura do código não prova isoladamente a causa completa.

### Conferência com ULog existente, sem nova simulação

`tools/inspect_magnetic_log.py` reconstrói o campo terrestre a partir de
`vehicle_magnetometer` (já calibrado, corpo FRD) e
`vehicle_attitude_groundtruth` (corpo FRD para NED), sem usar a atitude estimada
como referência. No log `gazebo-02-flight/rootfs/log/2026-09-22/22_55_33.ulg`,
29 amostras entre 4,496 e 6,456 s produziram:

- Campo NED mediano: `(0,171441; 0,071874; −0,143513)` gauss.
- Declinação reconstruída: **+22,744979°**.
- `EKF2_MAG_DECL` inicial no ULog: **−23,501257°**.
- Diferença: **46,246236°**, compatível com o erro de yaw medido de 46,221578°.

Saída preservada em `gazebo-02-flight/magnetic-diagnosis.json`. Isso evidencia
incoerência entre o campo recebido e a orientação física, antes do controle ROS.
Ainda não separa efeitos do plugin, do bridge e da calibração copiada; tampouco
valida o comportamento durante rotações. O script exige pyulog/numpy e pode ser
executado sobre logs existentes sem iniciar processos de simulação.

Próximo ensaio: comparar campo bruto, campo recebido pelo PX4 e orientação
física em outras poses conhecidas, antes de armar. Uma correção precisa manter unidade,
referencial do mundo e referencial do corpo coerentes em mais de uma orientação.
Não aplicar offset constante de yaw no ROS nem relaxar o limite de pré-voo.
Nenhuma alteração no firmware/plugin original foi feita neste diagnóstico.

Depois dessa correção continuam pendentes voo vertical, cruzeiro, frenagem,
desvio com LiDAR do Gazebo e RTL. Os resultados SIH não substituem esses ensaios.

## Rodadas 03/04 — diagnóstico de orientação, sem ARM

O wrapper agora aceita `--preflight-only`: mesmo que o yaw passe, encerra antes
de solicitar OFFBOARD ou ARM. `--yaw-enu` só é aceito nesse modo.

A rodada 03 revelou que o `px4-rc.gzsim` local lê apenas XYZ de
`PX4_GZ_MODEL_POSE`; os campos de rotação são ignorados. Por isso ela repetiu a
orientação Leste e não conta como teste de outra pose. Na rodada 04, o wrapper
aplicou 90° ENU via `/world/Plataforma_UERJ/set_pose` depois do spawn e aguardou
acomodação. A orientação física confirmou **Norte (−0,000014° NED)**.

| Pose física | Yaw estimado PX4 | Erro de yaw | Declinação reconstruída |
| --- | ---: | ---: | ---: |
| Leste, rodada 02 | 43,778° | −46,222° | +22,745° |
| Norte, rodada 04 | −46,905° | −46,905° | +23,388° |

O erro persiste após girar fisicamente o drone. Isso confirma a inconsistência
magnética em duas poses; não justifica compensar yaw no ROS. A diferença entre
poses também precisa considerar os offsets de calibração copiados. A medição
Norte usa 28 amostras do magnetômetro calibrado nos últimos 2 s do ULog.

Evidências: `.drone-v2-validation/gazebo-04-north/` contém `frame-check.json`,
`magnetic-diagnosis.json`, ULog e manifesto com a requisição de rotação.
`flight.json` confirma `commands=[]`. Todos os processos próprios encerraram;
não houve alteração dos modelos, plugins ou parâmetros originais.

## Correção experimental — rodada 06

Foi compilado somente `GZBridge.cpp` numa pasta isolada, substituído esse objeto
numa cópia da biblioteca e ligado um executável próprio, preservando o build
original. Artefatos e script: `.drone-v2-validation/magnetic-fix/`.
O patch experimental está em `tools/patches/px4-magnetometer-enu-experimental.patch`.
**Ele exige a configuração conjunta abaixo; não aplicar sozinho.**

- Plugin magnetômetro: `use_earth_frame_ned=false`, `use_units_gauss=true`.
- Bridge: campo corporal FLU para FRD por `(x,y,z) → (x,−y,−z)`.
- Unidade permanece gauss, conforme entrada esperada pelo PX4.

A cópia compilada inicialmente precisou dos auxiliares `px4-alias.sh` e
`px4-*` ao lado do executável (rodada 05 encerrou antes do teste). O comando de
link antigo também citava versões de bibliotecas já ausentes; a cópia usou os
links `.so` disponíveis das mesmas bibliotecas. Essa diferença deve constar
na interpretação do experimento; não foi uma recompilação integral do firmware.

Na rodada 06, o ULog mostrou **yaw estimado 90,96°**, próximo do físico 90°,
alinhamento e posição válidos. Isso apoia a correção dos referenciais, mas ainda
falta verificá-la em outra orientação e em movimento. O harness expirou esperando
telemetria/HOME no ROS antes de chegar à conferência de yaw; **não houve ARM**.
Logo, o ensaio não passou como pré-voo integrado. Todos os processos encerraram.

O wrapper aceita `--px4-binary` e `--server-config` para selecionar essas cópias;
os arquivos efetivamente usados e seus hashes ficam no manifesto do ensaio.

## Rodada 07 — pré-voo corrigido aprovado, apontando Norte

O harness deixou de reiniciar o cliente DDS após o boot: `rcS` já configura
domínio, porta e namespace pelo ambiente. A rodada 06 recebia posição, mas
não HOME; reiniciar o writer pode perder essa publicação pouco frequente.
Agora `PX4_UXRCE_DDS_NS=px4_27` explicita o namespace desde a partida.

Com o mesmo binário/configuração experimental e rotação de diagnóstico Norte,
HOME e LiDARs chegaram. Yaw físico −0,000014°, estimado **1,834180°**; erro
**1,834193°**, abaixo do limite 5°. Resultado **PREFLIGHT_PASS**, sem ARM/voo.
Artefatos em `.drone-v2-validation/gazebo-07-magnetic-north/`.
Este resultado junto da rodada 06 apoia a correção em duas orientações.

## Rodada 08 — yaw aprovado, ARM recusado

Com pose Leste e DDS sem reinício, erro de yaw **0,928708°**. O ensaio solicitou
ARM, mas recebeu rejeição PX4 código 1. O console registrou `Arming denied:
Resolve system health failures first` e a falha de pré-voo presente era ausência
de conexão com a estação de controle. Não houve TAKEOFF nem voo.

Próximo ensaio deve incluir conexão GCS, como na sequência do usuário com
QGroundControl, preservando as checagens de saúde. Evidências em
`.drone-v2-validation/gazebo-08-vertical/`; processos próprios encerrados.

## Rodada 09 — primeiro voo vertical Gazebo aprovado

Com a correção magnética isolada, DDS contínuo e `--gcs-heartbeat`, o cenário
concluiu **ARM → TAKEOFF 3 m → LAND**. A GCS mínima `simulation_gcs.py` envia
somente heartbeat MAVLink a `127.0.0.1:18597` (instância 27); não envia comandos
de voo nem muda parâmetros. Não substitui QGroundControl em operação real.

- ARM: 0,22 s.
- TAKEOFF: 11,17 s; velocidade medida máxima 0,743 m/s.
- LAND: 7,73 s; estado final confirmado `POUSADO_ARMADO`.
- Erro máximo de seguimento durante TAKEOFF: 0,668 m.

O teste confirma decolagem/pouso físicos, não desarme automático, cruzeiro,
desvio de obstáculos ou RTL. Mundo, massa, motores e parâmetros originais foram
mantidos; a correção de firmware/configuração continua apenas na cópia de teste.
Logs, comandos, métricas e ULog: `.drone-v2-validation/gazebo-09-vertical-gcs/`.
Todos os processos próprios foram encerrados após o resultado.

## Rodada 10 — cruzeiro e retorno de 40 m aprovados

`--scenario cruise` executou ARM, TAKEOFF 3 m, GOTO 40 m Norte, retorno ao HOME
e LAND usando os LiDARs do Gazebo, sem obstáculos sintéticos. Resultado
`FLIGHT_PASS cruise`; critério: ao menos 2 s contínuos a 3 m/s ±10% em cada trecho.

| Trecho | Duração | Pico de velocidade medida | Cruzeiro contínuo | Erro máximo de seguimento |
| --- | ---: | ---: | ---: | ---: |
| Ida 40 m | 25,40 s | 3,106 m/s | 10,35 s | 0,268 m |
| Volta | 28,23 s | 3,094 m/s | 10,34 s | 0,256 m |

TAKEOFF concluiu em 12,00 s e LAND em 6,28 s, com estado pousado confirmado.
Cada GOTO só conclui após atingir a tolerância de posição e estabilizar a
velocidade; não é apenas aceitação do comando. O teste não injeta obstáculos
e não comprova margem geométrica ou desvio diante de uma obstrução.
Evidências: `.drone-v2-validation/gazebo-10-cruise/flight.json` e ULog associado.

## Rodada 11 — desvio/cancelamento aprovados, pouso reprovado

Foi criado um cilindro estático de raio 0,4 m, altura 8 m, centro ENU
`(-70,-21,59)`: 6 m ao Norte da partida. Possui geometria visual e colisão;
as leituras vieram do LiDAR Gazebo, sem publicação sintética de LaserScan.

O GOTO de 12 m concluiu em 33,71 s, com um desvio; cancelamento posterior
confirmou parada. O ULog de groundtruth mediu distância mínima entre centro do
drone e superfície do cilindro de **1,082 m**, acima de raio do veículo + margem
configurados (0,80 m). Isso não equivale à distância entre as duas superfícies.
O cilindro foi removido antes do teste de cancelamento.

O cenário completo falhou em LAND: na posição após o cancelamento, não havia
o mesmo piso elevado da partida. O drone desceu até `z≈54,75 m` no referencial
local PX4, e a action terminou com perda de OFFBOARD após 42,43 s. Portanto,
não há aprovação de pouso nessa região. O próximo roteiro retorna ao HOME antes
de pousar; não altera tolerâncias nem considera essa rodada aprovada.
Evidências: `.drone-v2-validation/gazebo-11-obstacle/`, incluindo
`physical-clearance.json`; todos os processos próprios encerraram.

## Rodada 12 — desvio, cancelamento, retorno e pouso aprovados

O roteiro passou ao retornar ao HOME antes de LAND. GOTO com obstáculo concluiu
em **29,01 s**, com um desvio e erro máximo de seguimento **0,143 m**.
A distância centro do drone–superfície do cilindro foi **1,093 m** pela estimativa
PX4 e **1,123 m** pelo groundtruth, acima da margem combinada 0,80 m.
O cálculo físico considera a primeira chegada à região do alvo, antes de remover
o cilindro; a volta atravessa o local já livre e não deve entrar nessa métrica.

Cancelamento em movimento confirmou parada. O retorno ao HOME concluiu em
19,38 s e LAND em 7,85 s, com estado final `POUSADO_ARMADO`. Isso valida este
obstáculo cilíndrico e esta rota; não comprova qualquer geometria de plataforma.
Artefatos: `.drone-v2-validation/gazebo-12-obstacle-return/`.
Todos os processos próprios foram encerrados. RTL permanece pendente.

## Rodada 13 — RTL nativo aprovado

O cenário `rtl` decolou 3 m, afastou 12 m ao Norte e solicitou AUTO_RTL.
O PX4 retornou, pousou e desarmou; a action confirmou conclusão física e o
harness verificou também `is_armed=false`. Resultado `FLIGHT_PASS rtl`.
As referências ROS permaneceram inativas durante a autoridade nativa do PX4.
Artefatos: `.drone-v2-validation/gazebo-13-rtl/`; todos os processos encerrados.

A suíte funcional foi repetida após os cenários: **333 passed, 1 skipped**
(`.drone-v2-validation/functional-gazebo-final.txt`). Ainda falta consolidar a
correção magnética e suas instruções de reprodução; os ensaios usam cópia
experimental do executável e configuração de plugins, não os originais.

## Rodada 14 — missão integrada com visão controlada

`--scenario mission --mission-image /caminho/imagem.png` instancia MissionNode,
DroneNode e CVNode reais. O comando de início percorre o tópico público do
dashboard; navegação e serviços CV usam actions/services ROS reais.

A missão de um ponto decolou 3 m, navegou 3 m ao Norte, detectou `flare`, passou
por ESCANEANDO e ESCANEAMENTO_FINALIZADO, solicitou RTL e desarmou após pousar.
O vídeo MP4 gerado foi decodificado: **12 frames, 111377 bytes**.
Resultado `FLIGHT_PASS mission`, com todos os processos próprios encerrados.

A visão recebeu repetidamente a imagem local
`relatorio/Figuras/flare_gazebo_escaneamento.png` (hash no manifesto), utilizando
os pesos reais `flare_yolov8n_detection_300ep.pt` e `corrosion_yolo8n_detection.pt`
em CPU. Isso testa coordenação, inferência e persistência, **não percepção ao vivo
da câmera do Gazebo nem precisão do detector**. O voo/telemetria/LiDARs continuam
reais simulados. O último estado da missão coletado foi RETORNANDO; o harness
confirmou desarme físico antes de encerrar, sem esperar outro tick de estado.

Evidências: `.drone-v2-validation/gazebo-14-mission/{flight.json,mission-states.json,
mission-artifacts/}`. Após permitir opções ROS nos construtores de MissionNode e
CVNode, a suíte passou novamente: 333 testes, 1 ignorado.
