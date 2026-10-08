# Implementação v2.0 — registro de continuidade

Consolidação registrada em 23/09/2026. Branch local de ambos os repositórios: `v2.0`. A implementação foi consolidada em commits locais; não houve publicação
remota desta etapa. O diagnóstico histórico e as etapas
estão em `PLANO_MELHORIAS_V2.md`; este arquivo registra execução posterior.

## 04/10/2026 — vídeo WebRTC, JPEG e dashboard no navegador

- Gateway opcional `--webrtc` com aiortc; JPEG mantido. Interface compartilhada
  pelo navegador e APK com seleção Automático/WebRTC/JPEG na aba Câmeras.
- Só transmite o canal visível; ampliação reutiliza conexão; expiração de fonte,
  fallback, encerramento de sessões e limite de quatro espectadores.
- Ambiente isolado `.webrtc-venv`, scripts `mobile/setup-webrtc.sh` e
  `mobile/run-gateway.sh`; precedência de dependências corrige conflito com
  `PYTHONPATH` herdado do ROS. Sem alterar Python global.
- APK preview3 compilado, lint sem erros, assinatura e assets verificados.
- 45 testes Python, 16 JavaScript; vídeo real no navegador com fonte sintética
  e matriz de 16 verificações de layout. Sem voo ou validação em celular físico.
- Uso e limites em [ANDROID.md](ANDROID.md); teste visual reproduzível em
  `test/manual_mobile_video.py`. CPU/banda/latência no companion ainda por medir.

## Atualizações posteriores à consolidação

- **04/10:** Android preview2 inclui o dashboard no APK e abre sem servidor.
  Interface fixa por abas, seletores e paginação, adaptada a retrato/paisagem
  16:9, 21:9, 4:3 e proporção interna do Fold7. Mantida a aparência do dashboard
  de operação. Rotação/folding preservam WebView e estado de navegação; teclado
  e recortes tratados por insets. Detalhes e matriz em [ANDROID.md](ANDROID.md).
  Instalada a skill `frontend-design` do repositório oficial `anthropics/skills`.

- **03/10:** iniciador GUI/CLI permite selecionar arquivos de configuração e
  nós individuais (24 testes focados). Gateway HTTP autenticado, página móvel
  responsiva e projeto Android adicionados; testes HTTP/API e DDS sintético
  realizados, sem voo novo. SDK/Gradle instalados e APK debug gerado, com
  assinatura verificada; teste físico pendente. Copiloto Jev/OpenAI, voz Android,
  auditoria e ponte MCP stdio implementados com propostas revisáveis e modo
  shadow padrão. Sem chaves de API, não houve inferência paga nem avaliação dos
  modelos reais. Uso em [ANDROID.md](ANDROID.md), [COPILOTO.md](COPILOTO.md) e
  [MCP_DRONE.md](MCP_DRONE.md).
  Verificação: 111 testes Python/ROS (incluindo DDS sintético), 9 JavaScript,
  build colcon temporário e sessão MCP real em loopback aprovados. Interface
  conferida em largura de 393 px, sem overflow horizontal. Sem voo nesta etapa.

- **24/09:** reorganização visual e radar responsivo documentados em
  [DASHBOARD_V2.md](DASHBOARD_V2.md); seleção de pesos em [MODELOS_CV.md](MODELOS_CV.md).
- **26/09:** iniciador gráfico e menu CLI com perfis compartilhados. A entrega
  registrou 16 testes focados, build temporário e verificação dos executáveis;
  não acrescentou ensaio de voo. Uso em [EXECUCAO.md](EXECUCAO.md).
- **28/09:** revisão dos guias de montagem/execução e criação do
  [índice documental](README.md). Os resultados históricos abaixo foram preservados.

## Estado registrado na consolidação de 23/09

- Extração final da GUI concluída: seletor/detalhes de modelos e janela de
  análise em componentes próprios; seleção e detalhes preservados ao reabrir.
- Build final dos dois pacotes e checagem estática de erros aprovados.

- **339 testes funcionais passaram, 1 ignorado** na última execução
  (`.drone-v2-validation/functional-delivery-final.txt`). Estilo legado permanece
  como passivo separado; isso não significa aprovação de flake8/pep257 completos.
- Gazebo com Plataforma_UERJ/x500_uerj/4030: voo vertical, ida/volta de 40 m,
  desvio de cilindro físico, cancelamento, retorno ao HOME, pouso e RTL com
  desarme foram aprovados. Detalhes em `docs/VALIDACAO_GAZEBO.md`.
- Cruzeiro de 3 m/s ±10% por 10,35/10,34 s; erro máximo de seguimento 0,268 m.
  No desvio: distância física centro–superfície 1,123 m, margem exigida 0,80 m,
  erro máximo de seguimento 0,143 m.
- Esses resultados exigiram correção conjunta de campo ENU no plugin e FLU→FRD
  no bridge PX4. Foi usado executável/configuração isolados, preservando os
  originais. Usar só o projeto ROS com o firmware original mantém o erro de yaw.
- DDS não é reiniciado após o boot, preservando HOME. O harness usa GCS mínima
  local; a operação normal do usuário continua incluindo QGroundControl.
- O pouso após cancelamento deve ocorrer numa área com piso: o ensaio retorna
  ao HOME. A rodada 11 falhou ao pousar fora do piso elevado; não foi aprovada.
- Fora de OFFBOARD, o nó armado cede os setpoints ao PX4 nativo. Reentrada em
  OFFBOARD em voo requer protocolo explícito, ainda não implementado.
- Histórico SIH em `docs/VALIDACAO_SIH.md`: expôs correções úteis, mas seu cenário
  completo não foi aprovado. Os resultados Gazebo acima têm escopo próprio.
- Receita da correção disponível em `docs/REPRODUZIR_TESTES_GAZEBO.md`.
  Auditoria da entrega e limites estão em `docs/AUDITORIA_ENTREGA_V2.md`.

## Implementado e validado no escopo da entrega

- Navegação pura em `navigation/`: perfil Ruckig 0.19.4 local de estado a estado,
  p/v/a coerentes, limites de velocidade/aceleração/jerk, envelope de frenagem,
  mapa métrico NED com validade temporal e volume do veículo inflado.
- `Trajectory` integra planejamento e perfil às FSMs. Desvio após parada, retomada
  do alvo original, vertical com limite próprio, yaw em graus/s, chegada exige
  posição e velocidade reais. STOP preserva referência durante frenagem.
- Sensor adapter lê LaserScan original, preserva ausência de retorno e descarta
  dados vencidos. Descida GOTO exige distância inferior válida. LiDAR horizontal
  não comprova espaço livre acima: esta limitação permanece explícita.
- Missão: repositório/modelos/configuração/sessão extraídos, FSM autoridade única,
  clientes Action/CV com identidade de operação e deadlines monotônicos.
- CV/câmera: registro de modelos, inferência, buffer fresco de detecções e gravador
  compartilhado com sincronização extraídos. GUI: apresentação tipada e widgets
  separados. Build/launch/dependências e documentação revisados.

## Evidências históricas da primeira integração

- 34 testes matemáticos/geometria de navegação passaram.
- 10 testes de integração de trajetória/FSM passaram: vertical, curto, diagonal,
  yaw, obstáculo com retomada, perda de scan, STOP e HOME deslocado.
- Instanciação de DroneNode no overlay v2 passou (sandbox limita transporte DDS).
- Agente GUI: 11 testes GUI/build passaram.
- Agente percepção: 23 testes puros + 4 callbacks com interfaces ROS passaram.
- Esses resultados não são validação de voo. PX4/SITL e a suíte conjunta ainda
  precisam ser executados após finalizar todas as integrações.

## Resultados adicionais e correções após a primeira integração

- Build novo dos dois pacotes passou em `.drone-v2-validation` no workspace.
- Suite funcional conjunta chegou a 239 casos (antes do último mapper): 236
  passavam; 3 stubs de comandos precisavam do novo `px4_target_system_id`.
  O stub foi corrigido; suíte direcionada navegação/actions/telemetria: 38 passaram.
- Percepção/mídia/sensores: 62 testes, mais smoke real de vídeo MJPG, passaram.
- Missão: 39 testes, incluindo ciclo completo com doubles ROS, passaram.
- Sensor adapter agora associa scan ao histórico de pose, conserva idade original
  de aquisição, recupera reinício do relógio e desconta descida desde o scan inferior.
  Cinco testes específicos passaram.
- Telemetria foi extraída para mapper que preserva o Z legado (posição para cima,
  derivadas NED), evitando misturar convenções no cálculo de movimento.
- PX4 SIH isolado entrou em OFFBOARD, ARM e TAKEOFF com DDS real. O primeiro GOTO
  abortou por erro de seguimento >1m; isso é uma falha de validação registrada,
  não um teste de voo aprovado. A referência agora reduz avanço quando o veículo
  atrasa, inclusive no spool da decolagem.
- SIH 05: ARM/TAKEOFF passaram; GOTO abortou após atraso do executor de cerca
  de 256ms, com frenagem confirmada. O limite fixo de 200ms foi substituído pelo
  menor orçamento entre reação e validade da telemetria (350ms nos defaults).
  A integração continua limitada a 100ms; tempo perdido não vira salto de posição.
  Intervalo negativo ou acima do orçamento continua abortando e frenando.
  O diagnóstico informa intervalo e limite; o harness SIH registra o intervalo.
- Após essa correção, 17 testes de integração passaram, incluindo atraso de
  260ms tolerado, interrupções de 420ms/10s e relógio regressivo. Lint fatal
  dos três arquivos alterados e `git diff --check` passaram.
- O simulador usa instância 27/system ID 28/domínio DDS 173. DDS localhost exige
  UXRCE_DDS_PTCFG=1 nesse firmware; alterar MAV_SYS_ID após iniciar commander não
  muda o ID recebido por ele. O nó agora tem `px4_target_system_id` explícito.
- Cobertura angular analítica integrada em `navigation/obstacle_geometry.py` e
  `obstacles.py`: considera células completas, FOV desconhecido e alcance máximo;
  invalida frame vazio e trata retorno abaixo do mínimo como obstáculo próximo.
  Não bloqueia avanço por hits inteiramente atrás do volume do drone.
  27 testes passaram, incluindo comparação com um verificador por pontos.
- SIH 06 passou ARM, TAKEOFF e o primeiro GOTO de 12m. O GOTO de volta ficou
  retido em GIRANDO_INICIO: a condição de parada medida restaurava o yaw antigo
  quando a velocidade oscilava em hover. Corrigido para manter o giro enquanto
  a referência translacional está parada; a FSM continua exigindo parada medida
  antes de entrar nesse estado. 18 testes de integração passaram após o ajuste.
  Essa correção de yaw e a cobertura angular nova ainda exigem nova rodada SIH.
  No trecho de 12m, pico da referência foi 1,72m/s e velocidade medida 1,93m/s;
  cruzeiro configurado de 3m/s ainda não foi validado. Maior intervalo de tick
  registrado foi 156ms (esta rodada não reproduziu o atraso de 256ms da anterior).
- Validação final desta rodada: 265 testes funcionais passaram, 1 ignorado;
  build dos dois pacotes passou; lint fatal (E9/F63/F7/F82) de código, testes e
  ferramentas passou. Flake8 completo/pep257 ainda têm o passivo já registrado.
  SIH 06 encerrou por timeout de resposta no retorno; os processos foram limpos.

## Registros históricos de implementação

### Auditoria de coordenadas e ambiente do usuário

- Conversões de eixos e altura centralizadas em `common/coordinates.py`.
  Navegação usa `global_to_ned_offset()`; API antiga preservada e identificada
  como Norte/Leste/Cima. Corrigidos comentários enganosos de yaw e das interfaces.
- Monitor informa frame/unidades por tópico; mapa identifica altitude AMSL.
- 295 testes funcionais passaram (1 ignorado); build dos dois pacotes e lint
  fatal passaram após a revisão de coordenadas. Sem mudança de campos ROS.
- Auditoria estática do modelo real do usuário registrada em `INIT_SIMULACAO.md`:
  massa 2,326308kg, empuxo/peso ideal 1,50, inércias e geometria divergentes,
  dependências dos comandos e limites da simulação de bateria. Nenhum arquivo
  externo ou parâmetro de voo foi alterado. Naquela etapa, Gazebo 4030 ainda não havia sido ensaiado em voo; os resultados posteriores estão no estado atual.

### Feature adicional: monitor de estados

- Implementado monitor Qt acessível pelo botão **Monitor do drone** no dashboard,
  atalho **Ctrl+M** e executável independente `monitor_node`.
- Foco em DroneStateMSG, VehicleStatus PX4 e MissionStateMSG, com bateria,
  posição/velocidade, setpoints e disponibilidade de sensores como auxiliares.
- Visão geral, gráfico limitado a 60s e inspeção de campos com filtro, idade,
  frequência e estados sem dados/desatualizado. Nomes respeitam remaps ROS.
- Buffer thread-safe com snapshots imutáveis e atualização gráfica a 5Hz.
  Não adiciona comandos de voo nem exige alterações no pacote de mensagens.
- Validação: 285 testes funcionais passaram (1 ignorado), build dos dois pacotes
  e lint fatal passaram. Recepção DDS de seis tópicos e apresentação Qt testadas
  em domínio isolado; executável instalado abriu. Telas verificadas visualmente
  com dados de exemplo. Manual: `docs/MONITOR_DRONE.md`.

### Pendências registradas antes dos ensaios Gazebo

Integração mais recente:

- Retorno da missão corrigido: retenta RTL somente após rejeição explícita antes
  da aceitação, por até 40s, com intervalo de 1s. Timeout incerto, erro de transporte
  e RTL aceito não autorizam reenvio. Parâmetros integrados ao YAML e política
  documentada em `docs/MISSAO.md`. Quatro regressões novas cobrem esses limites.
- SIH 07 falhou na chegada de 40m por ultrapassagem do limite de seguimento de 1m.
  ULog indica resposta ao arrasto e erro de estimativa; sem saturação aparente.
- SIH 08 testou override de desaceleração 1m/s²: cruzeiro de 3m/s sustentado por
  4,95s, mas chegada ainda falhou. O padrão de 2m/s² do projeto foi preservado.
  Nenhum parâmetro/arquivo externo do PX4 ou Gazebo foi alterado.
- Harness agora registra métricas e exige cruzeiro sustentado e margem geométrica
  do obstáculo para aprovar o cenário completo. Rodadas 07/08 encerradas e processos
  próprios limpos. Detalhes e limites em `docs/VALIDACAO_SIH.md`.
- Validação integrada: **301 testes funcionais passaram, 1 ignorado**; build dos
  dois pacotes, lint fatal e `git diff --check` passaram. Passivo de estilo legado
  continua separado, conforme registro anterior.

Pendências:

Atualização de aproximação e seguimento:

- Aproximação terminal integrada: reserva de trecho lento, alvo fixo, limite que
  só diminui no segmento e piso positivo antes da parada final Ruckig. Parâmetros
  `arrival_approach_velocity=0.6` e `arrival_settling_time=2.0` no grupo navigation.
  Obstáculo com limite zero e critérios físicos de chegada continuam prevalecendo.
- Planta reduzida com arrasto/ganhos SIH reproduziu o aborto anterior (>1m); com
  aproximação, completou o movimento com erro máximo 0,870m. Não modela EKF completo.
- SIH 09 falhou antes da chegada por erro combinado longitudinal/lateral. Corrigido
  governador de avanço para descontar erro transversal da margem disponível.
- SIH 10 falhou na decolagem por pausa de executor de 631ms e telemetria expirada;
  não validou a navegação nova. Ensaios encerrados, sem processos próprios pendentes.
- **308 testes funcionais passaram, 1 ignorado**; build dos dois pacotes e lint fatal
  passaram. Detalhes em `docs/VALIDACAO_SIH.md`. Limite de seguimento de 1m preservado.

1. Resolver estabilização antes de novos desvios observada no SIH 11 e validar
   cruzeiro sustentado no primeiro trecho. A pausa do SIH 10 não se repetiu com
   instrumentação; mantê-la disponível para capturar a causa se voltar a ocorrer.
2. Validar em SIH a cobertura angular nova, giro em hover com ruído de velocidade
   e retorno da missão após cancelamento demorado.
3. Revisar alterações finais de ActionServer e validar LAND/RTL em SIH.
4. Concluir SIH: cruzeiro, chegada, desvio, cancelamento e pouso; auditar offsets
   dos sensores e comportamento de foco. Suítes com doubles já cobrem missão.
5. Repetir build dos dois projetos e suíte funcional conjunta ao integrar os
   últimos patches. Registrar
   passivo legado sem esconder as falhas existentes de flake8/pep257.
6. Validar em PX4/SITL isolado se os recursos locais permitirem; documentar
   resultados e limitações concretas. Revisar docs/arquivos/código sem uso.

## Ambiente de testes persistente (o /tmp anterior foi limpo)

- Raiz: `/home/keyslav/ros2_ws/src/.drone-v2-validation/`, com COLCON_IGNORE.
- Carregar `/opt/ros/jazzy/setup.bash` e `.drone-v2-validation/install/local_setup.bash`.
- PYTHONPATH inclui `.drone-v2-validation/python` (Ruckig) e o pacote fonte
  `/home/keyslav/ros2_ws/src/drone_inspetor`.
- `tools/validate_sitl.py` sempre inicia PX4 SIH próprio; sensores são sintéticos,
  controle/dinâmica são PX4 reais em simulação, sem hardware.
- Saídas persistentes `sitl-flight-04` (falha por tracking), `sitl-flight-05`
  (falha pelo limite de 200ms do temporizador) e `sitl-flight-06` (GOTO inicial
  aprovado; retorno retido no alinhamento de yaw).
- O install original de ros2_ws continua com interfaces v1 e não foi substituído.
- As alterações funcionais estão locais. Não houve push nem tag/release 2.0.0.
