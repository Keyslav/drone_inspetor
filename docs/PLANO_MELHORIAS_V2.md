# Plano de organização e legibilidade da versão 2.0

Análise em 20/09/2026. Base: `drone_inspetor` em `9d164b1b6bdd9f916d3bf31520994957f11b8d63` e `drone_inspetor_msgs` em `15dee716b1ccc069d127f6edfad5e8fe70cf732b`, ambos na branch `v2.0`. Este documento propõe trabalho futuro; nenhuma alteração funcional foi implementada nesta análise. As referências de linha correspondem a esses commits.

A recomendação é continuar a modularização existente, tornando explícitos os contratos, a propriedade do estado e os efeitos de cada operação. As primeiras entregas devem estabelecer testes e corrigir inconsistências comprovadas; em seguida, extrair responsabilidades dos arquivos maiores. Mudanças de comportamento e movimentações de arquivos devem ter commits distintos para facilitar revisão e diagnóstico de regressões.

## 1. Diagnóstico e evidências

Há uma base aproveitável: especificações centralizadas em `ros_interfaces`, máquinas de estados com objetos por estado, separação entre missão e controle de voo, `TargetStack`, `TrajectoryProfile` e comunicação da GUI por sinais. Essas decisões devem ser preservadas.

O problema principal é a separação incompleta das responsabilidades. Vários módulos recebem o nó ROS inteiro e acessam livremente seus atributos privados. Contextos também fazem leitura de arquivo, criam diretórios, cancelam actions e publicam mensagens. Assim, dividir o código em pastas ainda não produziu componentes que possam ser entendidos e testados isoladamente.

O pacote interno contém 125 arquivos Python e 16.420 linhas, incluindo comentários e linhas em branco. A concentração de responsabilidades aparece especialmente nestes arquivos:

| Arquivo, relativo a `drone_inspetor/` interno | Linhas | Responsabilidades a separar |
|---|---:|---|
| `nodes/cv_node/cv_node.py` | 1.033 | ROS, modelos, inferência, desenho, gravação, arquivos, serviços e estado de missão |
| `gui/cv_screen.py` | 844 | Imagem, seleção de modelos, detalhes, janela de análise e exportação |
| `gui/dashboard_gui.py` | 747 | Layout, ligações de sinais, leitura de missões, gerenciamento de telas e mapa |
| `gui/mapa.py` | 721 | Estado do mapa, integração JavaScript, recursos e janela expandida |
| `nodes/depth_node/depth_node.py` | 680 | Recepção ROS, estatística, obstáculos, renderização e serialização |
| `nodes/mission_node/mission_node.py` | 583 | Transporte ROS, acompanhamento de actions, serviços CV, saúde e coordenação |
| `gui/utils.py` | 508 | Widgets, imagens, matemática, formatação, logging e estilos |

Esses números indicam onde investigar. Reduzir linhas ou criar um arquivo para cada função não é, por si só, um critério de melhoria.

### Problemas comprovados que precisam de uma base de testes

**A1 — A missão usa campos inexistentes em dois serviços.** Em `nodes/mission_node/fsm/mission/states/executando_inspecionando_detectando.py:57`, são atribuídos `objeto_alvo`, `tipos_anomalia` e `timeout`. O contrato `drone_inspetor_msgs/srv/CVDetectionSRV.srv` define `object_name`, `anomaly_types` e `timeout_seconds`. Em `executando_inspecionando_escaneando.py:38`, `folder_path` também não existe em `RecordDetectionsSRV.Request`. As quatro atribuições foram reproduzidas nas classes ROS geradas e resultaram em `AttributeError`. Os fluxos de detecção e gravação precisam ser corrigidos antes de validar uma missão completa. A correção inicial deve usar o contrato existente; a pasta já chega ao CV pelo estado da missão, e qualquer mudança dessa associação merece uma decisão separada.

**A2 — O reset pode deixar dois estados diferentes para a mesma missão.** `nodes/mission_node/fsm/mission/context.py:132` altera `context.state`, mas não altera o estado ativo de `MissionFSM`. O timeout de telemetria em `mission_node.py:223` chama esse reset sem transicionar a máquina. Uma reprodução com as classes da aplicação mostrou `context.state == DESATIVADO` enquanto `machine.current_state_id == EXECUTANDO_INSPECIONANDO`. Centralizar transições na máquina e separar a limpeza dos dados da mudança de estado elimina essa ambiguidade.

**A3 — LAND/RTL conflitam com o tratamento da saída de OFFBOARD.** `nodes/drone_node/px4_commands.py:271` e `:292` delegam LAND/RTL aos modos automáticos do PX4. Entretanto, `fsm/drone/states/em_voo.py:40` retorna `OFFBOARD_DESATIVADO` para qualquer modo diferente de OFFBOARD; `nodes/mission_node/fsm/mission/machine.py:49` trata esse estado como motivo para resetar a missão. A transição foi reproduzida com telemetria simulada em memória, usando `AUTO_RTL`. A correção deve distinguir transferência intencional de controle de perda inesperada. A conclusão de pouso/retorno ainda precisa de validação integrada em SITL.

**A4 — Validação e execução de comandos discordam.** `nodes/drone_node/fsm/drone/context.py:111` aceita comandos ausentes de `VALID_COMMANDS`, enquanto `action_server.py:152` só despacha ARM, TAKEOFF, GOTO, LAND e RTL. Um comando inexistente retornou `(True, '')` na reprodução da validação. A action também anuncia STOP e DISARM em comentários. Definir o conjunto suportado em uma fonte explícita, validar valores e documentar o comportamento é mais importante que renomear esses métodos.

**A5 — Algumas configurações não controlam o comportamento anunciado.** `nodes/mission_node/mission_node.py:164` retorna um diretório fixo, ignorando `missions_directory`. `fsm/mission/context.py:87` fixa `missions.json`, apesar de existir `missions_file` no YAML. `nodes/drone_node/trajectory.py:49` fixa `dt=0.02`, enquanto o período do timer pode ser configurado. Carregar configurações uma vez, validá-las e passá-las aos componentes torna essas relações verificáveis.

### Riscos identificados por leitura, ainda sem teste de concorrência ou voo

**A6 — A propriedade das operações assíncronas não está explícita.** O servidor usa `ReentrantCallbackGroup` e `goal_callback` não reserva um único goal ativo; os callbacks compartilham trajetória, contextos e `_current_goal_handle`. No cliente de missão, `_action_result_callback` associa sucesso à chegada ao waypoint pelo estado corrente, sem identificar a operação/waypoint que originou a resposta. É necessário testar goals sobrepostos, cancelamento antes da aceitação e respostas atrasadas. A solução deve incluir um identificador interno de operação e uma política explícita de concorrência, sem depender apenas de flags booleanas.

**A7 — Gravação e processamento de imagem compartilham recursos entre callbacks.** Em `nodes/cv_node/cv_node.py:90`, imagem e serviços têm grupos distintos. `image_callback:454` escreve em `_video_writer`, enquanto `record_service_callback:752` pode liberá-lo. A seleção de modelos também altera objetos usados pela inferência. A exposição à concorrência está presente no código; corrupção de vídeo ou falha de inferência não foi reproduzida nesta análise. A extração de um gravador com propriedade clara do recurso e troca controlada de modelos deve ser acompanhada de testes.

**A8 — Tempo e disponibilidade precisam de políticas explícitas.** `cv_node.py:645` espera uma detecção usando relógio ROS e `time.sleep`, sem condição de shutdown no laço. Com tempo simulado pausado e sem detecção, o prazo ROS não avança. O serviço compartilha o grupo de callbacks com controles de gravação e anomalias. Separar tempo de missão, prazo operacional e data de arquivo; testar pausa/retomada do relógio, desligamento e ausência de frames. Também validar a entrada em OFFBOARD: atualmente o nó só publica `OffboardControlMode` quando o PX4 já informa esse modo (`drone_node.py:227`), portanto a inicialização depende do restante do sistema.

### Questões de organização e legibilidade

**A9 — Contextos e estados dependem de detalhes de infraestrutura.** `MissionFSMContext` carrega JSON, cria pastas, cancela actions e publica ROS. Os estados acessam `_cv_detection_client`, `_action_in_progress` e outros atributos internos do nó. `valida_missao`, em `context.py:103`, também carrega a missão e cria diretórios. Separar `validate`, `load` e `start` permitirá entender quais chamadas apenas verificam dados e quais produzem efeitos.

**A10 — Fronteiras de dados são pouco explícitas.** Waypoints e comandos circulam como dicionários; posições como listas/tuplas e múltiplas representações de yaw. `DroneStateMSG` usa posição Z positiva para cima e velocidade Z positiva para baixo; a aplicação faz conversões na montagem da telemetria. Isso deve ser documentado e encapsulado em conversores, mantendo inicialmente a compatibilidade das mensagens. Introduzir tipos pequenos com unidades/frame no nome reduz a necessidade de ler vários arquivos para interpretar um valor.

**A11 — A GUI faz transformações e carregamentos que poderiam ter uma única implementação.** `subscribers/dashboard_cv_subscriber.py:59` transforma detecções em dicionário e depois JSON; `gui/cv_screen.py:580` desfaz a serialização no mesmo processo. `gui/dashboard_gui.py:155` e `MissionFSMContext` carregam as mesmas missões por implementações distintas. `gui/utils.py` mistura widgets com cálculos, logging e estilos. Extrair um repositório de missões sem dependência ROS/Qt e modelos de apresentação tipados permite compartilhar regras, sem compartilhar estado mutável entre threads.

**A12 — Parte da modularização ainda é duplicação ou acoplamento de importação.** O LiDAR é classificado em `nodes/lidar_node/lidar_node.py` e novamente em `nodes/drone_node/obstacles/lidar_obstacles/lidar_obstacle.py`; antes de unificar, comparar limiares, cooldown e consumidores. `nodes/drone_node/__init__.py:18` e `nodes/mission_node/__init__.py:10` importam os nós completos, fazendo imports de componentes menores dependerem do ambiente ROS. Manter inicializadores mínimos e usar entry points explícitos facilita testes sem middleware.

**A13 — Documentação e ferramentas não representam a base atual.** `CLAUDE.md` descreve `common/state.py`, enums e mixins que já mudaram; afirma que não há testes, embora existam. `BaseStateMachine.tick` descreve `on_enter` e `on_step` em ciclos distintos, mas executa ambos no primeiro tick. Há importações repetidas em `gui/cv_screen.py:16` e atribuição duplicada em `cv_node.py:439`. Atualizar comentários para explicar contratos, unidades e motivos, removendo descrições óbvias da sintaxe e referências históricas já inválidas.

**A14 — Build e execução precisam ser reproduzíveis.** O pacote declara `ament_python`, mas mantém um CMake de instalação alternativo. `setup.py` copia alguns arquivos Python para `share` além da instalação normal. `px4_msgs`, `ros_gz_image` e dependências de inferência não estão integralmente declarados. Os pesos `.pt` e modelos de simulação são locais/ignorados pelo Git. `dashboard_launch.py:132` ativa somente bridges e drone; os demais componentes estão comentados. Ambos os pacotes ainda declaram versão `0.0.0`. Esses pontos dificultam distinguir configuração intencional de instalação incompleta.

## 2. Organização proposta

Manter inicialmente os dois pacotes ROS e os nomes públicos de nós, tópicos, serviços e actions. A maior parte da melhoria pode acontecer dentro dos módulos existentes. A árvore abaixo mostra os pontos de extração propostos; não exige criar todos de uma vez nem mover cada estado.

```text
drone_inspetor/
  nodes/
    drone_node/
      drone_node.py          # Montagem, callbacks ROS e timers
      command_manager.py    # Aceitação, execução, cancelamento e resultado
      px4_gateway.py        # Mensagens/comandos e confirmação do PX4
      telemetry_mapper.py   # Snapshot interno -> DroneStateMSG
      trajectory.py
      trajectory_profile.py
      target_stack.py
      fsm/                  # Preservar FSMs de voo e deslocamento
      obstacles/
    mission_node/
      mission_node.py       # Montagem e adaptação ROS
      action_client.py      # Ciclo de vida de cada operação de voo
      cv_client.py          # Construção de requests e tratamento de respostas
      fsm/                  # Decisões e transições da missão
    cv_node/
      cv_node.py            # Adaptação ROS e coordenação
      model_registry.py     # Catálogo, validação e carregamento de modelos
      inference.py          # Detecção de equipamentos e anomalias
      rendering.py          # Anotações sobre imagens
    depth_node/
      depth_node.py
      processing.py         # Estatísticas e classificação de profundidade
      rendering.py
    dashboard_node/
      dashboard_node.py
      bridges/              # Adaptadores ROS <-> sinais por funcionalidade
  missions/
    models.py               # MissionDefinition, Waypoint, InspectionTarget
    repository.py           # Ler e validar missões, sem criar sessões
    missions.json
  media/
    video_recorder.py       # Compartilhável por câmera e CV
    photo_writer.py
  gui/
    dashboard_gui.py        # Composição das telas
    widgets/                # Visualizador de imagem, janelas, seletor de modelos
    presentation/           # Dados e formatação das telas
    theme.py
    map_bridge.py           # Contrato Python <-> JavaScript
  ros_interfaces/           # Preservar specs; adicionar conversores onde necessário
  base_classes/            # Base pequena das FSMs
  common/                  # Somente elementos realmente compartilhados
  config/
  launch/
```

Direção desejada das dependências: GUI e adaptadores ROS usam modelos e regras; modelos, cálculos e decisões não importam GUI nem ROS. Dentro do controle de voo, fornecer aos componentes dados e operações específicas, em vez do nó inteiro. Interfaces Python pequenas podem descrever essas operações; não é necessário introduzir um framework de injeção de dependências.

Os novos nomes são exemplos de responsabilidade, não autorização para renomear toda a API. Preservar termos de domínio já usados em português; usar o vocabulário das APIs nas bordas ROS/PX4/Qt. A consistência deve ser local e documentada, evitando uma tradução global que aumentaria o diff sem melhorar o comportamento.

## 3. Plano por entregas

| Etapa | Entrega | Dependência | Critério de aceitação |
|---|---|---|---|
| 0 | Registrar a base e alinhar o ambiente | Nenhuma | Dois commits identificados, build isolado documentado, imports das interfaces da v2 e resultado dos testes registrados |
| 1 | Corrigir contratos de serviços e estado da missão | 0 | Detecção/gravação constroem requests válidos; reset e perda de telemetria deixam estado publicado e executado iguais |
| 2 | Tornar explícito o ciclo de comandos de voo | 1 | Política de goal ativo, cancelamento e respostas atrasadas testada; LAND/RTL intencionais distinguíveis de perda de controle; comandos desconhecidos rejeitados |
| 3 | Extrair modelos, configuração e clientes da missão | 1–2 | JSON validado antes de iniciar missão; estados não constroem mensagens ROS nem acessam clientes privados; caminhos e prazos configurados são respeitados |
| 4 | Separar inferência, gravação e processamento de sensores | 1 e contratos estabilizados | Algoritmos testáveis com imagens/dados fixos; start/stop/shutdown da gravação sem disputa de recurso; política de frames e troca de modelos explícita |
| 5 | Simplificar GUI e adaptadores do dashboard | 3; estabilização das saídas de 4 | Detecções atravessam sinais como dados estruturados; layout separado de regras; widgets reutilizados; abertura/fechamento e modo expandido verificados |
| 6 | Consolidar documentação, empacotamento e qualidade automática | Começa em 0 e acompanha as demais | Manifestos completos, launch por contexto, estilo acordado, pipeline reproduzível e documentação fiel ao código |

### Etapa 0 — Base de comparação

- Documentar instalação testada: ROS 2 Jazzy, Python 3.12.3 e commits dos dois repositórios; registrar também PX4/`px4_msgs`, Gazebo, bibliotecas de visão e artefatos dos modelos antes de declarar uma matriz suportada.
- Escrever um README com build, composição dos nós, formas de execução, testes e procedimento para trocar a versão dos dois pacotes juntos.
- Registrar os resultados atuais, incluindo falhas já existentes. Os testes de matemática/conversão atuais passam, mas não constituem teste de missão.
- Definir a lista de comportamentos observáveis que cada refatoração preservará: entradas/saídas, nomes de canais, cancelamento, frames, unidades e artefatos gravados.

### Etapa 1 — Contratos e estado coerentes

- Corrigir A1 usando fábricas/conversores de requests pequenos e testados contra as classes realmente geradas de `drone_inspetor_msgs`.
- Tornar a máquina de estados a autoridade sobre o estado ativo. O contexto passa a guardar dados da missão; sua limpeza não altera o estado por conta própria.
- Definir comportamento de `on_enter`, `on_step`, `on_exit`, transição para o mesmo estado e reset. Cobrir com testes de sequência de eventos; não mudar silenciosamente a ordem atual.
- Registrar regressões para telemetria expirada, cancelamento e resultado que chega após reset. Cada correção deve ter uma reprodução que falhe antes e passe depois.

### Etapa 2 — Controle de voo e operações assíncronas

- Extrair o acompanhamento do goal para `CommandManager`, com reserva atômica da operação, identificador e resultado final explícitos. Como padrão inicial, manter um goal ativo e rejeitar outro comando comum enquanto ele executa; definir separadamente a prioridade de cancelamento, LAND, RTL e emergência.
- Tratar callbacks antigos pelo identificador da operação, incluindo solicitação ainda aguardando aceitação. Evitar que um resultado antigo libere a flag de uma operação nova ou marque outro waypoint como alcançado.
- Modelar a transferência intencional para AUTO_LAND/AUTO_RTL e os eventos de pouso/desarme. Evitar classificar a saída esperada de OFFBOARD como falha genérica.
- Fazer o validador de comandos e o dispatcher compartilharem o mesmo conjunto suportado. Resolver explicitamente a situação de STOP e DISARM na documentação/contrato.
- Encapsular o acesso ao PX4 e a conversão de telemetria; manter o cálculo de trajetória com entradas explícitas. Usar o período configurado de integração e definir o tratamento de pausas/saltos no relógio.
- Validar a entrada em OFFBOARD, decolagem, GOTO com foco, LAND e RTL em SITL após os testes sem hardware.

### Etapa 3 — Missões legíveis e configuração previsível

- Criar `MissionDefinition`, `Waypoint` e `InspectionTarget` como estruturas tipadas pequenas. Validar coordenadas, valores finitos, altitude, duração, foco e nomes/campos obrigatórios no carregamento.
- Extrair `MissionRepository` para leitura/validação. Criar diretório e arquivos apenas no início de uma sessão, deixando `validate` sem efeitos colaterais.
- Extrair `DroneActionClient` e `CVClient` com operações claras, como `navigate_to`, `request_detection`, `start_recording` e `stop_recording`. Inicialmente podem usar os mesmos serviços/actions existentes.
- Fazer os estados consumirem resultados da operação atual, sem construir mensagens ROS ou depender de atributos privados do nó.
- Criar configurações tipadas por componente, carregadas uma vez. Passar relógio e caminhos como dependências quando necessário para teste.
- Distinguir prazo de uma operação de duração da missão e tempo de calendário; documentar o efeito de `use_sim_time` em cada um.

### Etapa 4 — Visão, sensores e mídia

- Extrair registro de modelos, inferência e desenho de overlays do `CVNode`. A função de inferência recebe frame/configuração e devolve detecções estruturadas; gravação e publicação ficam fora dela.
- Definir um proprietário do `VideoWriter`, protegendo escrita, encerramento e substituição. Compartilhar o componente entre câmera e CV somente onde formato, timestamps e encerramento tiverem a mesma semântica.
- Definir troca de modelos entre frames e erro de carregamento preservando uma configuração consistente. Testes de coordenação usam modelos substitutos; um teste separado valida os pesos reais.
- Separar estatísticas/obstáculos de renderização no `DepthNode`. Comparar as duas classificações LiDAR antes de extrair o que for comum; preservar limiares e cooldown por configuração.
- Definir política de fila/idade de frame e medir atraso de ponta a ponta. Mudanças de profundidade QoS ou compressão precisam de evidência; não aplicá-las como limpeza estética.
- Corrigir desligamento e cancelamento de esperas longas. Avaliar uma action para detecção prolongada somente se cancelamento/progresso exigirem mudança no contrato; essa migração teria etapa e compatibilidade próprias.

### Etapa 5 — GUI e apresentação

- Extrair seletor de modelos, painel de detalhes, visualizador de imagem e janela de análise do `CVScreen` conforme suas responsabilidades.
- Separar `gui/utils.py` em widgets, conversão de imagens, formatação e tema. Manter matemática de domínio em módulos sem Qt; evitar criar outro utilitário genérico.
- Passar detecções estruturadas pelos sinais, evitando serialização JSON dentro do mesmo processo. Garantir que o produtor não modifique dados depois de entregá-los a outra thread.
- Reutilizar o repositório de missões e isolar os comandos JavaScript do mapa em uma interface curta.
- Agrupar adaptadores de publicação/assinatura por funcionalidade quando isso reduzir navegação. Preservar as regras de thread e manter a composição dos adaptadores visível no dashboard.
- Auditar botões/tópicos legados sem consumidor no código atual. Identificar possíveis consumidores externos antes de remover contratos públicos.

### Etapa 6 — Legibilidade, documentação e entrega

- Manter inicialmente `ament_flake8` e `ament_pep257`, que já existem. Acordar aspas, largura de linha, imports e docstrings antes da limpeza em massa; tratar formatação em commits próprios, por área, e acompanhar a redução do passivo.
- Usar comentários para explicar frame, unidade, motivo de transição, tolerância, política de erro e concorrência. Resumir cabeçalhos decorativos e descrições que apenas repetem o código; manter a documentação de estados junto à implementação correspondente.
- Simplificar os `__init__.py` e tornar os entry points explícitos. Código puro deve poder ser importado sem carregar nó, GUI ou modelo neural.
- Consolidar o build `ament_python` da aplicação e o `ament_cmake` das interfaces; revisar/remover o CMake alternativo após confirmar que não atende nenhum fluxo externo. Instalar dados em `share` e Python como pacote, evitando cópias redundantes.
- Declarar dependências efetivamente usadas e registrar a origem/versão dos pesos e recursos de simulação. Validar instalação limpa; sucesso no computador atual não comprova reprodução a partir de um clone.
- Criar launch files/argumentos para simulação completa, controle isolado, processamento embarcado e estação. Parametrizar componentes e `use_sim_time`, eliminando a necessidade de editar comentários para escolher o modo.
- Definir versões coerentes em `package.xml`/`setup.py` quando a entrega estiver estabilizada. Registrar mudanças de interfaces e a combinação compatível dos dois repositórios.
- Adicionar pipeline com build das interfaces, build da aplicação, testes de contrato/domínio e lint. Testes com GPU, GUI ou SITL ficam identificados e executados no ambiente apropriado.

## 4. Convenções práticas de legibilidade

1. Uma função deve expressar uma operação identificável. Separar validação, transformação de dados e efeitos de I/O; funções de validação não iniciam missões nem criam arquivos.
2. Contextos guardam dados; máquinas conduzem transições; adaptadores tratam ROS/PX4/Qt; gravadores possuem seus recursos. Especificar quem pode alterar cada estado compartilhado.
3. Usar `dataclass`, enums e anotações onde esclarecem contratos entre módulos. Substituir dicionários abertos gradualmente, mantendo conversores compatíveis nas bordas.
4. Explicitar unidades e referenciais: `*_m`, `*_s`, `*_rad`, `*_deg`, posição NED e altitude positiva para cima. Não mudar o significado de campos ROS existentes durante uma simples extração.
5. Usar uma nomenclatura estável por área. Evitar aliases opacos e nomes como `data`, `ctx` e `utils` nas interfaces públicas quando ocultarem o significado.
6. Tratar erros previstos perto de sua origem. Nas bordas, registrar contexto e traceback para falhas inesperadas; não silenciar indiscriminadamente exceções como se fossem shutdown normal.
7. Preferir dependências explícitas e composição. Extrair apenas responsabilidades concretas e testáveis, mantendo os estados pequenos que já são claros.
8. Revisar a legibilidade pela facilidade de responder: quais dados entram, qual resultado sai, que estado muda e quais falhas são possíveis.

## 5. Verificação necessária por área

| Área | Casos prioritários |
|---|---|
| Contratos ROS | Criar requests reais, campos válidos, defaults/NaN, payload de foco e rejeição de comando desconhecido |
| FSM base e missão | Ordem enter/step/exit, reset, perda de telemetria, cancelamento, resultado atrasado, falha/timeout de CV, conclusão de waypoint |
| Commands | Dois goals próximos, cancelamento antes/depois da aceitação, timeout, emergência e transferência intencional para PX4 |
| Navegação | Pilha com desvios encadeados, chegada, parada, foco, wrap de yaw, coordenadas/altitudes, período de integração e pausas |
| Sensores | Dados vazios, NaN/Inf, limites dos setores, cooldown, idade dos dados e combinação LiDAR/depth |
| Mídia/CV | Iniciar/parar enquanto chega frame, falha no arquivo, shutdown, troca de modelo e resultado de inferência atrasado |
| GUI | Transformação dos dados, seleção de modelos, atualização de missão, janela expandida e encerramento |
| Integração | Launch completo em SITL, missão curta, cancelamento, pouso/retorno e arquivos gerados |

O objetivo inicial é cobrir as transições e contratos críticos. Cobertura percentual global e limites arbitrários de tamanho de arquivo não substituem esses cenários. Os testes existentes de yaw que aceitam resultados com sinais opostos também merecem revisão do comportamento esperado antes de serem usados como garantia da navegação.

## 6. Resultado da análise executada

- Build isolado dos dois pacotes concluído em ROS 2 Jazzy/Python 3.12.3, com os demais pré-requisitos disponíveis neste computador. Não foi uma instalação a partir de ambiente vazio.
- Suíte configurada da aplicação: **54 casos; 51 passaram, 2 falharam, 1 foi ignorado**. As falhas são `test_flake8` e `test_pep257`; o teste ignorado é copyright.
- `test_flake8` reportou 4.388 ocorrências. A maioria está em aspas, espaços e linhas longas; há também imports repetidos/não usados. Esse número não significa 4.388 bugs funcionais.
- Os testes funcionais atuais concentram-se em matemática e conversão de mensagens. `test_state.py` está vazio. A execução do pacote de interfaces não acrescentou casos de teste nesse build.
- Verificação estática direcionada das atribuições a mensagens/requests identificou os quatro campos inválidos de A1. Esse verificador simples não é prova de compatibilidade de todas as operações.
- Reproduções sem iniciar nós confirmaram os `AttributeError`, a divergência de estado após reset, a transição para OFFBOARD_DESATIVADO em AUTO_RTL e a aceitação de comando desconhecido pelo validador.
- A instalação original do workspace ainda expõe interfaces da v1: importar `mission_node` encontrou ausência de `DashboardMissionCommandMSG`. O build temporário da v2 resolveu esse requisito para a análise. A instalação original não foi substituída.
- Não foram iniciados GUI, nós ROS de operação, simulador ou voo. Riscos de concorrência e efeitos completos de navegação exigem os testes propostos.

Artefatos temporários desta verificação: `/tmp/drone-inspetor-v2-audit.ZgeyZC/`. O resultado JUnit está em `build/drone_inspetor/pytest.xml`; os logs estão em `log/` e `test-log/`. Esses arquivos são temporários e não são requisito do projeto.

A primeira entrega recomendada é a etapa 1, precedida do registro da base da etapa 0: contratos corretos e um único estado de missão. Ela cria a sustentação para as extrações seguintes e oferece uma melhora verificável sem uma reorganização extensa de pastas.
