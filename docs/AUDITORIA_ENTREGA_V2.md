# Auditoria de entrega — 23/09/2026

Comparação do plano original com código e evidências atuais. A implementação
e a validação funcional da entrega local estão concluídas, com os limites
explicitados abaixo. Os testes Gazebo usam a correção isolada
descrita em [REPRODUZIR_TESTES_GAZEBO.md](REPRODUZIR_TESTES_GAZEBO.md).

| Requisito | Evidência inspecionada | Situação |
| --- | --- | --- |
| Referências coerentes de posição, velocidade e aceleração | `trajectory.py` publica os três campos; perfil Ruckig; testes de navegação | Implementado/testado |
| Cruzeiro definido | Gazebo 10: 3 m/s ±10% por mais de 10 s em cada trecho | Comprovado nesse cenário |
| Redução perto de obstáculos | Gazebo 12: na amostra mais próxima, velocidade medida 0,857 m/s e referência 0,737 m/s, abaixo do cruzeiro 3 m/s | Comprovado nesse cenário |
| Desvio e retomada | Gazebo 12: cilindro físico, GOTO concluído, distância física centro–superfície 1,123 m | Comprovado nesse cenário |
| Frenagem/cancelamento | Testes de perfil/limites e cancelamento em movimento Gazebo 12, aguardando parada | Comprovado no escopo testado |
| LAND/RTL sem disputa com setpoints ROS | Testes de autoridade; Gazebo 09/12/13, RTL até desarme | Comprovado |
| Contratos reais ROS e callbacks atrasados | `test_mission_clients.py`, `test_drone_commands.py`, overlay v2 | Testes passaram |
| FSM e repositório tipado da missão | Testes FSM/modelos e Gazebo 14 com MissionNode/CVNode reais | Integração passou com imagem controlada; câmera ao vivo não avaliada |
| Inferência, registro de modelos e mídia separados | Componentes extraídos; testes CV/recorder, incluindo concorrência | Testes com substitutos + 10 pesos reais carregados/executados; precisão não avaliada |
| Monitor amigável e dados estruturados | Monitor Qt, adaptadores e testes de apresentação; evidência histórica no progresso | Implementado/testado |
| Empacotamento e versões | ament_python/ament_cmake, ambos 2.0.0; build de snapshot sem ignorados, imports fora do checkout | Instalação nova local aprovada; máquina sem dependências/CI remoto não comprovados |
| Documentação de coordenadas e física | `COORDENADAS.md`, `INIT_SIMULACAO.md`, diagnóstico magnético e receitas | Registrado; parâmetros físicos originais preservados |

## Fechamento das etapas

| Etapa do plano | Evidência da entrega |
| --- | --- |
| 0 — Base e contratos observáveis | README, versões/ambiente registrados, diagnóstico histórico preservado; testes com mensagens ROS geradas |
| 1 — Contratos e estado | `test_mission_clients.py`, `test_mission_fsm.py`, `test_drone_commands.py`: requests, reset, identidade de operação, rejeição e cancelamento |
| 2 — Controle e operações | CommandManager, autoridade PX4 e Trajectory; testes de concorrência/telemetria/perfil; Gazebo 10/12/13 comprovam cruzeiro, desvio, cancelamento e LAND/RTL |
| 3 — Missões/configuração | Modelos/repositório/sessão/configuração e clientes extraídos; testes de modelos/FSM/clientes; missão integrada Gazebo 14 |
| 4 — Visão/sensores/mídia | Registro/inferência/gravador e processamento de sensores separados; testes de concorrência/idade/dados inválidos; 10 pesos reais executados e atraso ROS medido |
| 5 — GUI | Imagens, seletor/detalhes e janela de análise extraídos; apresentação tipada, repositório de missões e adaptador JavaScript; testes GUI/mapa/monitor |
| 6 — Entrega | Build ament dos dois pacotes, launch por contexto, dependências/versões/documentação e workflow CI; estilo legado acompanhado separadamente conforme estratégia incremental |

Validação final em 23/09/2026: **339 testes passaram, 1 ignorado**, excluindo
somente os dois verificadores de estilo legado. `flake8 --select E9,F63,F7,F82`
e `git diff --check` passaram. Build final: **2 pacotes concluídos**.
Registros locais: `.drone-v2-validation/functional-delivery-final.txt` e
`.drone-v2-validation/build-delivery-final.txt`, relativos ao workspace.

O teste de reabertura da tela CV verifica seleção preservada e detalhes do
catálogo recebido antes de abrir a janela. Os testes da janela de análise
verificam atualização, limpeza e fechamento/reabertura. A extração final da
GUI não alterou navegação; os voos anteriores não foram repetidos por ela.

## Limites e reprodução

- Os 10 pesos foram validados quanto a carregamento/execução. A medição ROS
  encontrou 0,96–1,29 s com modelos grandes em CPU, imagem sintética e sem
  backlog. Precisão e proveniência dos datasets não foram comprovadas;
  detalhes em `VALIDACAO_VISAO.md`.
- A missão Gazebo 14 usou detecção real sobre imagem de referência repetida,
  vídeo decodificável, RTL e desarme físico. Não comprova percepção ao vivo.
  O último estado de missão coletado foi RETORNANDO; a verificação de término
  usou o desarme físico, antes do próximo ciclo da FSM.
- Build de snapshot limpo dos dois pacotes concluído em 14,5 s. Imports de
  DroneNode/MissionNode/CVNode a partir de `/tmp` apontaram para a instalação
  nova; docs/config/catalog foram encontrados no share. Evidências em
  `.drone-v2-validation/clean-package/{build.log,imports.json}`. Dependências
  existentes foram reutilizadas; ambiente vazio e CI remoto não executados.
- Os voos exigem o binário/configuração corrigidos, conforme
  `REPRODUZIR_TESTES_GAZEBO.md`. Os originais PX4/Gazebo foram preservados.
- Mapa local horizontal não observa teto; não há garantia para qualquer
  geometria. Reentrada em OFFBOARD em voo exige transferência explícita,
  ainda não implementada. A dívida de estilo legado permanece separada;
  não se declara aprovação de flake8/pep257 completos.
- Entrega em commits locais na branch `v2.0`, sem publicação remota.
