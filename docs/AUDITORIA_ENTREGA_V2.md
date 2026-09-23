# Auditoria de entrega — 23/09/2026

Comparação do plano original com código e evidências atuais. Esta auditoria
não declara todo o plano concluído. Os testes Gazebo usam a correção isolada
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

## Pendências para o plano completo

1. Teste dos 10 pesos e medição ROS concluídos no escopo de compatibilidade: 0,96–1,29 s
   com modelos grandes em CPU, imagem sintética e sem backlog. Resultados em
   `VALIDACAO_VISAO.md`; precisão e proveniência dos datasets não comprovadas.
2. Missão curta integrada aprovada na rodada Gazebo 14: detecção real sobre
   imagem de referência repetida, vídeo decodificável, RTL e desarme físico.
   Não é validação de percepção ao vivo ou precisão de inspeção.
3. Build de snapshot limpo dos dois pacotes concluído em 14,5 s. Imports de
   DroneNode/MissionNode/CVNode executados a partir de `/tmp` apontaram somente
   para a instalação nova. Docs/config/catalog foram encontrados no share.
   Evidências: `.drone-v2-validation/clean-package/{build.log,imports.json}`.
   Dependências ROS/Python existentes foram reutilizadas; CI remoto não executado.
4. Revisão funcional e lint fatal passaram; commits locais organizados, sem
   publicação remota. A etapa 5 ainda tem extrações de organização pendentes:
   seletor/detalhes de modelos em CVScreen e interface curta para JavaScript
   do mapa. As extrações não são cobertas pelos ensaios de voo.

Limitações explícitas: mapa local horizontal não observa teto; não há garantia
para qualquer geometria; reentrada em OFFBOARD em voo exige transferência
explícita ainda não implementada. A dívida de estilo legado permanece reportada
separadamente e não deve ser apresentada como lint completo aprovado.
