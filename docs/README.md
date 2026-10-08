# Índice da documentação

Revisado em **03/10/2026**. Este índice indica a finalidade de cada Markdown
dos dois pacotes. A sequência para começar é **instalar → preparar a infraestrutura
de simulação, se usada → escolher um modo de execução**.

## Montagem e execução

| Documento | Para que existe / quando consultar |
| --- | --- |
| [README principal](../README.md) | Instalar dependências, preparar Python/ROS, compilar, atualizar os pacotes v2 e verificar o workspace |
| [EXECUCAO.md](EXECUCAO.md) | Operação diária: tela inicial, menu CLI, três perfis, launchs diretos, relógios, bridges, parada e diagnóstico |
| [INIT_SIMULACAO.md](../INIT_SIMULACAO.md) | Referência local do Gazebo/PX4: caminhos, mundo Plataforma_UERJ, modelo x500_uerj, autostart 4030, comandos externos e auditoria física |
| [README das interfaces](../../drone_inspetor_msgs/README.md) | Compilar `drone_inspetor_msgs`, entender os contratos ROS e a compatibilidade com a aplicação; esse pacote não executa nós |

## Uso e configuração dos componentes

| Documento | Para que existe |
| --- | --- |
| [DASHBOARD_V2.md](DASHBOARD_V2.md) | Apresentação dos painéis do dashboard, ampliação, radar e comportamento de redimensionamento |
| [MONITOR_DRONE.md](MONITOR_DRONE.md) | Acesso à janela de telemetria, fontes/tópicos, unidades, remaps e dados expirados |
| [MODELOS_CV.md](MODELOS_CV.md) | Armazenamento dos pesos, catálogo, pasta externa e seleção de redes no popup |
| [COORDENADAS.md](COORDENADAS.md) | Contrato de eixos, origem, altitude e conversões Gazebo/ROS/PX4 |
| [MISSAO.md](MISSAO.md) | Regras do retorno da missão, falhas e retentativas; não é um tutorial geral de partida |
| [ANDROID.md](ANDROID.md) | Dashboard Android/navegador, WebRTC/JPEG, conexão ROS, autenticação e testes |
| [README Android](../mobile/README.md) | Preparar SDK/Gradle, gerar APK e conectar o aplicativo à estação |
| [IA_CONTROLE_DRONE.md](IA_CONTROLE_DRONE.md) | Escopo atual de IA, arquitetura, critérios de avaliação e etapas futuras de VLM/VLA |

| [COPILOTO.md](COPILOTO.md) | Ativar Jev/LLM, voz, propostas e revisão; dados enviados, auditoria e avaliação sem voo |
| [JEV.md](JEV.md) | Identificação do Jev da TypeSafe, API, contrato, versões e limitações para este projeto |
| [MCP_DRONE.md](MCP_DRONE.md) | Instalar e conectar a ponte MCP stdio, com quatro ferramentas e revisão no painel |

## Manutenção do código

| Documento | Para que existe |
| --- | --- |
| [GUIA_LEITURA_CODIGO.md](GUIA_LEITURA_CODIGO.md) | Localizar o código responsável por inicialização, voo, missão, visão e interface |
| [CLAUDE.md da aplicação](../CLAUDE.md) | Orientações de arquitetura e manutenção para desenvolvedores/assistentes; remete aos guias de uso |
| [CLAUDE.md das interfaces](../../drone_inspetor_msgs/CLAUDE.md) | Orientações para alterar contratos e registrar novas interfaces na compilação |
| [README de profundidade](../drone_inspetor/nodes/depth_node/README.md) | Projeção métrica, calibração, parâmetros e contrato do scan de profundidade |
| [README de LiDAR](../drone_inspetor/nodes/lidar_node/README.md) | Processamento para apresentação, convenção angular e expiração das leituras |
| [README de mídia](../drone_inspetor/media/README.md) | Propriedade e sincronização dos recursos de gravação |

## Ensaios e histórico

Resultados abaixo valem para as datas, versões e cenários registrados. Contagens
antigas de testes e caminhos de evidência não representam uma nova validação do
checkout atual. Para comandos cotidianos, use `EXECUCAO.md`.

| Documento | Para que existe |
| --- | --- |
| [REPRODUZIR_TESTES_GAZEBO.md](REPRODUZIR_TESTES_GAZEBO.md) | Receita para executar ensaios automatizados, preparar a cópia corrigida do PX4 e gerar evidências; o script cria sua própria simulação |
| [VALIDACAO_GAZEBO.md](VALIDACAO_GAZEBO.md) | Diário dos ensaios com Plataforma_UERJ/x500_uerj, resultados, correção magnética e limites |
| [VALIDACAO_SIH.md](VALIDACAO_SIH.md) | Receita e histórico dos testes com dinâmica SIH e obstáculos sintéticos; não valida a física do x500_uerj |
| [VALIDACAO_VISAO.md](VALIDACAO_VISAO.md) | Carregamento e execução dos pesos reais, medições e inconsistências encontradas |
| [DIAGNOSTICO_MISSAO_FLARE.md](DIAGNOSTICO_MISSAO_FLARE.md) | Investigação da decolagem seguida de retorno e formato/localização do diário `events.jsonl` |
| [CORRECAO_INICIALIZACAO_DASHBOARD.md](CORRECAO_INICIALIZACAO_DASHBOARD.md) | Incidente de interfaces/build desatualizados e evidências da correção em 23/09 |
| [PLANO_MELHORIAS_V2.md](PLANO_MELHORIAS_V2.md) | Diagnóstico e plano originais de 20/09; problemas descritos podem já ter sido corrigidos |
| [PROGRESSO_V2.md](PROGRESSO_V2.md) | Continuidade do trabalho e entregas por etapa/data |
| [AUDITORIA_ENTREGA_V2.md](AUDITORIA_ENTREGA_V2.md) | Conferência da entrega de 23/09 contra o plano, com evidências e limitações daquele momento |
| [manual/drone_node.md](../manual/drone_node.md) | Manual histórico anterior à reorganização; consultar o guia de leitura para o código v2 atual |
| [README dos scripts legados](../drone_inspetor/scripts/legados/README.md) | Identifica scripts antigos preservados para consulta, fora da execução atual |

Os links entre `drone_inspetor` e `drone_inspetor_msgs` pressupõem os repositórios
irmãos em `~/ros2_ws/src`, conforme a montagem do README. Alguns documentos,
manuais e logs só existem no checkout de desenvolvimento; o diretório `install`
não substitui esse índice de fontes.
