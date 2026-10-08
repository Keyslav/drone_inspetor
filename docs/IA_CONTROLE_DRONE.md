# IA no controle do Drone Inspetor

Estado em **03/10/2026**. O nome esclarecido pelo usuário é **Jev**, modelo
System One da TypeSafe AI. A integração inicial está implementada por API,
junto de um adaptador LLM, ponte MCP e entrada de voz no Android. Consulte
[COPILOTO.md](COPILOTO.md) para executar e [JEV.md](JEV.md) para a pesquisa.

## Implementado e ainda experimental

O copiloto escolhe uma ação de catálogo, registra uma proposta e apresenta-a
para revisão. No modo padrão `shadow`, nenhuma proposta pode ser executada.
No modo `review`, com comandos habilitados, a confirmação passa pelas mesmas
validações determinísticas da API móvel e pelas FSMs existentes.

```text
Android / navegador → texto ou transcrição → Jev / LLM → proposta
Cliente MCP → estado + catálogo → proposta ───────────────┘
                                                         ↓
                                   revisão humana + validação atual
                                                         ↓
                                    MissionNode → DroneCommand → PX4
```

Isso permite experimentar “iniciar Flare”, “cancelar missão” ou “tirar foto”.
Jev classifica; a LLM também pode explicar o snapshot em texto. Não há geração
livre de trajetórias, alteração de coordenadas ou sequência de ações pelo modelo.
A confiança retornada não substitui as condições operacionais do controlador.

O SDK MCP e as ferramentas Android estão instalados localmente. Os provedores
Jev/OpenAI foram testados com respostas sintéticas; **não houve chamada aos
modelos reais**, pois as credenciais não estão configuradas. O APK foi compilado,
mas voz e operação em celular continuam pendentes. Esses resultados não
comprovam qualidade de decisão nem habilitam voo autônomo por IA.

## Responsabilidades no código

| Módulo | Responsabilidade |
| --- | --- |
| `copilot/jev.py` | Contrato de classificação da API TypeSafe |
| `copilot/providers.py` | LLM com saída estruturada e demonstração sem IA |
| `copilot/transport.py` | HTTP limitado, timeout e proteção de credenciais |
| `copilot/service.py` | Catálogo, contexto, propostas, validade, revisão e auditoria |
| `copilot/mcp_server.py` | Consultas e rascunhos por MCP; nenhuma ferramenta de execução |
| `copilot/evaluate.py` | Comparação de classificações em casos JSON, sem ROS |
| `mobile_web/copilot.js` | Pedido, propostas e confirmação no painel |
| `mobile/.../MainActivity.java` | Reconhecimento de fala pelo Android e entrega da transcrição |

`mobile_gateway/api.py` continua responsável por comandos aceitos, validações,
frescor da telemetria e deduplicação. O backend guarda os argumentos da proposta;
o navegador envia apenas seu ID na confirmação. A execução nunca depende de o
modelo inventar ou converter números de coordenadas.

O contrato existente permanece: GOTO usa AMSL, TAKEOFF usa altura relativa ao HOME
e navegação interna usa NED. Veja [COORDENADAS.md](COORDENADAS.md). A validação de
um catálogo não comprova geofence, trajeto livre ou energia disponível.

Inferência e rede não participam do laço de setpoints. O PX4 Offboard tem seus
próprios requisitos de continuidade e failsafe, que precisam ser conferidos no
firmware/configuração usados: [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard).
Falha da consulta de IA não gera um comando novo; uma missão já aceita segue
sob controle das FSMs e proteções existentes.

## Próximas avaliações

1. **Jev e LLM em shadow:** executar os casos de `copilot_cases.json` com chaves
   próprias; ampliar para estados reais anonimizados, português, negações,
   pedidos ambíguos e respostas fora de contexto. Medir acerto e latência;
   a documentação do fornecedor não substitui as medições deste drone.
2. **Android físico:** testar voz, rede, suspensão, reconexão e seleção/revisão.
3. **SITL:** testar missões aprovadas, expiração, perda de telemetria, IA lenta,
   comandos repetidos e cancelamento. Só então avaliar uso operacional.
4. **Diagnóstico por diários:** acrescentar leitura seletiva das sessões com
   respostas fundamentadas em eventos. A LLM atual recebe apenas um snapshot;
   não analisa automaticamente todo o histórico da Flare.
5. **Visão como pesquisa:** associar timestamp/imagem/pose e experimentar VLM para
   relatos de inspeção, mantendo revisão e sem inferir trajetória livre de texto.

Como pesquisa futura, VLA pode sugerir próximas vistas no simulador após coleta
de demonstrações. [Openpi, da Physical Intelligence](https://github.com/Physical-Intelligence/openpi)
é uma referência de ferramentas e modelos robóticos, não uma demonstração de que
os pesos disponíveis pilotem este drone. Ainda não há VLM/VLA integrado.

Os diários do MissionNode e a auditoria do copiloto têm finalidades diferentes:
o primeiro registra execução da missão; a segunda registra a intenção, contexto,
proposta e revisão. A relação entre causa confirmada e hipótese continua
necessária, como no [diagnóstico da Flare](DIAGNOSTICO_MISSAO_FLARE.md).
