# Copiloto: Jev, LLM e comandos falados

Implementação local de **03/10/2026**. O painel **Copiloto de inspeção** está no
dashboard web e no Android, acessível pelo navegador também no computador.
A janela Qt antiga não contém esse painel.

## O que está disponível

| Entrada | Comportamento |
| --- | --- |
| Jev (TypeSafe AI) | Classifica texto + resumo de telemetria em uma escolha do catálogo, com confiança quando fornecida |
| LLM (OpenAI Responses) | Escolhe uma ação e fornece explicação; perguntas usam `none`, sem ação |
| MCP | Um cliente externo consulta o estado e cria uma proposta para revisão no painel |
| Falar, no Android | Reconhecimento de fala do dispositivo preenche o pedido; o operador revisa o texto antes de enviá-lo |
| Demonstração local | Regras simples para testar a interface; não é IA e suas propostas nunca são executáveis |

O catálogo contém iniciar uma missão existente, cancelar missão, fotografar,
ativar/parar gravação de câmera/CV e ativar/desativar análise de anomalias.
Não permite criar coordenadas, alterar PX4, armar, mudar OFFBOARD ou comandar
motores. Uma solicitação comporta uma ação. As FSMs existentes continuam
responsáveis pela missão e pelo voo.

## Demonstrar sem drone, chave ou custo de API

```bash
cd ~/ros2_ws/src/drone_inspetor
python3 -m drone_inspetor.mobile_gateway.server --demo --copilot
```

Abra `http://127.0.0.1:8765`, conecte com token `demo`, selecione **Demonstração
local** e digite `iniciar Flare` ou `tirar foto`. A proposta aparece, mas executar
permanece bloqueado. Sem `--copilot`, esse recurso fica desativado no gateway.

## Conectar os provedores

As credenciais ficam no ambiente do **computador que executa o gateway**, nunca
no APK. Configure somente o provedor desejado, sem colocar chaves no Git:

```bash
# Digitação oculta; execute no terminal que iniciará o gateway.
read -rsp 'Chave TypeSafe: ' TYPESAFE_API_KEY; echo
export TYPESAFE_API_KEY
export DRONE_JEV_MODEL=jev-1.13.0

# Alternativa LLM: modelo compatível com Responses e Structured Outputs.
read -rsp 'Chave OpenAI: ' OPENAI_API_KEY; echo
export OPENAI_API_KEY
export DRONE_LLM_MODEL='ID_DO_MODELO_ESCOLHIDO'
```

Não há modelo LLM pago escolhido automaticamente. Sem chave/modelo, a respectiva
opção aparece indisponível. Jev usa a API hospedada; não foram instalados pesos
locais. Identificação, contrato e limitações: [JEV.md](JEV.md).

Para observar telemetria ROS, siga primeiro a compilação e criação de token em
[ANDROID.md](ANDROID.md) e acrescente `--copilot` ao gateway:

```bash
ros2 run drone_inspetor mobile_gateway --host 0.0.0.0 \
  --token-file ~/.config/drone_inspetor/mobile.token --copilot
```

O padrão é **shadow**: registra propostas, sem executá-las. Para ensaiar revisão
e execução no simulador, reinicie acrescentando **ambas** as opções:

```text
--enable-commands --copilot-mode review
```

O operador deve selecionar a proposta, tocar **Revisar e executar** e confirmar.
Os argumentos vêm do registro no servidor, não da confirmação enviada pelo app.
A proposta expira em 60 segundos; mudança no estado operacional ou catálogo exige
nova proposta. Antes de publicar, o gateway aplica as validações de cada comando,
incluindo telemetria recente e estado da missão. Repetir o mesmo ID não publica
novamente; resultado `submitted` significa pedido enviado, não missão concluída.

## Dados, registros e limites

```text
Texto/voz → Jev ou LLM → escolha finita → proposta no servidor
MCP externo ─────────────────────────→ proposta no servidor
                                             ↓ revisão e confirmação
                                   validação atual → API → FSMs ROS
```

Os provedores diretos recebem o pedido, nomes/descrições das escolhas e resumo
selecionado de estados, bateria e proximidade. Não recebem GPS preciso, imagens,
logs completos ou tokens. O texto digitado continua sendo enviado ao provedor
escolhido. **MCP tem escopo distinto**: o cliente pode consultar telemetria e
catálogo com coordenadas; veja [MCP_DRONE.md](MCP_DRONE.md).

O botão **Falar** usa o reconhecedor instalado no Android, que pode processar áudio
em serviço externo. O app informa isso antes de abrir o reconhecedor. Receber a
transcrição não prepara nem confirma uma ação. Sem reconhecedor, use digitação.

Auditoria em `~/Drone_Inspetor_IA/events.jsonl`, alterável por
`--copilot-log-dir`. Registra pedido, contexto selecionado, provedor/modelo,
proposta/confiança, confirmação e resultado. Arquivo criado com permissão 0600;
não há rotação automática nesta versão. Solicitações podem conter informações
que o operador digitou. Propostas em memória não sobrevivem ao reinício.

Há limite de uma inferência simultânea, intervalo mínimo de 2 segundos e timeout
HTTP de 8 segundos; não há retentativa automática. A confiança do Jev não é
probabilidade garantida de uma missão segura e nunca autoriza execução.
Nenhum provedor participa do laço rápido de controle/setpoints.

## Comparar modelos sem executar comandos

O conjunto inicial tem oito casos em português: ações diretas, negação,
ambiguidade, múltiplas ações, pedido não suportado e tentativa de desviar regras.
Ele serve para começar a avaliação, não certifica um modelo.

```bash
cd ~/ros2_ws/src/drone_inspetor
python3 -m drone_inspetor.copilot.evaluate \
  --cases drone_inspetor/config/copilot_cases.json

# Só com --run haverá consultas ao provedor; no máximo 5 neste exemplo.
python3 -m drone_inspetor.copilot.evaluate \
  --cases drone_inspetor/config/copilot_cases.json \
  --provider jev --limit 5 --run --output /tmp/jev-avaliacao.jsonl
```

Troque por `--provider openai` para comparar a LLM configurada, ou `demo` para
regras locais. O resultado inclui escolha esperada/obtida, latência, modelo e
erros; não sobrescreve um arquivo existente. Esse avaliador não inicializa ROS.

Os contratos foram testados com respostas sintéticas, a interface foi exercitada
em navegador e o protocolo MCP real foi verificado em loopback. **Não houve
inferência paga, medição de qualidade do Jev/LLM, reconhecimento de voz em celular
ou voo por IA.** Próxima validação: avaliar os modelos com chaves próprias,
instalar o APK num dispositivo e executar missões aprovadas em SITL.
