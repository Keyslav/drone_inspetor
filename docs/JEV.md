# Jev no copiloto do Drone Inspetor

Pesquisa e contrato conferidos em **03/10/2026**. **Jev, da TypeSafe AI**, é o
modelo System One mencionado para classificação e tomada de decisões. A
identificação foi confirmada pelo [anúncio oficial de 15/09/2026](https://typesafe.ai/blog/introducing-system-one-models-and-jev).
Isso resolve a incerteza anterior em torno do nome “Jet”.

## O que o modelo fornece

Jev recebe estado em texto/JSON e perguntas delimitadas. `Choice` escolhe uma
alternativa, `Score` avalia uma escala e `Noul` estima a probabilidade de uma
afirmação ser verdadeira. Sua saída contém valores estruturados; ele não gera
conversa, explicações ou código. A interface aceita somente **texto**, incluindo
objetos JSON: áudio precisa de transcrição e imagens precisam de percepção prévia.
Fontes: [System One](https://docs.typesafe.ai/concepts/system-one) e
[especificações de modelos](https://docs.typesafe.ai/models).

O acesso documentado é pela API hospedada da TypeSafe. As fontes oficiais
consultadas não apresentam pesos nem instruções para executar Jev no companion.
Nenhum peso Jev é instalado por este projeto.

## Adaptador implementado

`drone_inspetor/copilot/jev.py` fornece `JevProvider(api_key, model, post)` e
`propose(prompt, context)`. O transporte HTTP pode ser substituído por uma função
local nos testes. O provedor é independente de ROS e não publica comandos.

O contexto recebido contém `choices`, com pares `id`/`description`, e `telemetry`.
As opções são fornecidas pelo servidor, incluindo missões existentes como
`start:Flare`, `mission.cancel` e `camera.capture`. A opção `none` sempre existe
para pedidos informativos, ambíguos, múltiplos ou incompatíveis com o catálogo.
O nome de uma missão não se transforma em coordenadas geradas pelo modelo.

O adaptador usa o contrato oficial:

```text
POST https://api.typesafe.ai/v1/systemone
Authorization: Bearer <TYPESAFE_API_KEY>
model: jev-1.13.0
state: {request: texto, telemetry: snapshot}
questions.action: {type: choice, instructions: critérios, criteria: {id: descrição}}
```

A resposta usada é `answers.action.choice`, acompanhada de `confidence` quando
presente. O adaptador rejeita tipos inválidos, escolhas fora do catálogo e
confiança não finita ou fora de 0–1. A explicação devolvida à interface é texto
local que identifica a classificação; não é uma justificativa produzida por Jev.
Um catálogo sem ações devolve `none` sem chamar o serviço. Uma consulta tem timeout
de oito segundos; falhas são propagadas ao chamador, sem tentativa de executar
uma ação alternativa.

A versão padrão é fixada em `jev-1.13.0` para tornar comparações reproduzíveis.
`jev-latest` é um alias móvel. A API retorna o modelo que atendeu à consulta.
O provedor não aplica limiar de confiança como autorização: validação de estado,
revisão e execução pertencem ao supervisor e ao caminho de missões existente.
Fontes: [referência HTTP](https://docs.typesafe.ai/api),
[Choice](https://docs.typesafe.ai/primitives/choice) e
[quickstart](https://docs.typesafe.ai/introduction/quickstart).

## Limitações relevantes ao drone

Jev pode classificar incorretamente mesmo com confiança alta. A confiança de
`Choice` é uma medida calculada a partir das probabilidades; calibração é uma
propriedade estatística de conjuntos de previsões, não garantia de acerto individual.
Consulte [confiança](https://docs.typesafe.ai/confidence).

A TypeSafe documenta dificuldades com precisão numérica, contagem, comparações de
datas, contexto irrelevante, conteúdo adversarial e ordem das alternativas.
Essas limitações tornam inadequado delegar ao modelo cálculos de distância,
referenciais, altitude, bateria mínima, geofence ou sequência de setpoints.
Esses cálculos e limites continuam em código. Texto presente em logs ou na
transcrição não concede permissão para alterar regras do sistema. Fonte:
[limitações do Jev 1.13](https://docs.typesafe.ai/model-jaggedness/jev-1.13).

Inglês é o idioma de maior desempenho declarado. As instruções internas de
classificação são em inglês; os pedidos e descrições de missão podem permanecer
em português, mas precisam de avaliação específica. A documentação publica
US$ 0,042 por milhão de tokens de entrada e saída gratuita; preços e limites devem
ser conferidos antes de uma campanha de testes. Chave e requisições reais não são
necessárias para os testes locais. Fonte: [modelos](https://docs.typesafe.ai/models).

## Avaliação antes de uso em campo

O teste do adaptador comprova o contrato de transporte e rejeição de respostas
inválidas. Ele **não mede a qualidade do Jev**, não realiza chamadas pagas e não
comprova comportamento de voo. O próximo ensaio proposto é:

```text
casos anotados / replay de rosbag
  → snapshot compacto e cálculos determinísticos
  → classificação Jev
  → registro shadow, sem publicação de comandos PX4
  → comparação com resultado esperado e regras atuais
```

Preparar pedidos válidos e ambíguos em português, missão inexistente, pedido com
várias ações, instruções conflitantes, texto adversarial, telemetria ausente ou
expirada e falha de rede. Separar os casos usados para ajustar critérios daqueles
usados para medir o resultado final. Reordenar alternativas para detectar viés.
Medir erros por ação, propostas indevidas, frequência de `none`, latência e custo.
Registrar versão efetiva, critérios, estado enviado e distribuição completa no
harness de avaliação; o contrato compacto do provedor não é esse harness.

Depois desse replay, avaliar a integração em SITL com as mesmas proteções do
supervisor. A inferência remota não participa do ciclo rápido de controle nem
substitui failsafes, sensores ou autoridade da FSM. Não há neste documento alegação
de que esses ensaios com Jev real ou a integração em campo já tenham sido aprovados.

Teste local do adaptador, a partir da raiz do pacote:

```bash
python3 -m pytest -q test/test_copilot_jev.py
```

## Uso no projeto

O adaptador `copilot/jev.py` já está ligado ao painel móvel. Configuração,
comandos e avaliador de casos: [COPILOTO.md](COPILOTO.md). O modelo padrão é
`jev-1.13.0`, alterável por `DRONE_JEV_MODEL`; requer `TYPESAFE_API_KEY`.
Os 26 testes de contrato usam respostas sintéticas. Não houve consulta paga
nem avaliação da capacidade real do modelo neste projeto.
