# Dashboard: organização e responsividade

Atualização local de 24/09/2026. O comando permanece:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch drone_inspetor dashboard_launch.py
```

Se as bridges já estiverem em execução, acrescente `bridges:=false`.

## Organização

- Resumo superior de drone, PX4, velocidade e bateria. Dados expirados deixam
  de ser apresentados como atuais; detalhes continuam no Monitor (Ctrl+M).
- Câmera, CV, profundidade e mapa em cartões com ação **Ampliar**. As imagens
  preservam a proporção e se ajustam ao resize sem esperar outro frame.
- Radar e operação da missão no painel lateral. A árvore completa da máquina
  de estados fica recolhida, com acesso pelo botão **Máquina de estados**.
- Abaixo de 1100 px, a área lateral passa para baixo dos sensores; abaixo de
  720 px, os sensores usam uma coluna e o resumo usa duas. Há rolagem vertical
  quando necessária, preservando controles legíveis. O reflow não recria
  instrumentos, conexões Qt ou seleções.

## Radar

O radar usa QPainter nativo, substituindo o HTML/WebEngine anterior. O mapa
continua usando Leaflet/WebEngine. O círculo é calculado a partir do tamanho
disponível, com escalas de 3, 6 e 12 m, contagem de retornos, distância mais
próxima e distância inferior.

O referencial é FLU solidário ao drone: zero à frente, +90° à esquerda. Yaw
global não gira o radar. O contrato existente continua sendo um vetor plano
alternando distância em metros e ângulo em radianos.

Como LidarMSG não contém timestamp, a idade mostrada é a recepção na GUI.
Depois de 1,5 s sem atualização, os retornos deixam de ser desenhados. Vetor
vazio não certifica espaço livre. O radar não altera o controle de obstáculos.

## Comentários e manutenção

O [guia de leitura](GUIA_LEITURA_CODIGO.md) organiza os fluxos e pontos de entrada.
Foram revisados comentários de controle, navegação, missão, visão e interfaces,
incluindo correções de descrições obsoletas da decolagem e dos quadrantes LiDAR.
Campos e tipos das interfaces ROS foram preservados.

## Verificação

- 365 testes passaram, 1 ignorado; verificadores completos de estilo legado
  separados. Lint de erros estáticos e `git diff --check` passaram.
- Testes de geometria/resizing, frescura do resumo e radar, convenção FLU,
  seleção e comandos do painel de missão. Teste do reflow verifica que as
  instâncias dos instrumentos permanecem as mesmas.
- Prévia inspecionada em 1600×953, 1100×780 e 640×800, sem rolagem horizontal.
  Capturas usam imagem de referência e pontos sintéticos; não são ensaio de voo.
- DashboardNode instalado, backend e monitor iniciados em domínio ROS 178
  isolado. Dashboard/monitor visíveis, mapa `Loaded`, GUI encerrou com código 0.
  Sem comandos de voo ou nova simulação.
- Evidências locais: `.drone-v2-validation/dashboard-design/` no workspace.
