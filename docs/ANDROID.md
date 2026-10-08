# Dashboard móvel e aplicativo Android

Implementação inicial em **03/10/2026**. A interface, o gateway ROS e o aplicativo
Android estão implementados. **APK debug gerado e assinatura verificada** em
`mobile/app/build/outputs/apk/debug/app-debug.apk`. SDK/Gradle foram instalados
em `mobile/.android-tools/`. Ainda não houve teste em celular. Não é uma substituição completa do QGroundControl.

## Arquitetura e funcionalidades

```text
Android (WebView) ou navegador
        ↓ HTTP/HTTPS autenticado, na rede local
mobile_gateway no computador/companion
        ↓ tópicos e serviços ROS 2 existentes
drone_node / mission_node / câmera / CV / sensores / telemetria PX4
```

O telefone não precisa instalar ROS. No Android, a interface é incluída no APK
e abre mesmo sem estação configurada ou servidor acessível. Somente os dados,
imagens e comandos dependem do gateway. No navegador comum, o gateway continua
servindo a mesma interface. Atualizar a interface Android requer gerar/instalar
novamente o APK; atualizar somente o servidor não troca os arquivos do app. Para conectar, ambos devem
estar em redes que se alcancem. Internet não é necessária; o mapa OSM é opcional
e baixa imagens externas somente quando marcado. Sem ele, posição e rota aparecem
sobre uma grade. Não há cache offline de mapas nesta versão.

| Recurso | Implementação |
| --- | --- |
| Estados | Drone, PX4, missão, bateria, posição NED, referência NED e sensores, com idade/frequência |
| Mapa | GPS e pontos do catálogo de missões; altitude GPS indicada em AMSL |
| Radar | Canvas redimensionável; frente para cima, ângulo positivo à esquerda, alcance visual 20 m |
| Imagens | RGB, CV e profundidade; WebRTC ou JPEG selecionável, ampliação só da imagem |
| Redes CV | Popup com catálogo do CVNode; carregamento solicitado e consulta posterior dos modelos ativos |
| Missões | Seleção, início e cancelamento com confirmação; resultado acompanha a FSM |
| Câmera | Foto e gravação manual no computador que executa o gateway |
| CV | Solicitar gravação e análise de anomalias, bloqueadas durante uma missão |
| Copiloto | Jev/LLM, propostas MCP e revisão de comandos; [configuração](COPILOTO.md) |
| Voz no Android | Botão Falar transcreve pelo serviço do dispositivo; o texto é revisado antes do envio |
| Diagnóstico | Eventos recentes e últimas 200 entradas de cada diário de missão |

Armar, desarmar, joystick virtual, mudar OFFBOARD, editar parâmetros PX4 e enviar
setpoints diretamente não são recursos deste app. Continue usando o QGC para
preparação de voo. O gateway não inicia Gazebo, PX4, MicroXRCEAgent ou os nós da
aplicação. Use o [iniciador](EXECUCAO.md) para a aplicação ROS.

## Layout do aplicativo

A interface usa a área disponível da tela, sem uma página longa para rolar.
As abas **Voo, Câmeras, Missão, Copiloto e Telemetria** alternam o conteúdo.
Seletores e páginas de detalhes permitem consultar listas ou mensagens extensas
sem aumentar a altura da tela. O teclado virtual e as escolhas nativas do Android
continuam seguindo o comportamento do sistema.

A disposição responde ao tamanho útil da janela, não a uma lista fechada de
modelos de celular: retrato/paisagem, 16:9, 21:9, 4:3 e telas dobráveis. O Fold7
aberto tem 2184 × 1968 pixels, próximo de 10:9, portanto é verificado separadamente
de 4:3. Referência: [relatório oficial Samsung, Galaxy Z Fold7](https://images.samsung.com/is/content/samsung/assets/global/ir/docs/2025_3Q_Interim_Report.pdf).
Pixels físicos não são pixels CSS: densidade, barras do sistema e preferências
de tamanho de fonte alteram a área útil que o aplicativo recebe.

Sem conexão, as abas continuam navegáveis, com mapas/câmeras/instrumentos vazios,
indicações de ausência de dados e controles de comando bloqueados. Isso não é o
modo demo: nenhum estado, GPS ou imagem de voo fictício é apresentado no offline.
**Estação** configura o endereço; **Conexão** recebe o token e conecta.

No Android, os recursos públicos HTML/CSS/JS/imagens são empacotados e atendidos
localmente sob a origem configurada. Rotas `/api/` continuam passando pela rede;
não há cópia local de respostas de comando ou de credenciais. Sem estação,
usa-se uma origem local reservada, sem consultas a um servidor externo.

## Testar a interface sem drone

No checkout, sem carregar ROS:

```bash
cd ~/ros2_ws/src/drone_inspetor
python3 -m drone_inspetor.mobile_gateway.server --demo
```

Abra `http://127.0.0.1:8765` e informe o token **demo**. Os estados são fictícios,
as imagens ficam ausentes e os comandos permanecem bloqueados mesmo se
`--enable-commands` for informado. Para demonstrar no celular, acrescente
`--host 0.0.0.0` e abra `http://IP_DO_COMPUTADOR:8765`. O localhost do celular não
é o computador; em emulador Android padrão, o host é normalmente `10.0.2.2`.

## Usar com ROS

Compile após atualizar o checkout; a entrada `mobile_gateway` é instalada pelo
`setup.py`, junto dos arquivos web:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select drone_inspetor_msgs drone_inspetor
source install/setup.bash
ros2 run drone_inspetor mobile_gateway \
  --create-token ~/.config/drone_inspetor/mobile.token
ros2 run drone_inspetor mobile_gateway --host 0.0.0.0 \
  --token-file ~/.config/drone_inspetor/mobile.token
```

O arquivo é criado com permissão `0600` e nunca sobrescrito. Leia seu conteúdo
localmente e digite no formulário do dashboard. O token não vai na URL nem é
persistido pelo aplicativo. A API exige autenticação também para imagens e logs.
O padrão permite **somente leitura**. Para operações, reinicie acrescentando
`--enable-commands`. Iniciar uma missão ainda exige confirmação e estados recentes
de drone, missão e PX4, além de `MissionNode` em `PRONTO`.

Use o mesmo `ROS_DOMAIN_ID`, ambiente de mensagens e remaps dos demais nós. Se
usar um catálogo personalizado no MissionNode, passe o **mesmo arquivo** com
`--missions-file /caminho/missions.json`. O gateway carrega o catálogo ao iniciar;
ele não descobre automaticamente o arquivo configurado no MissionNode. Alterar
o catálogo requer reiniciar ambos para que a prévia corresponda à execução.

O gateway mede expiração com relógio monotônico local; não depende de `/clock`.
Os produtores mantêm a configuração de tempo definida no iniciador. Idade dos
tópicos significa tempo desde a recepção, não certifica frescor na origem.

HTTP local não cifra o token ou as imagens. Use rede de confiança; para HTTPS,
informe `--cert-file certificado.pem --key-file chave.pem`, com certificado
confiável no Android. O app não ignora certificados inválidos. Não há serviço
de acesso público, descoberta automática ou conexão pela internet nesta entrega.

## Vídeo WebRTC e acesso pelo navegador — 04/10/2026

A página web já é servida pelo próprio gateway. Abra **`http://IP_DO_PC:8765`**
em um navegador ou configure esse mesmo endereço em **Estação** no Android.
Ambos usam o mesmo HTML/CSS/JavaScript e o mesmo token; os atalhos nativos de
estação e reconhecimento de voz só aparecem no APK. O navegador precisa alcançar
o gateway para carregar a página; o APK inclui a interface e abre também offline.

Na aba **Câmeras**, escolha RGB, Detecção CV ou Profundidade e a transmissão:

| Opção | Comportamento |
| --- | --- |
| Automático (padrão) | Tenta WebRTC; se indisponível, passa para JPEG e informa no painel/aviso |
| WebRTC | Exige conexão de vídeo; mostra o erro se falhar, sem trocar silenciosamente |
| JPEG · compatível | Mantém o método anterior, consultando a imagem HTTP cerca de 2 vezes/s |

Somente a câmera visível é transmitida. Ampliar reutiliza o mesmo stream; trocar
câmera, sair da aba ou suspender encerra a sessão anterior. A seleção vale durante
a abertura atual do painel. O modo automático permanece em JPEG após uma falha;
selecione WebRTC ou reabra a aba para tentar novamente. Telemetria e comandos
continuam na API HTTP: não passam pelo canal de vídeo.

### Preparar e iniciar com WebRTC

As dependências opcionais foram instaladas neste checkout em `.webrtc-venv/`.
Para reproduzir em outra máquina:

```bash
cd ~/ros2_ws/src/drone_inspetor
bash mobile/setup-webrtc.sh
```

Com ROS e os nós da aplicação já iniciados:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/drone_inspetor
# Gere o token somente se ainda não existir:
bash mobile/run-gateway.sh --create-token ~/.config/drone_inspetor/mobile.token
bash mobile/run-gateway.sh --host 0.0.0.0 \
  --token-file ~/.config/drone_inspetor/mobile.token
```

O script inclui `--webrtc`, prioriza o Python isolado e os fontes deste checkout,
mas preserva os caminhos das mensagens ROS. Isso evita misturar `pyOpenSSL` do
venv com `cryptography` antigo do apt quando o ambiente ROS antepõe
`/usr/lib/python3/dist-packages` ao `PYTHONPATH`. O gateway verifica DTLS ao iniciar.
Adicione `--enable-commands` e/ou `--copilot` somente quando quiser esses recursos;
o padrão segue somente leitura. Não execute dois gateways na mesma porta.

Sem WebRTC, o comando `ros2 run drone_inspetor mobile_gateway ...` da seção anterior
continua funcionando sem instalar `aiortc`. O painel em Automático recua para JPEG.

### Rede, custo e limites

A negociação SDP passa pela API autenticada (`GET /api/v1/video`,
`POST /api/v1/video/offer` e `/close`). O vídeo usa ICE/UDP + DTLS/SRTP diretamente
entre gateway e cliente. HTTP na porta 8765 sozinho não basta: o firewall e o Wi-Fi
precisam permitir UDP entre os dois aparelhos, sem isolamento de clientes.
Nesta versão não há STUN/TURN externos nem relay pela internet; em redes que
impedem conexão direta, escolha JPEG. WebRTC cifra a mídia, mas o token e os dados
HTTP só são cifrados quando o gateway usa HTTPS com certificado confiável.

A implementação usa aiortc/PyAV, negocia VP8/H.264 conforme o cliente e codifica
por software. Limita a até 4 sessões, 15 quadros/s e 1280 pixels em cada dimensão,
preservando a proporção. Não há aceleração NVIDIA nesta entrega. Cada espectador
exige codificação própria: medir CPU, temperatura, banda e atraso no companion
continua necessário. JPEG pode ser mais econômico em CPU quando poucos quadros
por segundo bastam. WebRTC é adequado para vídeo contínuo, mas não elimina atrasos
de captura, compressão ROS ou transporte.

Quadros sem recepção por mais de 3 segundos são ocultados e identificados como
sem imagem recente. A perda de vídeo não altera automaticamente missão ou modo de
voo. Sessões abandonadas são encerradas pelo watchdog/ICE; a fila contém somente
a imagem mais recente, sem acumular vídeo atrasado.

Referências: [conexões WebRTC](https://webrtc.org/getting-started/peer-connections),
[API aiortc](https://aiortc.readthedocs.io/en/latest/api.html).

## Resultados de comandos e arquivos

- `submitted`: pedido publicado no ROS; **não** comprova início de voo ou troca de modelo.
- `completed`: operação local/serviço confirmou o resultado.
- `rejected`: pedido recusado; a interface mostra a razão.
- `uncertain`: resposta não confirmada. Confira estados e logs antes de nova ação.

IDs de requisição e nonces de uso único impedem repetição acidental no gateway.
Nonces expiram em 10 segundos; não são persistidos entre reinícios. A página não
reenvia comandos quando volta da suspensão ou reconecta. O QoS dos comandos de
missão mudou para **RELIABLE + VOLATILE, depth=1** para evitar replay de mensagens
retidas. Reinicie também dashboard desktop e MissionNode após atualizar; misturar
instâncias antigas e novas pode impedir a comunicação devido ao QoS diferente.

Fotos/vídeos manuais ficam em `~/Drone_Inspetor_Mobile` (override `--media-dir`),
no host do gateway. Esse gravador é separado da gravação automática de missão;
pará-lo não encerra a gravação da missão. Frames vêm da câmera ROS existente.
O vídeo manual usa MJPG/AVI a até 15 amostras/s, com FPS declarado fixo; perdas
de frames podem encurtar o tempo reproduzido. Não substitui uma gravação temporal
de sensores. A transmissão pode usar WebRTC (até 15 quadros/s) ou JPEG (cerca de
2 consultas/s). A cadência efetiva depende da câmera, da rede e da CPU; ainda não
foi medida a latência com câmera real para uso em pilotagem.

Diários vêm de `~/Drone_Inspetor_Missoes` (override `--sessions-dir`). A API lista
até 50 sessões e lê até 256 KiB/200 eventos por consulta; não exporta logs completos
nem permite navegar por outros arquivos. Os eventos do próprio gateway ficam
em memória durante a execução; os diários continuam sob responsabilidade do
MissionNode. Recursos CV seguem os serviços existentes e podem ser recusados.

## Android e manutenção

Abra [o projeto Android](../mobile/README.md) para gerar/instalar o APK. A tela
nativa **Estação**, acessível pela barra do dashboard, salva só o endereço do
gateway. Ao trocar de estação ou reabrir o aplicativo, digite o token novamente.
Rotação usa o mesmo WebView; suspensão bloqueia controles até receber novo estado. Android mínimo: 8.0/API 26, com WebView atualizado.

Código dividido em:

- `mobile_gateway/state.py`: snapshots, expiração de imagens e JSON sem NaN/Inf.
- `mobile_gateway/api.py`: validações, idempotência, catálogo e diários.
- `mobile_gateway/ros_adapter.py`: assinaturas ROS, serviços e gravação manual.
- `mobile_gateway/server.py`: HTTP, autenticação, recursos web e modo demo.
- `mobile_gateway/webrtc.py`: sessões ICE/DTLS, codificação e expiração de vídeo.
- `mobile_web/video.js`: escolha de transporte, fallback e ciclo de vida da câmera.
- `mobile_web/`: apresentação, navegação e atualização dos painéis.
- `mobile/`: projeto Android sem dependência de ROS/Qt.

Validação do conjunto móvel/iniciador/copiloto: **111 testes Python/ROS e 9 JavaScript passaram**;
build colcon temporário e execução do `mobile_gateway --help` instalado conferidos.
Inclui testes unitários da API, HTTP real em loopback,
integração DDS no domínio isolado 187 com produtores sintéticos (incluindo JPEG,
serviço de catálogo e comando recebido uma única vez), regras JavaScript e
conferência visual da página no navegador. Não houve voo novo nem ensaio em
celular. O APK foi gerado posteriormente, com build e lint concluídos e assinatura
verificada; não havia dispositivo conectado. Ainda faltam instalação física,
mudanças de rede/suspensão Android, reconhecimento de voz,
latência com câmera real e missão completa pelo app em SITL antes do uso de campo.


## Verificação do layout — 04/10/2026

APK **2.0.0-preview2**, com atualização por cima do preview1 usando a mesma
assinatura de desenvolvimento. Interface testada no navegador com as seguintes
áreas úteis em pixels CSS, tanto desconectada como com dados de demonstração:

| Formato | Retrato | Paisagem |
| --- | --- | --- |
| 16:9 | 360 × 640 | 640 × 360 |
| 21:9 | 360 × 840 | 840 × 360 |
| 4:3 | 768 × 1024 | 1024 × 768 |
| Fold7 aberto, proporção 2184:1968 | 656 × 728 | 728 × 656 |

Foram 84 verificações de abas/subabas na matriz, mais consultas de câmera,
conexão, redes CV e eventos. Sem overflow de página, botões fora da tela ou
texto cortado nos leitores paginados. A página inicial não tenta autenticar
antes de o usuário tocar em Conectar e não abre um modal obrigatório.
Em paisagem baixa, o título redundante da aba Voo cede espaço aos instrumentos.

A validação inclui 44 testes Python dos contratos HTTP/API/copiloto, 11 testes
JavaScript (incluindo paginação Unicode e comandos), conferência dos arquivos
públicos empacotados e compilação Android com lint. Não foram executados novos
ensaios de voo. Rotações físicas, teclado do aparelho, recortes específicos,
abrir/fechar o Fold e escalas de acessibilidade ainda precisam de teste em celular.
A emulação de viewport não representa uma certificação do hardware.

## Verificação WebRTC — 04/10/2026

APK **2.0.0-preview3** recompilado com a mesma interface web e `video.js`.
`assembleDebug`, `lintDebug` (0 erros/2 avisos) e assinatura v2 aprovados.
Foram **45 testes Python** (42 API/HTTP/WebRTC, 1 DDS sintético e 2 de assets APK)
e **16 testes JavaScript** (incluindo 5 de ciclo de vida de vídeo).

A regressão de encerramento reproduz ofertas do navegador sem marcador final
ICE e troca de câmera antes da conexão. O encerramento drena verificações STUN
antes de fechar UDP; a repetição no navegador terminou sem avisos ou tarefas
pendentes. Esse ajuste depende das versões fixadas aiortc 1.15/aioice 0.10.2 e
deve ser revalidado antes de atualizar essas bibliotecas.

No navegador, WebRTC recebeu RGB, CV e profundidade sintéticos em 640 × 480;
a seleção JPEG também recebeu imagem. Interromper a fonte ocultou o vídeo após
a expiração, com aviso explícito. Expansão reutiliza o stream. As 16 verificações
de câmera/ampliação nas oito viewports da matriz acima passaram sem overflow.
O fallback automático foi observado durante a falha inicial de dependências
Python e também possui teste de lógica. Não houve voo ou teste físico Android.

Para reproduzir vídeo sem ROS/drone:

```bash
cd ~/ros2_ws/src/drone_inspetor
PYTHONPATH=. .webrtc-venv/bin/python test/manual_mobile_video.py
```

Abra `http://127.0.0.1:8766`, use token `demo` e a aba Câmeras. Os quadros são
identificados como **VIDEO SINTETICO**; comandos seguem desativados. Para testar
no telefone, acrescente `--host 0.0.0.0` e use o IP do computador.

```bash
PYTHONPATH=. .webrtc-venv/bin/python -m pytest -q \
  test/test_mobile_api.py test/test_mobile_http.py test/test_mobile_webrtc.py
node test/test_mobile_video.cjs
```

A conferência DDS usa o domínio isolado 187 e não mede FPS de captura. Ainda
faltam medições de atraso/banda/CPU na câmera e no companion reais, bem como
instalação e suspensão/rotação em aparelho Android físico.
