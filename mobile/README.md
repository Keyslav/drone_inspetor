# Drone Inspetor para Android — preview 3

Aplicativo Android Java para o [dashboard móvel](../docs/ANDROID.md). O APK inclui
os arquivos da interface: abre o painel mesmo sem estação configurada ou sem rede.
Os campos mostram a falta de conexão até o operador conectar a um gateway ROS.
Não embute ROS, pesos CV, tokens ou chaves de IA.

A interface ocupa a tela disponível, com navegação por abas e adaptação a retrato,
paisagem e telas dobráveis. Rotação e mudança do tamanho da janela mantêm o WebView
atual; o token continua somente na memória. Não existe mais uma barra nativa
ocupando espaço acima do dashboard. Os controles **Estação** e **Falar** ficam na
própria interface web; o app cuida dos recortes da tela, barras do sistema e teclado.

## APK disponível

Versão **2.0.0-preview3**, `versionCode 3`, gerada em **04/10/2026**:
`app/build/outputs/apk/debug/app-debug.apk`.

```bash
cd ~/ros2_ws/src/drone_inspetor/mobile
bash build-apk.sh
.android-tools/sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk
```

Instalados neste projeto, em `.android-tools/` (ignorado pelo Git): SDK Android,
API 35, build-tools 34.0.0, platform-tools e Gradle 8.7. O script usa essas
ferramentas quando presentes. Em outra máquina, configure `ANDROID_HOME` com SDK
compatível; o Gradle Wrapper incluído baixa a distribuição fixada e verifica
seu SHA256. Também é possível abrir a pasta no Android Studio.

Plugin Android **8.6.1**, Gradle **8.7**, JDK mínimo **17**, compileSdk **35**,
targetSdk **34**, minSdk **26**. Build local feito com JDK 21. Compatibilidade:
[tabela oficial](https://developer.android.com/build/releases/agp-8-6-0-release-notes).
O primeiro build requer acesso aos repositórios oficiais Google/Gradle/Maven.

`assembleDebug` e `lintDebug` concluíram; assinatura de desenvolvimento verificada
por `apksigner`. O lint tem dois avisos: atualização de targetSdk e dataExtractionRules. Não há assinatura release nem publicação na Play Store. O alvo atual
é teste local. Nenhum dispositivo estava conectado ao `adb`; o APK ainda não foi
instalado nem validado num celular.

## Conectar e falar

1. Abra o app: o dashboard aparece mesmo sem servidor, indicando **Desconectado**.
2. Inicie o gateway no computador/companion com acesso pela LAN.
3. Toque em **Estação** na interface e informe `http://IP_DO_COMPUTADOR:8765`.
4. Digite o token no diálogo de conexão; para o gateway `--demo`, use `demo`.
5. Com `--copilot` no gateway, **Falar** abre o reconhecedor de voz do Android.
6. Revise a transcrição na aba **Copiloto** e toque em **Preparar proposta**.

No diálogo de estação, **Sem estação** limpa o endereço salvo e reabre o dashboard
local. Voltar ao aplicativo após fechá-lo não restaura o token: reconecte com ele.

O app informa que o reconhecedor do dispositivo pode enviar áudio ao fornecedor.
Sem serviço de reconhecimento, é possível digitar. A transcrição nunca executa
comandos automaticamente. Modos Jev, LLM e MCP: [COPILOTO.md](../docs/COPILOTO.md).

O aplicativo guarda somente a origem, nunca o token. Não há ponte
`addJavascriptInterface`, leitura de arquivos ou abertura automática de páginas
externas. O WebView usa JavaScript e mantém validação TLS; HTTP é permitido para
LAN. A captura de fala é delegada a outro aplicativo por `RecognizerIntent`, sem
permissão própria de gravação de áudio. API: [WebSettings](https://developer.android.com/reference/android/webkit/WebSettings).

## Interface local e atualização

`app/build.gradle` copia `../drone_inspetor/mobile_web` (relativo à pasta `mobile`)
para `assets/dashboard/` durante o build e gera `assets/dashboard-assets.txt`, uma
lista explícita dos arquivos públicos incluídos. O WebView serve somente esses
caminhos via GET, sob a mesma origem do gateway; as rotas `/api/` continuam
consultando a rede. Sem estação, a origem local é
`https://appassets.androidplatform.net` e `/api/` retorna indisponibilidade local.
Não usa `file://`, service worker nem ponte JavaScript com acesso genérico ao Android.

Por isso, **alterar a interface web exige gerar e instalar um novo APK**. Atualizar
somente o servidor não troca a cópia de HTML/CSS/JavaScript instalada no celular.
Os únicos atalhos nativos são `drone-inspetor://station` e
`drone-inspetor://speech`, aceitos a partir de toque no frame principal local.

Ainda validar em dispositivo: instalação, retrato/paisagem, teclado, voltar,
suspensão, perda/retorno de rede e fala. Depois, câmera e missão completa em SITL.
O build e os testes de lógica não substituem esses ensaios.

## Vídeo e navegador

A aba **Câmeras** oferece **Automático**, **WebRTC** e **JPEG · compatível**.
O gateway WebRTC é iniciado por `bash mobile/run-gateway.sh` a partir da raiz
do repositório, depois de carregar o ambiente ROS. Dependências opcionais:
`bash mobile/setup-webrtc.sh`. Instruções de rede e token: [ANDROID.md](../docs/ANDROID.md#vídeo-webrtc-e-acesso-pelo-navegador--04102026).

No navegador, abra o mesmo endereço da estação (`http://IP_DO_PC:8765`).
O WebRTC somente recebe vídeo; não solicita câmera/microfone do celular.
A recepção usa autoplay com vídeo mudo, e a expansão compartilha o stream.
A mídia usa UDP direto na LAN; JPEG continua disponível se essa conexão falhar.
