# Execução do Drone Inspetor v2

Guia de uso diário, revisado em **03/10/2026** conforme os launchs e o controlador
em `drone_inspetor/startup/`. Para instalar e compilar, comece pelo
[README principal](../README.md). Para modelos, mundo e comandos externos do
Gazebo/PX4 nesta máquina, use [INIT_SIMULACAO.md](../INIT_SIMULACAO.md).

## 1. Preparar o terminal

Em cada terminal usado para comandos ROS:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
# Se você compilou usando o venv descrito no README:
source .venv/bin/activate
source install/setup.bash
ros2 pkg prefix drone_inspetor
ros2 pkg executables drone_inspetor
```

Use o mesmo Python do build. A lista de executáveis deve conter
`drone_inspetor_start` e `drone_inspetor_start_gui`; caso contrário, recompile
`drone_inspetor` como descrito no README e carregue novamente `install/setup.bash`.

## 2. Escolher a interface de partida

```bash
# Tela de inicialização, em computador com ambiente gráfico
ros2 run drone_inspetor drone_inspetor_start_gui

# Menu no terminal, inclusive por SSH sem interface gráfica
ros2 run drone_inspetor drone_inspetor_start
```

Escolha um deles para iniciar a sessão. A tela de partida é independente do
dashboard de operação. Ela mostra o comando previsto, disponibilidade do ambiente,
logs e ações de início/parada. O menu oferece os mesmos perfis, consulta de estado,
logs recentes e parada. A tela permite escolher arquivos de parâmetros, missões e
bridges, além de habilitar cada nó. As escolhas ficam separadas por perfil enquanto
a janela está aberta; **Restaurar padrões deste perfil** limpa apenas o perfil
selecionado. Campos de arquivo vazios mantêm os padrões dos launchs.
Para personalizar a partida no terminal, use os comandos diretos abaixo; o menu
numérico usa os valores automáticos.

| Perfil | Nós iniciados | Bridges Gazebo–ROS | Relógio padrão |
| --- | --- | --- | --- |
| Projeto na simulação (`sim`) | câmera, CV, profundidade, lidar, drone, missão e dashboard | Automático | Simulado |
| Companion (`companion`) | câmera, CV, profundidade, lidar, drone e missão | Não | Real |
| Somente dashboard (`dashboard`) | dashboard | Não | Simulado se `/clock` estiver visível; real se não estiver |

O perfil companion é o backend completo da aplicação. Ele não instala/inicia
drivers específicos da câmera/LiDAR nem configura a conexão física com o PX4.
Se só parte desses nós for necessária, desmarque-os na tela ou use as opções
`--no-with-*` da CLI. Os perfis definem os padrões, e a seleção individual pode
alterá-los (inclusive habilitar o dashboard no companion).

## 3. Sequência de operação

### Simulação local

1. Inicie o servidor Gazebo com `Plataforma_UERJ`, PX4 4030 e MicroXRCEAgent,
   conforme [INIT_SIMULACAO.md](../INIT_SIMULACAO.md). Abra Gazebo GUI/QGroundControl
   conforme necessário.
2. Carregue o ambiente ROS e consulte `drone_inspetor_start status` pelo comando abaixo.
3. Na tela/menu, inicie **Projeto na simulação**. Se já iniciou as bridges à parte,
   selecione **Usar bridges já existentes** ou `--bridges off`.
4. Acompanhe dados e mensagens no dashboard/Monitor. A partida dos nós não inicia
   automaticamente uma missão nem altera o modo de voo.

Nenhum perfil inicia Gazebo, PX4, MicroXRCEAgent ou QGroundControl. Os indicadores
dos processos externos são uma detecção **local**: não encontrar PX4 na estação,
por exemplo, é esperado quando ele roda no autopiloto ou em outro computador.
A indicação de `/clock` identifica o tópico, não comprova que o relógio esteja avançando.

### Companion e estação em computadores distintos

No companion, inicie `companion`; na estação, inicie `dashboard --time real`.
Os nós precisam ser descobertos na mesma rede ROS, com `ROS_DOMAIN_ID` compatível
e conectividade DDS. Para acompanhar um backend simulado, use `--time sim` na estação.
O domínio é herdado do terminal; o iniciador não configura rede nem transportes.

O iniciador administra **uma sessão por usuário na máquina**. Para separar backend
e dashboard no mesmo computador, use launchs diretos (exemplo na seção 5), em vez
de tentar abrir dois perfis gerenciados ao mesmo tempo.

## 4. CLI para comandos diretos

```bash
ros2 run drone_inspetor drone_inspetor_start status
ros2 run drone_inspetor drone_inspetor_start status --json
ros2 run drone_inspetor drone_inspetor_start start sim --bridges auto
ros2 run drone_inspetor drone_inspetor_start start companion
ros2 run drone_inspetor drone_inspetor_start start dashboard --time sim
# Backend com parâmetros próprios, catálogo próprio e sem CV
ros2 run drone_inspetor drone_inspetor_start start companion \
  --params-file "/caminho/parametros.yaml" \
  --missions-file "/caminho/missoes.json" --no-with-cv
# Simulação usando outro mapeamento de tópicos das bridges
ros2 run drone_inspetor drone_inspetor_start start sim \
  --bridges-file "/caminho/ros_gz_bridges.yaml"
ros2 run drone_inspetor drone_inspetor_start stop
```

Cada `start` mantém o processo e os logs no terminal até o encerramento; os exemplos
são alternativas, não uma sequência a executar inteira no mesmo computador.

- `--bridges auto`: no perfil `sim`, inicia bridges se não detectar um processo
  local `parameter_bridge` ou `image_bridge`. Essa detecção não verifica se todos
  os tópicos necessários estão cobertos.
- `--bridges on`: solicita as duas bridges; a partida recusa bridges locais já detectadas.
- `--bridges off`: usa as bridges existentes. `on` só é aceito no perfil `sim`.
- `--time auto`: usa os padrões da tabela; `sim`/`real` definem explicitamente
  `use_sim_time`. Na tela, a escolha de relógio aparece para o dashboard isolado.
- `--params-file`, `--missions-file`, `--bridges-file`: arquivos existentes e
  legíveis, encaminhados ao launch como caminhos absolutos. Caminhos relativos
  usam o diretório atual. Um catálogo relativo não encontrado nele também é
  procurado em `share/drone_inspetor/missions` do pacote instalado. O arquivo de
  bridges só é utilizado quando as bridges são iniciadas.
- `--with-camera`/`--no-with-camera`, `--with-cv`/`--no-with-cv`, e as mesmas
  opções para `depth`, `lidar`, `drone`, `mission`, `dashboard`: substituem a
  seleção de cada nó. Opções omitidas mantêm o padrão do perfil.

O iniciador verifica existência e leitura dos arquivos antes de iniciar o
subprocesso; o preflight do launch continua validando seu conteúdo. Caminhos com
espaços são aceitos (use aspas no terminal). O comando é executado como uma lista
de argumentos, sem shell. A detecção de aplicação externa considera somente os
nós selecionados, para não bloquear componentes que podem coexistir.

Tempo simulado acompanha `/clock`; pode pausar, reiniciar ou avançar em outro ritmo.
Sem `/clock`, o relógio ROS simulado não avança. Prazos operacionais e idade de
recepção de telemetria usam relógio monotônico em vários componentes e continuam
contando durante uma pausa. Veja [relógios e diagnóstico](GUIA_LEITURA_CODIGO.md).

## 5. Launchs diretos e configuração avançada

Os iniciadores reutilizam estes arquivos, preservados por compatibilidade:

| Launch | Comportamento padrão |
| --- | --- |
| `dashboard_launch.py` | Todos os sete nós e bridges, com tempo simulado; não é só a interface |
| `drone_inspetor_launch.py` | Todos os sete nós, sem bridges, com tempo real; inclui a interface |
| `simulation_launch.py` | Somente bridges, com tempo simulado; não inicia Gazebo/PX4 |

A GUI e a CLI encaminham os arquivos e a seleção de nós aos mesmos launchs.
Os comandos diretos continuam disponíveis para automações e sessões independentes.

```bash
# Aplicação em simulação com bridges já abertas
ros2 launch drone_inspetor dashboard_launch.py bridges:=false

# Backend sem dashboard e sem CV, por exemplo
ros2 launch drone_inspetor drone_inspetor_launch.py \
  with_dashboard:=false with_cv:=false

# Só bridges de uma simulação já iniciada
ros2 launch drone_inspetor simulation_launch.py

# Configuração completa customizada (substitua os caminhos)
ros2 launch drone_inspetor drone_inspetor_launch.py \
  params_file:=/caminho/parametros.yaml missions_file:=/caminho/missoes.json
```

Para backend simulado e dashboard independentes no **mesmo computador**, em dois
terminais com o mesmo ambiente:

```bash
# Terminal 1: backend + bridges; use bridges:=false se elas já existirem
ros2 launch drone_inspetor dashboard_launch.py with_dashboard:=false

# Terminal 2: só dashboard, com configurações padrão dos launchs
ros2 launch drone_inspetor dashboard_launch.py bridges:=false \
  with_camera:=false with_cv:=false with_depth:=false \
  with_lidar:=false with_drone:=false with_mission:=false
```

| Argumento | Finalidade |
| --- | --- |
| `use_sim_time` | Seleciona relógio ROS real ou simulado |
| `bridges` | Habilita `ros_gz_bridge` e `ros_gz_image` |
| `bridges_file` | YAML de tópicos da bridge; padrão `share/drone_inspetor/config/ros_gz_bridges.yaml` |
| `with_camera`, `with_cv`, `with_depth`, `with_lidar` | Selecionam processadores de sensores |
| `with_drone`, `with_mission`, `with_dashboard` | Selecionam controle, missão e interface |
| `params_file` | YAML completo de parâmetros dos nós |
| `missions_file` | Catálogo absoluto ou relativo a `share/drone_inspetor/missions` |
| `log_level` | Nível de logs ROS; padrão `info` |

Para pesos externos, siga [MODELOS_CV.md](MODELOS_CV.md). O nó CV acessa os arquivos
na máquina onde ele roda, mesmo que o dashboard esteja em outra estação.

## 6. Encerramento e diagnóstico

Fechar a tela de partida encerra o launch iniciado **por ela**. Sair do menu encerra
o launch iniciado nesse menu. `Ctrl+C` encerra a execução CLI em primeiro plano;
`stop` pode encerrar uma sessão gerenciada iniciada por outra dessas interfaces.
Processos abertos manualmente são detectados, mas não encerrados pelo iniciador.

Fechar o dashboard encerra o launch que o contém. Portanto, no perfil `sim`, isso
também encerra o backend daquele launch. Use o exemplo de terminais separados para
manter o backend ativo ao fechar a interface. **Parar execução encerra nós**;
não equivale a solicitar cancelamento da missão, RTL ou pouso.

| Sintoma | Conferência |
| --- | --- |
| Executável do iniciador não encontrado | Recompilar `drone_inspetor`, recarregar `install/setup.bash` e conferir `ros2 pkg prefix` |
| Pacote não encontrado | Carregar o ambiente ROS e a instalação correta do workspace |
| Interfaces v1 ou `DashboardMissionCommandMSG` ausente | Recompilar os dois pacotes v2; ver [ocorrência e correção](CORRECAO_INICIALIZACAO_DASHBOARD.md) |
| Ruckig ausente | Usar o mesmo Python do build com `requirements-runtime.txt` instalado |
| Modo já ativo / aplicação externa detectada | Consultar `status`; encerrar a sessão anterior na interface que a iniciou |
| Dashboard sem dados | Conferir produtores, domínio DDS, transporte PX4, bridges e `/clock` quando simulado |
| Dashboard abre, mas RGB/CV/depth ficam vazios | Conferir `image_bridge` e os tópicos `/drone_inspetor/externo/*`; MicroXRCEAgent e a ligação Gazebo–PX4 não substituem as bridges de imagens ROS |

Os logs exibidos pelos iniciadores são recentes e limitados; eles não constituem
um arquivo permanente de missão. O diário `events.jsonl` e sua localização estão
em [DIAGNOSTICO_MISSAO_FLARE.md](DIAGNOSTICO_MISSAO_FLARE.md).
Para testes que iniciam sua própria simulação e executam manobras, use
[REPRODUZIR_TESTES_GAZEBO.md](REPRODUZIR_TESTES_GAZEBO.md).

## Diagnóstico de câmeras — 07/10/2026

Com `dashboard_launch.py bridges:=false`, a janela abriu, mas não havia processos
`parameter_bridge`/`image_bridge` nem tópicos de câmera no ROS. Gazebo, PX4 e
MicroXRCEAgent estavam ativos. O MicroXRCEAgent disponibiliza os tópicos `/fmu/*`;
para imagens e `/clock`, também são necessárias as bridges Gazebo–ROS do projeto.

Iniciar `ros2 launch drone_inspetor simulation_launch.py` restabeleceu as imagens.
A verificação passiva de oito segundos, com os nós câmera/CV/depth, recebeu
148 quadros RGB internos, 138 quadros CV e 84 imagens de profundidade processadas.
Não foi iniciada missão ou manobra; os nós de controle/mission/dashboard ficaram
desabilitados no ensaio. Nenhuma alteração no processamento de imagens foi necessária.

Escolha uma das formas de partida:

```bash
# Bridges pertencem ao mesmo launch do dashboard:
ros2 launch drone_inspetor dashboard_launch.py bridges:=true

# Ou mantenha bridges separadas, em outro terminal:
ros2 launch drone_inspetor simulation_launch.py
# Depois, no terminal do dashboard:
ros2 launch drone_inspetor dashboard_launch.py bridges:=false
```

Essas formas são alternativas; evite duplicar as bridges. O aviso
`Depth sem calibração` desabilita o scan de navegação derivado da câmera,
mas não impede a imagem de profundidade no dashboard. Não estime a calibração
só para eliminar esse aviso: a geometria precisa corresponder ao sensor/modelo.
