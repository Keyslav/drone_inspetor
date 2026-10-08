# Inicialização real do dashboard — 23/09/2026

Relato de incidente e correção naquela data. Para preparar o workspace hoje,
use o [README](../README.md); para os novos iniciadores e perfis, use
[EXECUCAO.md](EXECUCAO.md). As evidências abaixo não foram reexecutadas nesta
revisão documental.

O usuário encontrou `ImportError: DashboardMissionCommandMSG` ao iniciar
`ros2 launch drone_inspetor dashboard_launch.py`. A entrega anterior havia sido
validada em overlay isolado e não atualizou a instalação normal. Foi uma lacuna
na validação/entrega: testes daquele overlay não comprovavam o launch habitual.

## Causas e correções

- `~/ros2_ws/install/drone_inspetor_msgs` ainda continha interfaces v1.
  Reconstruídos os dois pacotes v2 no workspace normal com `--symlink-install`.
- Ruckig ausente no Python habitual. Instalado `ruckig==0.19.4` no Python do
  usuário, sem substituir outras dependências. Novo preflight do launch detecta
  interfaces incompatíveis e Ruckig ausente antes de iniciar processos.
- O registro de modelos rejeitava arquivos existentes por resolver o symlink
  de `share` para fora desse diretório. Agora aceita recursos registrados
  instalados por symlink; continua rejeitando nomes arbitrários e categoria
  incorreta. Teste cobre symlinks válidos e quebrados.
- O JavaScript do mapa acessava `overlay-status`, ausente no HTML. Elemento
  restaurado; mapa e imagem da plataforma carregaram sem a exceção anterior.
- Receber o catálogo antes de abrir o seletor é normal; não gera mais alerta
  de dropdown ausente.

## Verificação executada

- Launch real abriu o dashboard no display do usuário, usando a instalação
  `~/ros2_ws/install`, sem PYTHONPATH do overlay de validação.
- Verificação visual com DashboardNode real, backend e bridges ROS/Gazebo:
  câmera da plataforma, inferência com pesos reais, imagem de profundidade,
  mapa e monitor recebendo telemetria PX4/drone. Nenhum comando de voo enviado.
- Dashboard e monitor visíveis; JavaScript confirmou mapa existente e estado
  do overlay `Loaded`. GUI encerrou com código 0.
- Suíte no ambiente normal: **344 passaram, 1 ignorado**. Os dois verificadores
  de estilo legado foram excluídos; checagem de erros estáticos passou.
- Evidências em `.drone-v2-validation/dashboard-startup/` no workspace:
  capturas, report.json, gui.log, backend.log e tests.log.

## Ocorrência externa no encerramento

Ao interromper as bridges criadas pelo teste, o executável instalado
`/opt/ros/jazzy/lib/ros_gz_image/image_bridge` lançou `RCLError` durante publicação
após shutdown e terminou com -6. Isso ocorreu ao encerrar, não durante abertura
ou recepção de imagens. Não foi corrigido no pacote externo. Os seis nós de
backend da aplicação e o dashboard terminaram normalmente nessa execução.
O teste encerrou seus processos e preservou Gazebo, PX4 e MicroXRCE existentes.
