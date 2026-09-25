# Catálogo, armazenamento e seleção de redes CV

## Interface

O botão **Redes CV** está no cabeçalho do painel de visão computacional.
Ele abre uma janela independente com abas **Equipamentos** e **Anomalias**,
filtro, detalhes e indicação dos modelos em uso. Selecionar um candidato não
troca a rede: **Aplicar seleção** envia o pedido, e o popup só confirma quando
o serviço do nó CV informa os nomes carregados. Fechar sem aplicar descarta
o rascunho. Um pedido já enviado continua sendo processado pelo nó.

**Ampliar** abre somente o vídeo CV, preservando o frame original e sua
proporção. No vídeo ampliado, botão direito ou **Ctrl+R** abre o seletor.
A janela de análise é independente e não impede a abertura do vídeo.

Disponibilidade, tamanho e pasta são informados pelo **computador do nó CV**,
não inferidos pelo dashboard. Pesos ausentes permanecem visíveis no catálogo,
mas não podem ser escolhidos. O catálogo usa o contrato ROS existente.

## Armazenamento recomendado

Mantenha os pesos grandes em uma pasta estável fora de `build`/`install`, com
`models.json` ao lado dos arquivos `.pt`. O parâmetro `models_directory` do
`cv_node` permite apontar para essa pasta. Vazio mantém o diretório instalado
`share/drone_inspetor/redes_treinadas`, preservando o funcionamento atual.

Copie o `config/param_ros.yaml` atual para `~/cv-models.yaml` e altere somente
`models_directory` na seção `cv_node`, preservando os demais parâmetros.
Trecho da configuração:

```yaml
cv_node:
  ros__parameters:
    models_directory: /home/keyslav/ModelosCV
```

Depois de preparar a pasta com catálogo e pesos:

```bash
ros2 launch drone_inspetor dashboard_launch.py params_file:=$HOME/cv-models.yaml
```

O trecho acima não é o arquivo completo: `params_file` substitui a configuração
geral, por isso a cópia deve manter os ajustes de navegação e percepção.

Organização esperada:

```text
ModelosCV/
  models.json
  flare_yolov8n_detection_300ep.pt
  corrosion_yolo8x_detection.pt
  ...
```

O catálogo já agrupa equipamentos e anomalias e guarda nomes, classes, dataset
e demais metadados. A identificação enviada ao CV continua sendo `file_name`.
Nomes devem ser únicos e sem caminhos relativos; links simbólicos para pesos
em outro disco são aceitos. Não é necessário duplicar os arquivos grandes.

As alterações desta entrega não moveram pesos nem reescreveram o catálogo.
Os metadados de disponibilidade/tamanho são calculados na consulta. Ao editar
`models.json`, reinicie o nó CV para recarregar suas entradas; **Atualizar
catálogo** consulta a lista já carregada pelo nó e a situação atual dos arquivos.
A seleção aplicada vale para o processo atual; ao reiniciar, permanece a regra
existente de carregar o primeiro modelo de cada categoria do catálogo.

## Verificação

368 testes passaram, 1 ignorado (estilo legado separado), incluindo rascunho,
fechamento sem aplicação, confirmação ROS, filtro, peso ausente, metadados e
pasta externa. Teste em domínio ROS isolado carregou o catálogo real e trocou
equipamentos para `flare_yolov8n_detection_300ep.pt`, mantendo a rede de anomalias.
O popup confirmou os modelos ativos; vídeo ampliado sem seletores. GUI encerrou
com código 0. Evidências em `.drone-v2-validation/cv-popup/` no workspace.
