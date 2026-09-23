# Validação dos pesos reais — 23/09/2026

Os 10 pesos locais registrados em `redes_treinadas/models.json` carregaram e
executaram três inferências cada: PyTorch 2.8.0+cu128, Ultralytics 8.3.206,
CPU com duas threads, entrada sintética preta 640×640. Este teste verifica
compatibilidade e saída finita; **não mede precisão de detecção**.

| Modelo | Média de duas inferências aquecidas (CPU) |
| --- | ---: |
| Equipamentos YOLOv8x | 0,816 s |
| Equipamentos YOLOv8n, variantes | 0,038–0,042 s |
| Flare YOLOv8n | 0,037 s |
| Corrosão YOLOv8x detecção | 0,865 s |
| Corrosão YOLOv8n detecção | 0,040 s |
| Corrosão YOLOv8x segmentação | 1,180 s |
| Trincas YOLOv8x segmentação | 1,105 s |
| Trincas YOLOv8n segmentação | 0,081 s |

Amostra pequena; tempos não representam benchmark nem latência ROS ponta a
ponta. Segmentadores grandes ultrapassaram o limite padrão de idade de frame
de 1 s só na inferência CPU; não aumentar esse limite automaticamente. O caminho
hierárquico pode executar equipamento e anomalia por recorte e custar ainda mais.

## Inconsistências encontradas

- `plataform_objects_yolov8n_detection_300ep.pt` contém classes `0` a `8`.
  Não há evidência de correspondência com os nomes de equipamentos do catálogo.
  Filtro por nome como `Flare` não encontrará essa classe nesse modelo. O catálogo
  agora mostra os nomes reais e registra aviso; os pesos não foram alterados.
- Os dois detectores de corrosão contêm dez classes, incluindo `car`,
  `copper corrosion` e níveis de corrosão, não apenas `Rust`. O catálogo foi
  corrigido para refletir exatamente os pesos. Isso merece revisão do dataset
  antes de usar suas detecções como evidência de anomalia.
- Diferenças de maiúsculas/minúsculas em `flare` e `rust` também foram corrigidas
  no catálogo. O filtro de alvo existente compara nomes sem distinguir caixa.

Datas dos checkpoints locais vão de 19 a 25/01/2026. SHA-256, versão declarada
pelo checkpoint e classes foram registrados no catálogo. Os nomes de datasets
são os informados pelo projeto; não comprovam URL, licença ou proveniência do
treinamento. Esses dados continuam dependendo dos artefatos de treinamento.

Evidências completas e script utilizado:
`.drone-v2-validation/vision-real/{report.json,check.py,check.log}`.
Ainda falta medir o caminho ROS completo e executar uma missão integrada.

## Medição ROS ponta a ponta

CVNode real, pesos padrão `plataform_objects_yolov8x_detection.pt` e
`corrosion_yolo8x_detection.pt`, CPU com duas threads, domínio DDS 174.
Quatro imagens JPEG pretas 640×640 foram publicadas sequencialmente em
`/drone_inspetor/externo/camera/compressed`; a saída foi recebida em
`/drone_inspetor/interno/cv_node/compressed`, preservando o timestamp.

Latências publicação–recepção: **1,287; 0,962; 1,091; 0,981 s**. Isso inclui
transporte ROS, decodificação, inferência e codificação JPEG, em dois nós no
mesmo processo. Não inclui aquisição física, rede entre máquinas, fila contínua
ou inferência de anomalias por recortes (a imagem não contém equipamentos).

O caminho real está operacional, mas os modelos grandes em CPU ficam próximos
ou acima da validade padrão de frame (1 s). Os testes não justificam aumentar
essa validade. Medir GPU e carga real antes de escolher modelos/taxa de câmera
para operação. Não foi alterado o padrão `inference_device=auto`.

Script, relatório e log: `.drone-v2-validation/vision-ros/`.
Os nós foram encerrados normalmente. Missão integrada ainda pendente.
