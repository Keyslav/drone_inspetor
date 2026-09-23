# Profundidade métrica e apresentação

`projection.py` converte profundidade óptica em distâncias horizontais radiais.
`processing.py` calcula estatísticas, alertas e imagens para o dashboard.
`depth_node.py` é somente o adaptador ROS entre esses componentes e os tópicos.

O tópico `Topics.Interno.DEPTH_SCAN` usa `sensor_msgs/LaserScan`. O timestamp é o
mesmo da imagem de origem; `frame_id` é `scan_frame_id`, cuja origem está na câmera
e cujos eixos são FLU (frente, esquerda, cima), paralelos aos do drone após aplicar
o yaw de montagem. `NavigationSensors` deve aplicar somente a translação da câmera;
o yaw de montagem já está contido nos ângulos do scan. Nenhum timer republica o scan.

## Calibração e parâmetros

| Parâmetro | Padrão | Contrato |
|---|---:|---|
| `scan_enabled` | `true` | Permite produzir scan se a calibração estiver completa |
| `scan_horizontal_fov_deg` | `0.0` | HFOV medido em graus; zero significa desconhecido |
| `scan_fx`, `scan_fy` | `0.0` | Focais em pixels, ambas necessárias na opção intrínsecos |
| `scan_cx`, `scan_cy` | `-1.0` | Centro óptico; negativo usa o centro da resolução calibrada |
| `scan_calibration_width`, `scan_calibration_height` | `0` | Resolução dos intrínsecos; exigida quando `fx`/`fy` são positivos |
| `scan_mount_yaw_deg` | `0.0` | Rotação anti-horária FLU em relação à frente do drone |
| `scan_camera_height_m` | `0.0` | Altura da câmera acima do centro do drone |
| `scan_band_half_height_m` | `0.35` | Meia altura da faixa considerada ao redor do centro do drone |
| `scan_bins` | `181` | Número de bins angulares uniformes dentro do campo observado |
| `scan_frame_id` | `depth_scan_flu` | Frame de origem câmera e orientação FLU do drone |
| `min_depth`, `max_depth` | `0.1`, `10.0` | Alcance óptico válido, em metros |
| `proximity_threshold` | `1.0` | Limiar de alerta da imagem inteira no dashboard |
| `visualization_mode` | `grayscale` | `grayscale` ou `colormap` |

Os intrínsecos têm precedência sobre HFOV e são escalados quando a resolução muda.
Na alternativa HFOV, assume-se `fx=fy` (pixels quadrados) e centro óptico no centro
da imagem. A câmera precisa estar nivelada e a imagem retificada; montagem com
pitch/roll, lente distorcida ou extrínsecos não medidos exige retificação/calibração
antes de habilitar o scan. Os padrões não inventam o campo de visão: sem HFOV ou
intrínsecos fornecidos, somente o dashboard recebe dados.

`32FC1` representa metros; `16UC1` representa milímetros. Outros encodings são
recusados. NaN, ±Inf, zero, negativos e valores acima de `max_depth` permanecem
**desconhecidos (NaN)**. Retornos positivos abaixo do mínimo são conservados como
hits em `range_min`; nunca viram espaço livre. O menor retorno válido em cada bin
vence. Bins vazios não são interpolados nem extrapolados além do FOV.

O cálculo 3D elimina pontos fora da faixa vertical configurada. O scan radial usa
um `range_max` derivado da profundidade máxima e do maior ângulo observado. A
imagem e os alertas da GUI continuam analisando a imagem inteira, incluindo chão
ou teto; esses alertas não representam por si só um obstáculo no plano de voo.

A navegação trata depth como veto adicional opcional: somente hits calibrados e
recentes acrescentam obstáculos. Uma região desconhecida de depth não atesta
espaço livre. A disponibilidade obrigatória e a cobertura de navegação continuam
sendo verificadas pelo LiDAR. As mensagens booleanas legadas não representam
idade nem desconhecido e não devem ser usadas para decidir deslocamentos.

## Verificação

`test_depth_projection.py` cobre geometria FLU, radiais, FOV, altura, intrínsecos,
unidades e valores inválidos. `test_perception_ros_callbacks.py` usa mensagens ROS
reais sem iniciar nós: confirma timestamp/frame, emissão de lista vazia para limpar
alertas e independência da publicação métrica em relação ao renderer. Não valida
calibração física, latência em hardware ou voo.
