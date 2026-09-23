# LiDAR para apresentação

O nó mantém o contrato `LidarMSG` do dashboard. `processing.py` interpreta ângulos
como `angle_min + i * angle_increment`; +90° corresponde à esquerda em FLU. Cada
scan substitui completamente as flags anteriores, sem cooldown que retenha
obstáculos já ausentes. Leituras positivas abaixo do mínimo viram hits no mínimo;
NaN/Inf/negativos não são retornos válidos. O sensor inferior publica NaN quando não
há uma distância válida, limpando a medida antiga na GUI.

`lidar_data_publish_rate` e `obstacles_publish_rate` (ambos 5Hz) limitam a taxa de
apresentação. Os timers só publicam mudanças de snapshot. `sensor_timeout_seconds`
(padrão 0.75s) usa tempo monotônico para expirar dados; timestamp repetido ou antigo
não renova a observação. Na expiração, o nó emite uma única limpeza das listas,
distância e flags. As flags não distinguem desconhecido de ausência de detecção:
são somente apresentação. A navegação utiliza scans originais com distância,
referencial e timestamp, e verifica a disponibilidade de seus sensores obrigatórios.
