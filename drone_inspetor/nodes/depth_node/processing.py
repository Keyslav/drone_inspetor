"""Estatística, alertas e visualização sem estado ou dependência de ROS."""

import cv2
import numpy as np


def filtered_depth(depth, minimum, maximum):
    """Mantém somente profundidades métricas finitas dentro do alcance de exibição."""
    valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
    return np.where(valid, depth, 0.0)


def depth_statistics(depth, timestamp=''):
    """Preserva os campos JSON do dashboard inclusive quando não há pixels válidos."""
    valid = depth[np.isfinite(depth) & (depth > 0)]
    result = {'timestamp': timestamp, 'total_pixels': int(depth.size),
              'valid_pixels': int(valid.size),
              'valid_percentage': 100.0 * valid.size / depth.size if depth.size else 0.0}
    if valid.size == 0:
        result['error'] = 'Nenhum pixel de profundidade válido'
        return result
    result.update({
        'min_distance': float(valid.min()), 'max_distance': float(valid.max()),
        'mean_distance': float(valid.mean()), 'median_distance': float(np.median(valid)),
        'std_distance': float(valid.std()), 'range_meters': float(np.ptp(valid)),
    })
    for name, mask in (('close', valid <= 1.0), ('medium', (valid > 1) & (valid <= 5)),
                       ('far', valid > 5)):
        count = int(mask.sum())
        result[f'{name}_pixels'] = count
        result[f'{name}_percentage'] = 100.0 * count / valid.size
    return result


def proximity_alerts(depth, threshold, timestamp=''):
    """Uma lista vazia significa que o alerta anterior deve ser limpo na GUI."""
    valid = depth[np.isfinite(depth) & (depth > 0)]
    close = valid[valid <= threshold]
    if close.size == 0:
        return []
    percentage = 100.0 * close.size / valid.size
    level = ('CRÍTICO' if percentage > 50 else 'ALTO' if percentage > 25
             else 'MÉDIO' if percentage > 10 else 'BAIXO')
    return [{'id': f'proximity_{timestamp.replace(":", "")}', 'type': 'proximity',
             'level': level, 'min_distance': float(close.min()),
             'affected_pixels': int(close.size), 'affected_percentage': round(percentage, 1),
             'threshold': threshold, 'timestamp': timestamp}]


def render_depth(depth, statistics, alerts, mode='grayscale'):
    """Compartilha normalização dos dois modos e representa desconhecido em preto."""
    valid = np.isfinite(depth) & (depth > 0)
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid):
        values = depth[valid]
        span = float(np.ptp(values))
        normalized[valid] = ((values - values.min()) / span * 254 + 1).astype(np.uint8) if span else 127
    if mode == 'colormap':
        rendered = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    else:
        rendered = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
    rendered[~valid] = 0
    if 'min_distance' in statistics:
        text = f"Min: {statistics['min_distance']:.2f}m  Media: {statistics['mean_distance']:.2f}m"
        cv2.putText(rendered, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    for index, alert in enumerate(alerts):
        cv2.putText(rendered, f"ALERTA {alert['level']}: {alert['min_distance']:.2f}m",
                    (10, max(15, rendered.shape[0] - 20 - index * 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
    return rendered
