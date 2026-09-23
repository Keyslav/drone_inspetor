"""Snapshots imutáveis de detecção para atravessar a fronteira ROS→Qt."""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class DetectionView:
    """Detecção copiada da mensagem; caixas em coordenadas de imagem."""

    object_type: str
    class_name: str
    confidence: float
    bbox: tuple[float, ...]
    bbox_center: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class DetectionFrame:
    """Resultado completo de um frame sem referência à mensagem ROS original."""

    timestamp: str
    detections: tuple[DetectionView, ...] = ()

    @classmethod
    def from_message(cls, message: Any) -> 'DetectionFrame':
        """Copia campos na borda ROS, sem importar as classes geradas."""
        return cls(
            timestamp=str(message.timestamp),
            detections=tuple(
                DetectionView(
                    object_type=str(item.object_type),
                    class_name=str(item.class_name),
                    confidence=float(item.confidence),
                    bbox=tuple(float(value) for value in item.bbox),
                    bbox_center=tuple(float(value) for value in item.bbox_center),
                )
                for item in message.detections
            ),
        )

    @property
    def count(self) -> int:
        """Usa o conteúdo como autoridade, evitando contagens inconsistentes."""
        return len(self.detections)


def format_analysis_report(frame: DetectionFrame, logs: list[dict], updated_at: str) -> str:
    """Formata o relatório exibido e exportado, sem dependência dos widgets."""
    content = [
        '=' * 80,
        'LOGS DE ANÁLISE DE VISÃO COMPUTACIONAL',
        '=' * 80,
        f'Última atualização: {updated_at}',
        f'Total de análises: {len(logs)}',
        f'Detecções atuais: {frame.count}',
        '',
    ]
    if frame.detections:
        content.extend(['DETECÇÕES ATUAIS:', '-' * 40])
        for index, detection in enumerate(frame.detections, start=1):
            label = detection.class_name or detection.object_type
            content.append(f'{index}. {label} (confiança: {detection.confidence:.2f})')
        content.append('')
    if logs:
        quality = sum(log.get('quality_score', 0) for log in logs) / len(logs)
        sharpness = sum(log.get('sharpness_score', 0) for log in logs) / len(logs)
        content.extend([
            'ESTATÍSTICAS:', '-' * 40,
            f'Qualidade média: {quality:.1f}/100', f'Nitidez média: {sharpness:.1f}', '',
        ])
    content.extend(['LOGS RECENTES (últimos 10):', '-' * 40])
    for log in reversed(logs[-10:]):
        detections = log.get('detections', [])
        content.append(
            f"[{log.get('timestamp', 'N/A')}] Qualidade: {log.get('quality_score', 0):.1f}"
            f" | Nitidez: {log.get('sharpness_score', 0):.1f} | Detecções: {len(detections)}"
        )
        for detection in detections:
            label = detection.get('object_type', detection.get('label', 'unknown'))
            content.append(f"    → {label} ({detection.get('confidence', 0):.2f})")
        content.append('')
    return '\n'.join(content)
