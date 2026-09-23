"""Inferência hierárquica e renderização sem dependência de ROS ou arquivos."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class InferenceOptions:
    """Configuração imutável usada durante todo o processamento de um frame."""

    object_confidence: float = 0.5
    anomaly_confidence: float = 0.5
    target: str = ''
    filter_target: bool = False
    enable_anomalies: bool = False
    device: object = 'cpu'


@dataclass(frozen=True)
class AnomalyCapture:
    """Cinco vistas da mesma observação para documentação da inspeção."""

    object_name: str
    original: np.ndarray
    object_image: np.ndarray
    annotated: np.ndarray
    crop: np.ndarray
    annotated_crop: np.ndarray


def _boxes(result, minimum_confidence):
    """Transfere tensores para CPU uma vez por resultado de inferência."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return
    coordinates = boxes.xyxy.cpu().numpy()
    confidence = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    for index, (bbox, score, class_id) in enumerate(zip(coordinates, confidence, classes)):
        if np.isfinite(bbox).all() and np.isfinite(score) and score > minimum_confidence:
            yield index, bbox, float(score), int(class_id)


def _clip_bbox(bbox, width, height):
    x1, y1, x2, y2 = bbox
    return (max(0, min(width, int(x1))), max(0, min(height, int(y1))),
            max(0, min(width, int(x2))), max(0, min(height, int(y2))))


def _draw_bbox(image, bbox, label, color, scale=0.5):
    x1, y1, x2, y2 = bbox
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(image, label, (x1, max(12, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)


def _draw_anomaly(image, result, index, offset, bbox, label):
    masks = getattr(result, 'masks', None)
    if masks is not None and len(masks.xy) > index and masks.xy[index].size:
        segment = masks.xy[index].copy()
        segment += np.asarray(offset)
        segment = segment.astype(np.int32)
        overlay = image.copy()
        cv2.fillPoly(overlay, [segment], (0, 0, 255))
        cv2.addWeighted(overlay, 0.4, image, 0.6, 0, image)
        cv2.drawContours(image, [segment], -1, (0, 0, 255), 2)
    else:
        _draw_bbox(image, bbox, label, (0, 0, 255), scale=0.4)


class DetectionPipeline:
    """Produz detecções e imagens; o nó decide publicação e persistência."""

    def __init__(self, models):
        self.models = models

    def process(self, image, options):
        """Usa um par consistente de modelos e ignora crops fora da imagem."""
        with self.models.snapshot() as (objects, anomalies):
            if objects is None:
                return image, [], []
            classes = None
            if options.filter_target and options.target:
                names = objects.names
                entries = names.items() if isinstance(names, dict) else enumerate(names)
                classes = [index for index, name in entries
                           if name.casefold() == options.target.casefold()]
                if not classes:
                    return image, [], []
            results = objects.predict(image, verbose=False, device=options.device, classes=classes)
            if not results:
                return image, [], []
            annotated = image.copy()
            detections, captures = [], []
            height, width = image.shape[:2]
            for _, bbox, score, class_id in _boxes(results[0], options.object_confidence):
                x1, y1, x2, y2 = _clip_bbox(bbox, width, height)
                if x2 <= x1 or y2 <= y1:
                    continue
                name = objects.names[class_id]
                _draw_bbox(annotated, (x1, y1, x2, y2), f'{name}: {score:.2f}', (0, 255, 0))
                detection = {
                    'object_type': name.lower(), 'class': name, 'confidence': score,
                    'bbox': [x1, y1, x2, y2],
                    'bbox_center': [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                    'anomalies': [],
                }
                if options.enable_anomalies and anomalies is not None:
                    object_image = annotated.copy()
                    crop = image[y1:y2, x1:x2]
                    anomaly_results = anomalies.predict(crop, verbose=False, device=options.device)
                    if anomaly_results:
                        for index, local_box, confidence, anomaly_id in _boxes(
                            anomaly_results[0], options.anomaly_confidence,
                        ):
                            relative = _clip_bbox(local_box, x2 - x1, y2 - y1)
                            ax1, ay1, ax2, ay2 = relative
                            if ax2 <= ax1 or ay2 <= ay1:
                                continue
                            absolute = [ax1 + x1, ay1 + y1, ax2 + x1, ay2 + y1]
                            anomaly_name = anomalies.names[anomaly_id]
                            _draw_anomaly(annotated, anomaly_results[0], index, (x1, y1),
                                          absolute, f'{anomaly_name}: {confidence:.2f}')
                            detection['anomalies'].append({
                                'class': anomaly_name, 'confidence': confidence,
                                'bbox': absolute, 'bbox_relative': list(relative),
                            })
                    if detection['anomalies']:
                        captures.append(AnomalyCapture(
                            name, image.copy(), object_image, annotated.copy(),
                            crop.copy(), annotated[y1:y2, x1:x2].copy(),
                        ))
                detections.append(detection)
            return annotated, detections, captures
