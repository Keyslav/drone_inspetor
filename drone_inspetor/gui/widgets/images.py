"""Conversão de frames em imagens Qt com memória independente."""

import cv2
import numpy as np
from PyQt6.QtGui import QImage
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QSizePolicy

from ..logging import gui_log_error


class ResponsiveImageLabel(QLabel):
    """Redimensiona o original mesmo quando o próximo frame ainda não chegou."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._source_pixmap = None
        # A resolução do sensor não deve determinar a largura mínima do painel.
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)

    def set_source_pixmap(self, pixmap):
        self._source_pixmap = pixmap
        self._fit_source()

    def _fit_source(self):
        if self._source_pixmap is not None:
            self.setPixmap(self._source_pixmap.scaled(
                self.contentsRect().size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._fit_source()


class ImageProcessor:
    """Converte imagens sem manter referências a buffers de callbacks ROS."""

    def __init__(self):
        self.bridge = None

    def ros_to_qimage(self, msg, encoding="bgr8"):
        """Converte a mensagem apenas na borda; o caminho OpenCV dispensa ROS."""
        from cv_bridge import CvBridge

        if self.bridge is None:
            self.bridge = CvBridge()
        try:
            return self.cv_to_qimage(self.bridge.imgmsg_to_cv2(msg, encoding))
        except Exception as exc:
            gui_log_error("ImageProcessor", f"Erro na conversão ROS→Qt: {exc}")
            return None

    def cv_to_qimage(self, cv_image):
        """Retorna uma cópia Qt de um frame uint8 BGR/BGRA/cinza, ou None."""
        if cv_image is None or cv_image.size == 0:
            return None
        if cv_image.dtype != np.uint8:
            gui_log_error("ImageProcessor", "O frame de exibição deve ser uint8")
            return None
        if cv_image.ndim == 2:
            pixels = cv_image
            image_format = QImage.Format.Format_Grayscale8
        elif cv_image.ndim == 3 and cv_image.shape[2] == 3:
            pixels = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image_format = QImage.Format.Format_RGB888
        elif cv_image.ndim == 3 and cv_image.shape[2] == 4:
            pixels = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGBA)
            image_format = QImage.Format.Format_RGBA8888
        else:
            gui_log_error("ImageProcessor", f"Formato de frame inválido: {cv_image.shape}")
            return None
        pixels = np.ascontiguousarray(pixels)
        height, width = pixels.shape[:2]
        # QImage(buffer, ...) não é dona da memória numpy. A cópia preserva o
        # frame quando o callback libera/reutiliza o array entre as threads.
        return QImage(
            pixels.data, width, height, pixels.strides[0], image_format
        ).copy()
