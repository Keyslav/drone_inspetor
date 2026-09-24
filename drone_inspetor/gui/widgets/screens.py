"""Janelas expandidas e atualização de imagens compartilhadas."""

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QLabel, QMainWindow, QSizePolicy

from ..theme import COMMON_STYLES, IMAGE_QUALITY


class ExpandedWindow(QMainWindow):
    """Janela independente que fecha ao receber duplo clique."""

    def __init__(self, title, content_widget, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'{title} - AMPLIADO')
        self.setStyleSheet(
            f"QMainWindow {{ background-color: {COMMON_STYLES['dark_background']};"
            f"color: {COMMON_STYLES['text_color']}; }}"
        )
        self.setCentralWidget(content_widget)
        self.showMaximized()

    def mouseDoubleClickEvent(self, event):
        """Permite retornar à visualização principal sem alterar seu estado."""
        self.close()


class BaseScreen:
    """Mantém uma imagem principal e janelas independentes da mesma tela."""

    def __init__(self, video_label, screen_name):
        self.video_label = video_label
        self.screen_name = screen_name
        self.expanded_windows = []
        self._last_pixmap = None
        self._target_size = IMAGE_QUALITY['main_display_size']
        if video_label is not None:
            video_label.setCursor(Qt.CursorShape.PointingHandCursor)
            video_label.mousePressEvent = self.expand_screen
            video_label.setMinimumSize(*IMAGE_QUALITY['min_widget_size'])
            video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    @staticmethod
    def _fit(pixmap, width, height):
        return pixmap.scaled(
            max(1, width), max(1, height), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )

    def expand_screen(self, event=None):
        """Reutiliza a janela visível ou abre uma cópia da imagem atual."""
        self.expanded_windows = [window for window in self.expanded_windows if window.isVisible()]
        if self.expanded_windows:
            window = self.expanded_windows[0]
            window.raise_()
            window.activateWindow()
            return
        label = QLabel()
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        if self._last_pixmap is not None:
            label.setPixmap(self._fit(self._last_pixmap, *IMAGE_QUALITY['expanded_display_size']))
        else:
            label.setText(f'Aguardando {self.screen_name}...')
        window = ExpandedWindow(self.screen_name, label)
        self.expanded_windows.append(window)

    def update_display(self, q_image):
        """Preserva o frame original para a expansão não ampliar uma miniatura."""
        if q_image is None or q_image.isNull() or self.video_label is None:
            return
        self._last_pixmap = QPixmap.fromImage(q_image)
        width, height = self.video_label.width(), self.video_label.height()
        if width <= 0 or height <= 0:
            width, height = self._target_size
        if hasattr(self.video_label, 'set_source_pixmap'):
            self.video_label.set_source_pixmap(self._last_pixmap)
        else:
            self.video_label.setPixmap(self._fit(self._last_pixmap, width, height))
        self.update_expanded_windows(self._last_pixmap)

    def update_expanded_windows(self, pixmap):
        """Atualiza apenas janelas abertas que exibem uma imagem."""
        self.expanded_windows = [window for window in self.expanded_windows if window.isVisible()]
        for window in self.expanded_windows:
            label = window.centralWidget()
            if isinstance(label, QLabel):
                label.setPixmap(self._fit(pixmap, *IMAGE_QUALITY['expanded_display_size']))

    def close(self):
        """Encerra as janelas independentes ao fechar o dashboard."""
        for window in self.expanded_windows:
            window.close()
        self.expanded_windows.clear()
