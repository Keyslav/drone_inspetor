"""Importações de compatibilidade; novas telas usam os módulos específicos."""

from .logging import gui_log_debug, gui_log_error, gui_log_info, gui_log_warn
from .presentation.formatting import (
    calculate_distance, format_coordinates, format_timestamp, quaternion_to_euler,
)
from .theme import COMMON_STYLES, IMAGE_QUALITY
from .widgets.images import ImageProcessor
from .widgets.screens import BaseScreen, ExpandedWindow

__all__ = [
    "BaseScreen", "ExpandedWindow", "ImageProcessor", "COMMON_STYLES",
    "IMAGE_QUALITY", "calculate_distance", "format_coordinates",
    "format_timestamp", "quaternion_to_euler", "gui_log_debug",
    "gui_log_error", "gui_log_info", "gui_log_warn",
]
