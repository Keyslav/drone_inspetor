"""Paleta e dimensões compartilhadas pelas telas."""

IMAGE_QUALITY = {
    "main_display_size": (640, 480),        # Tamanho padrão fixo para display principal
    "expanded_display_size": (1600, 1200),  # Tamanho fixo para janelas expandidas
    "thumbnail_size": (320, 240),           # Tamanho para thumbnails
    "compression_quality": 85,
    "min_widget_size": (320, 240),          # Tamanho mínimo do widget
    "size_change_threshold": 5              # Threshold menor e mais permissivo
}

COMMON_STYLES = {
    "dark_background": "#0b1321",
    "light_background": "#111c2e",
    "text_color": "#e6edf5",
    "border_color": "#26354b",
    "accent_color": "#249eac",
    "error_color": "#ff7f88",
    "success_color": "#269f8b"
}
