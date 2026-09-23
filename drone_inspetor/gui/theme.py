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
    "dark_background": "#2c3e50",
    "light_background": "#34495e",
    "text_color": "#ecf0f1",
    "border_color": "#7f8c8d",
    "accent_color": "#3498db",
    "error_color": "#e74c3c",
    "success_color": "#27ae60"
}
