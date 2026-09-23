"""Aplicação completa com relógio real e sensores ROS externos."""

from drone_inspetor.launch.composition import create_launch


def generate_launch_description():
    """Parâmetros e seleção de subsistemas são compartilhados com o dashboard."""
    return create_launch()
