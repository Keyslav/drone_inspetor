"""Câmera com republicação e persistência de mídia da missão."""

__all__ = ['CameraNode', 'main']


def __getattr__(name):
    """Mantém o entry point sem importação antecipada de ROS."""
    if name in __all__:
        from .camera_node import CameraNode, main
        return {'CameraNode': CameraNode, 'main': main}[name]
    raise AttributeError(name)
