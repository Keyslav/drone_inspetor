"""Visão computacional; componentes puros não carregam ROS ou pesos YOLO."""

__all__ = ['CVNode', 'main']


def __getattr__(name):
    """Preserva o entry point público sem importar o nó antecipadamente."""
    if name in __all__:
        from .cv_node import CVNode, main
        return {'CVNode': CVNode, 'main': main}[name]
    raise AttributeError(name)
