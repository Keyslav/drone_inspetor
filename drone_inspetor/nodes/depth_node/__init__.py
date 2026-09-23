"""Projeção métrica e visualização da câmera de profundidade."""

__all__ = ['DepthNode', 'main']


def __getattr__(name):
    if name in __all__:
        from .depth_node import DepthNode, main
        return {'DepthNode': DepthNode, 'main': main}[name]
    raise AttributeError(name)
