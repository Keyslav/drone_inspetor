"""Processamento dos scans horizontal e inferior do LiDAR."""

__all__ = ['LidarNode', 'main']


def __getattr__(name):
    if name in __all__:
        from .lidar_node import LidarNode, main
        return {'LidarNode': LidarNode, 'main': main}[name]
    raise AttributeError(name)
