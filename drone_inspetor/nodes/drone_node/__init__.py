"""Controle de voo; importar os algoritmos não inicializa dependências ROS."""


def main(args=None):
    """Entry point compatível, com carregamento explícito do adaptador ROS."""
    from .drone_node import main as run
    return run(args)


def __getattr__(name):
    if name == 'DroneNode':
        from .drone_node import DroneNode
        return DroneNode
    raise AttributeError(name)
