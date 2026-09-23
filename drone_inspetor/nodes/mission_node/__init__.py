"""Missão: domínio importável sem inicializar ROS, Qt ou inferência."""


def main(args=None):
    """Preserva o entry point instalado, carregando ROS apenas na execução."""
    from drone_inspetor.nodes.mission_node.mission_node import main as run
    return run(args)


def __getattr__(name):
    if name == 'MissionNode':
        from drone_inspetor.nodes.mission_node.mission_node import MissionNode
        return MissionNode
    raise AttributeError(name)
