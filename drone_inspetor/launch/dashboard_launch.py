"""Dashboard e aplicação completos conectados a uma simulação existente."""

from drone_inspetor.launch.composition import create_launch


def generate_launch_description():
    """Permite desabilitar nós por with_<componente>:=false."""
    return create_launch(simulation=True, bridges=True)
