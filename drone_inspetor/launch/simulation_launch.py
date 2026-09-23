"""Bridges ROS/Gazebo para um simulador já em execução."""

from drone_inspetor.launch.composition import create_launch


def generate_launch_description():
    """Não inicia PX4/Gazebo nem pressupõe diretórios locais de modelos."""
    return create_launch(simulation=True, bridges=True, application=False)
