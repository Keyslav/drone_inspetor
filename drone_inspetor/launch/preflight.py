"""Diagnóstico do ambiente instalado antes de iniciar processos ROS."""

from importlib import import_module


def check_runtime(*, drone=False):
    """Recusa interfaces antigas e dependências ausentes com ação de reparo."""
    try:
        import_module('drone_inspetor.ros_interfaces')
    except ImportError as exc:
        raise RuntimeError(
            'Interfaces ROS ausentes ou incompatíveis com drone_inspetor v2. '
            'No workspace, execute: colcon build --symlink-install '
            '--packages-select drone_inspetor_msgs drone_inspetor; '
            'depois source install/setup.bash. Detalhe: ' + str(exc)
        ) from exc
    if drone:
        try:
            import_module('ruckig')
        except ImportError as exc:
            raise RuntimeError(
                'Ruckig ausente no Python deste launch. Instale ruckig==0.19.4 '
                'no mesmo ambiente usado para compilar/executar o projeto '
                '(veja requirements-runtime.txt e README).'
            ) from exc


def check_launch(context, components):
    """Launch só de bridges não exige dependências da aplicação."""
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration

    enabled = {name for name in components if IfCondition(
        LaunchConfiguration(f'with_{name}')).evaluate(context)}
    if enabled:
        check_runtime(drone='drone' in enabled)
    return []
