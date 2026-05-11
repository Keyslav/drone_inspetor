"""
param_utils.py - Utilitários para carregamento de parâmetros ROS2.

Simplifica o padrão repetitivo de declare_parameter() + get_parameter() que
aparece em todos os nós do projeto. Os valores são carregados do arquivo
config/param_ros.yaml (passado via launch file), usando o default como fallback.

FLUXO DO PARÂMETRO ROS2:
    1. O launch file carrega o param_ros.yaml e associa os valores ao nó pelo nome
    2. declare_parameter(nome, default) registra o parâmetro no nó ROS2
       - Se o YAML forneceu um valor para esse nome, ele SUBSTITUI o default
       - Se o YAML não forneceu, o default é usado
    3. get_parameter(nome) recupera o valor final (YAML ou default)

USO:
    from drone_inspetor.common.param_utils import load_param

    # No __init__ de qualquer nó ROS2:
    step_distance = load_param(self, "step_distance", 5.0)
    photo_format  = load_param(self, "photo_format", "jpg")
    video_fps     = load_param(self, "video_fps", 15)
    video_enabled = load_param(self, "video_enabled", True)
"""

from rclpy.node import Node


def load_param(node: Node, name: str, default):
    """
    Declara e carrega um parâmetro ROS2 em uma única chamada.

    Encapsula o padrão verboso:
        node.declare_parameter("nome", default)
        valor = node.get_parameter("nome").get_parameter_value().<tipo>_value

    O tipo do valor retornado é inferido automaticamente a partir do default:
        - float  → double_value
        - int    → integer_value
        - str    → string_value
        - bool   → bool_value

    Args:
        node:    Instância do nó ROS2 (self dentro de um Node)
        name:    Nome do parâmetro (deve corresponder ao nome no param_ros.yaml)
        default: Valor padrão (usado se o param_ros.yaml não fornecer o parâmetro)

    Returns:
        O valor do parâmetro com o tipo correto.

    Raises:
        TypeError: Se o tipo do default não for suportado (float, int, str, bool).
    """
    node.declare_parameter(name, default)
    param_value = node.get_parameter(name).get_parameter_value()

    if isinstance(default, bool):
        # bool DEVE vir antes de int, pois bool é subclasse de int em Python
        return param_value.bool_value
    elif isinstance(default, int):
        return param_value.integer_value
    elif isinstance(default, float):
        return param_value.double_value
    elif isinstance(default, str):
        return param_value.string_value
    else:
        raise TypeError(
            f"Tipo de default não suportado para parâmetro '{name}': {type(default).__name__}. "
            f"Use float, int, str ou bool."
        )
