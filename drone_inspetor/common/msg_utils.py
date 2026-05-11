"""Utilitário genérico para converter mensagens ROS2 em dicts Python."""


def msg_to_dict(msg) -> dict:
    """
    Converte qualquer mensagem ROS2 em um dict Python.
    Usa introspection via get_fields_and_field_types() para mapear
    automaticamente todos os campos.

    Campos do tipo sequence (list) são convertidos para list() nativo.

    Args:
        msg: Instância de qualquer mensagem ROS2 gerada por rosidl.

    Returns:
        dict com {nome_campo: valor} para todos os campos da mensagem.
    """
    result = {}
    for field, ftype in msg.get_fields_and_field_types().items():
        value = getattr(msg, field)
        if 'sequence' in ftype or isinstance(value, (list, tuple)):
            value = list(value)
        result[field] = value
    return result
