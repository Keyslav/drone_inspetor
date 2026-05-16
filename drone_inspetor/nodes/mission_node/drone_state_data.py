# drone_state_data.py
# =================================================================================================
# ESPELHO DE TELEMETRIA DO DRONE PARA O MISSION NODE
# =================================================================================================
# Encapsula todos os campos de DroneStateMSG como atributos Python.
# Usa introspection de ROS para inicialização e atualização automática.
# =================================================================================================

from drone_inspetor_msgs.msg import DroneStateMSG
from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription


# Campos que possuem defaults especiais (diferentes de 0.0 / False / "")
_SPECIAL_DEFAULTS = {
    'state': int(DroneFSMDescription.OFFBOARD_DESATIVADO),
}

# Mapeamento tipo ROS → default Python
_TYPE_DEFAULTS = {
    'boolean': False,
    'string': '',
}


class DroneStateData:
    """
    Espelho local da telemetria do drone (DroneStateMSG).

    Todos os campos são criados automaticamente a partir da definição do .msg,
    garantindo que novos campos adicionados à mensagem sejam refletidos
    sem mudança nesta classe.
    """

    def __init__(self):
        for field, ftype in DroneStateMSG.get_fields_and_field_types().items():
            if field in _SPECIAL_DEFAULTS:
                default = _SPECIAL_DEFAULTS[field]
            else:
                default = _TYPE_DEFAULTS.get(ftype, 0.0)
            setattr(self, field, default)
        # Propriedade derivada: enum tipado do state (atualizado em update_from_msg)
        self.state = DroneFSMDescription.OFFBOARD_DESATIVADO

    def update_from_msg(self, msg: DroneStateMSG):
        """Atualiza todos os campos a partir de uma mensagem DroneStateMSG."""
        for field in msg.get_fields_and_field_types():
            setattr(self, field, getattr(msg, field))
        # Converte state int → enum tipado
        try:
            self.state = DroneFSMDescription(msg.state)
        except ValueError:
            pass
