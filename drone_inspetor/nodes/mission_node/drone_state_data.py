"""Snapshot da telemetria pública; desconhecimento de estado bloqueia a missão."""

from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription

from drone_inspetor_msgs.msg import DroneStateMSG


class DroneStateData:
    """Preserva tipos/defaults da mensagem gerada e adiciona enum de ciclo de voo."""

    def __init__(self):
        """Usa os defaults tipados da interface ROS gerada."""
        self.update_from_msg(DroneStateMSG())

    def update_from_msg(self, message: DroneStateMSG):
        """Copia a mensagem e rejeita códigos desconhecidos como controle indisponível."""
        for field in message.get_fields_and_field_types():
            setattr(self, field, getattr(message, field))
        try:
            self.state = DroneFSMDescription(message.state)
        except ValueError:
            self.state = DroneFSMDescription.OFFBOARD_DESATIVADO
