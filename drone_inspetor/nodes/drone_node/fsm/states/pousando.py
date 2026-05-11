# pousando.py
# Estado: POUSANDO
# Drone descendo para pousar (controlado pelo PX4 via AUTO_LAND).

from drone_inspetor.common.state import State
from drone_inspetor.common.enums import DroneStateDescription as DS


class PousandoState(State):
    """
    Drone em processo de pouso.

    O controle de descida é realizado pelo PX4 (modo AUTO_LAND ou comando LAND).
    Este estado aguarda a confirmação de pouso via px4.is_landed.
    Não transiciona automaticamente — o desarme subsequente será capturado
    pela verificação global de desarmamento em DroneFSM.
    """

    def on_step(self):
        if self.context.state_px4.is_landed:
            self.node.get_logger().info(
                "Pouso completo. Aguardando desarme automático do PX4...",
                throttle_duration_sec=5.0,
            )
        return None
