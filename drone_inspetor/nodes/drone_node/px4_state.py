# px4_state.py
# =================================================================================================
# ESTADO INTERNO DO PX4 (AUTOPILOT)
# =================================================================================================
# Encapsula toda a telemetria e flags de status recebidas do PX4.
# Usado pelo DroneNode como container de dados atualizado pelos callbacks PX4.
# =================================================================================================

from px4_msgs.msg import VehicleStatus


class DroneStatePX4:
    """
    Encapsula o estado interno do PX4 (autopilot).
    Contém todas as variáveis relacionadas à telemetria e status do PX4.
    """

    def __init__(self):
        """Inicializa todas as variáveis de estado do PX4 com valores padrão."""
        # Status e controle do veículo
        self.nav_state = None            # Modos de Navegação atual do PX4
        self.local_position = None        # Última posição local (X, Y, Z) em metros
        self.home_position = None         # Posição de home (ponto de partida)
        self.global_position = None       # Última posição global (latitude, longitude, altitude)
        self.vehicle_attitude = None      # Última atitude (orientação) do drone
        self.current_yaw_deg = 0.0        # Yaw atual do drone (em graus)
        self.current_yaw_rad = 0.0        # Yaw atual do drone (em radianos)
        self.current_yaw_deg_normalized = 0.0  # Yaw atual do drone (em graus) normalizado (-180 a 180)
        self.current_control_mode = None   # Modo de controle atual do PX4
        self.is_armed = False             # Flag booleano para o estado de armamento
        self.is_landed = False            # Flag booleano para o estado de pouso
        self.last_command_ack = None      # Última confirmação de comando recebida do PX4
        self.battery_status = None        # Status da bateria

        # --- Velocidade Corrente (m/s) ---
        self.current_velocity_x = 0.0     # Velocidade X local (m/s, Norte)
        self.current_velocity_y = 0.0     # Velocidade Y local (m/s, Leste)
        self.current_velocity_z = 0.0     # Velocidade Z local (m/s, para baixo)

        # --- Aceleração Corrente (m/s²) ---
        self.current_acceleration_x = 0.0  # Aceleração X local (m/s²)
        self.current_acceleration_y = 0.0  # Aceleração Y local (m/s²)
        self.current_acceleration_z = 0.0  # Aceleração Z local (m/s²)

        # --- Referência HOME (atualizada via home_position_callback) ---
        self.home_global_lat = None       # Latitude do HOME (onde local X=0)
        self.home_global_lon = None       # Longitude do HOME (onde local Y=0)
        self.home_global_alt = None       # Altitude do HOME (onde local Z=0)
        self.home_local_position = None   # Posição local do HOME [x, y, z] (deve ser ~[0,0,0])
        self.home_yaw_deg = None          # Yaw do drone ao armar (graus, 0 a 360)
        self.home_yaw_deg_normalized = None  # Yaw do drone ao armar (graus, -180 a 180)
        self.home_yaw_rad = None          # Yaw do drone ao armar (radianos, -pi a pi)

    def get_nav_state_name(self) -> str:
        """Retorna o nome legível do nav_state atual."""
        NAV_STATE_NAMES = {
            VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
            VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTITUDE_CONTROL",
            VehicleStatus.NAVIGATION_STATE_POSCTL: "POSITION_CONTROL",
            VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "AUTO_MISSION",
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "AUTO_LOITER",
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "AUTO_RTL",
            VehicleStatus.NAVIGATION_STATE_ACRO: "ACRO",
            VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
            VehicleStatus.NAVIGATION_STATE_STAB: "STABILIZED",
            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "AUTO_TAKEOFF",
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "AUTO_LAND",
            VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET: "FOLLOW_TARGET",
            VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND: "PRECISION_LAND",
            VehicleStatus.NAVIGATION_STATE_ORBIT: "ORBIT",
            VehicleStatus.NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: "VTOL_TAKEOFF",
            VehicleStatus.NAVIGATION_STATE_DESCEND: "DESCEND",
            VehicleStatus.NAVIGATION_STATE_TERMINATION: "TERMINATION",
        }
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")
