"""
Constantes numéricas centralizadas do sistema drone_inspetor.
"""


class DroneConstants:
    """Constantes de navegação, timeouts e parâmetros do drone."""

    # --- Conversão Geográfica ---
    METERS_PER_DEGREE = 111132.0  # Metros por grau de latitude (aprox.)

    # --- Timeouts de Comandos (segundos) ---
    COMMAND_TIMEOUTS = {
        "ARM": 10.0,
        "TAKEOFF": 60.0,
        "LAND": 60.0,
        "GOTO": 120.0,
        "RTL": 180.0,
    }
    DEFAULT_COMMAND_TIMEOUT = 60.0

    # --- Intervalos de Timer (segundos) ---
    OFFBOARD_PUBLISH_INTERVAL = 0.02    # Offboard control mode + trajectory setpoint
    DRONE_STATE_PUBLISH_INTERVAL = 0.5  # Atualização da máquina de estados do drone
    MISSION_UPDATE_INTERVAL = 0.6       # Loop principal da máquina de missão
    HEALTH_CHECK_INTERVAL = 2.0         # Verificação de saúde dos tópicos
    TOPIC_HEALTH_TIMEOUT = 5.0          # Timeout para considerar tópico morto

    # --- Parâmetros de Navegação ---
    DEFAULT_STEP_DISTANCE = 5.0         # Distância do próximo passo (metros)
    DEFAULT_YAW_STEP_DEG = 15.0         # Passo máximo de rotação em graus
    POSITION_TOLERANCE = 0.15           # Tolerância em metros para chegar ao destino
    YAW_TOLERANCE_DEG = 2.0             # Tolerância em graus para o yaw
    DEFAULT_TAKEOFF_ALTITUDE = 2.5      # Altitude padrão de decolagem (metros)
    DEFAULT_RTL_ALTITUDE = 30.0         # Altitude do RTL (metros)
    YAW_STABILIZATION_DELAY = 3.0       # Tempo de espera após alinhar yaw (segundos)

    # --- Limites de Bateria ---
    BATTERY_CRITICAL_THRESHOLD = 0.10   # 10% - nível crítico de bateria
