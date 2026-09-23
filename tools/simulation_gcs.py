#!/usr/bin/env python3
"""GCS mínima para o ensaio isolado (PX4 instância 27): somente heartbeat.

Não envia comandos de voo nem altera parâmetros. Requer pymavlink.
Não substitui uma estação de controle para operação real.
"""

import time

from pymavlink import mavutil


def main():
    connection = mavutil.mavlink_connection(
        'udpout:127.0.0.1:18597', source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    try:
        while True:
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            time.sleep(1)
    finally:
        connection.close()


if __name__ == '__main__':
    main()
