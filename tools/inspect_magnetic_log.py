#!/usr/bin/env python3
"""Inspeção somente leitura do campo magnético usando atitude groundtruth no ULog.

Requer pyulog e numpy. Usa os últimos 2 s de magnetômetro calibrado, sem
extrapolar a atitude. A referência é física, não a atitude estimada pelo EKF.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def inspect(path):
    log = ULog(str(path))
    mag = log.get_dataset('vehicle_magnetometer').data
    attitude = log.get_dataset('vehicle_attitude_groundtruth').data
    times = mag['timestamp_sample'].astype(float)
    attitude_times = attitude['timestamp_sample'].astype(float)
    selected = ((times >= times[-1] - 2e6) & (times >= attitude_times[0])
                & (times <= attitude_times[-1]))
    times = times[selected]
    if len(times) < 2:
        raise ValueError('Amostras sincronizadas insuficientes')
    # Quaternion PX4 wxyz: corpo FRD para mundo NED.
    q = np.column_stack([np.interp(times, attitude_times, attitude[f'q[{i}]'])
                         for i in range(4)])
    q /= np.linalg.norm(q, axis=1)[:, None]
    body = np.column_stack([mag[f'magnetometer_ga[{i}]'][selected] for i in range(3)])
    cross = 2 * np.cross(q[:, 1:], body)
    earth = body + q[:, :1] * cross + np.cross(q[:, 1:], cross)
    field = np.median(earth, axis=0)
    declination = float(np.degrees(np.arctan2(field[1], field[0])))
    expected = float(log.initial_parameters['EKF2_MAG_DECL'])
    error = (declination - expected + 180) % 360 - 180
    return dict(
        source=str(path.resolve()), samples=len(times),
        sample_interval_s=[float(times[0] / 1e6), float(times[-1] / 1e6)],
        body_frd_gauss_median=np.median(body, axis=0).tolist(),
        earth_ned_gauss_median=field.tolist(),
        earth_ned_gauss_std=np.std(earth, axis=0).tolist(),
        magnetic_declination_from_groundtruth_deg=declination,
        initial_px4_magnetic_declination_deg=expected,
        declination_error_deg=error,
        limitation='Referência EKF2_MAG_DECL inicial; não é certificado de calibração ou voo.',
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect(args.ulog), indent=2))
