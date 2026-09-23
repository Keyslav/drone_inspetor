#!/usr/bin/env python3
"""Ensaio TAKEOFF/LAND com Gazebo e PX4 próprios, sensores reais e arquivos isolados."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from validate_sitl import run_flight, stop_process
from drone_inspetor.common.coordinates import yaw_enu_to_ned


def fingerprint(path):
    """Registra a entrada efetivamente utilizada, sem modificar sua origem."""
    with path.open('rb') as stream:
        return {'path': str(path), 'sha256': hashlib.file_digest(stream, 'sha256').hexdigest()}


def obstacle_service(world, output, present):
    """Cilindro físico 6 m ao Norte do spawn, existente apenas neste servidor."""
    name = 'validation_obstacle'
    if present:
        sdf = ('<sdf version="1.9"><model name="validation_obstacle"><static>true</static>'
               '<pose>-70 -21 59 0 0 0</pose><link name="body">'
               '<collision name="collision"><geometry><cylinder><radius>0.4</radius>'
               '<length>8</length></cylinder></geometry></collision>'
               '<visual name="visual"><geometry><cylinder><radius>0.4</radius>'
               '<length>8</length></cylinder></geometry></visual></link></model></sdf>')
        (output / 'obstacle.sdf').write_text(sdf)
        service, request_type, request = 'create', 'EntityFactory', f'sdf: {json.dumps(sdf)}'
    else:
        service, request_type, request = 'remove', 'Entity', f'name: "{name}", type: MODEL'
    command = ['gz', 'service', '-s', f'/world/{world}/{service}', '--reqtype',
               f'gz.msgs.{request_type}', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', request]
    response = subprocess.check_output(command, text=True, timeout=10)
    (output / f'obstacle-{service}.json').write_text(json.dumps(dict(command=command, response=response)))
    if 'data: true' not in response:
        raise RuntimeError(f'Gazebo recusou {service} do obstáculo')


def groundtruth_yaw_ned(odometry):
    """Extrai quaternion do texto protobuf; não aceita orientação inválida."""
    match = re.search(r'\borientation\s*\{([^{}]*)\}', odometry)
    if match is None:
        raise ValueError('Odometry Gazebo sem orientação')
    values = {key: float(value) for key, value in re.findall(r'\b([xyzw]):\s*(\S+)', match[1])}
    x, y, z, w = (values.get(key, 0.) for key in 'xyzw')
    norm = x*x + y*y + z*z + w*w
    if not math.isfinite(norm) or abs(norm - 1.) > .01:
        raise ValueError('Quaternion Gazebo não normalizado ou não finito')
    yaw_enu = math.atan2(2.*(w*z + x*y), 1. - 2.*(y*y + z*z))
    return yaw_enu_to_ned(yaw_enu)


def check_frame_alignment(drone, output):
    """Compara orientação real/estimada antes de autorizar este ensaio vertical."""
    raw = subprocess.check_output(['gz', 'topic', '-e', '-t',
                                   '/model/x500_uerj_27/odometry', '-n', '1'],
                                  text=True, timeout=10)
    (output / 'groundtruth-before-arm.txt').write_text(raw)
    actual = groundtruth_yaw_ned(raw)
    measured = drone.state_px4.current_yaw_rad
    error = math.atan2(math.sin(measured - actual), math.cos(measured - actual))
    report = dict(groundtruth_yaw_ned_deg=math.degrees(actual),
                  estimated_yaw_ned_deg=math.degrees(measured), error_deg=math.degrees(error),
                  maximum_error_deg=5.)
    (output / 'frame-check.json').write_text(json.dumps(report, indent=2))
    if not math.isfinite(error) or abs(report['error_deg']) > report['maximum_error_deg']:
        raise RuntimeError(f'Yaw PX4 diverge do Gazebo em {report["error_deg"]:.2f} graus; ensaio não armado')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('px4-root', 'model-store', 'world-sdf', 'output'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    parser.add_argument('--agent', default='MicroXRCEAgent')
    parser.add_argument('--px4-binary', type=Path, help='Executável experimental; usa etc/rootfs de --px4-root')
    parser.add_argument('--server-config', type=Path, help='Configuração experimental de plugins Gazebo')
    parser.add_argument('--gcs-heartbeat', action='store_true',
                        help='GCS mínima local com pymavlink, somente heartbeat para a instância 27')
    parser.add_argument('--scenario', choices=('land', 'cruise', 'obstacle', 'rtl', 'mission'), default='land',
                        help='land: voo vertical; cruise: 40 m Norte, retorno e pouso, com LiDAR real')
    parser.add_argument('--preflight-only', action='store_true',
                        help='Encerra após conferir sensores/yaw, antes de OFFBOARD/ARM')
    parser.add_argument('--mission-image', type=Path, help='Imagem controlada para o cenário mission')
    parser.add_argument('--yaw-enu', type=float, default=0.,
                        help='Orientação de diagnóstico em radianos ENU, somente com --preflight-only')
    args = parser.parse_args()
    if args.scenario == 'mission' and (args.mission_image is None or not args.mission_image.is_file()):
        parser.error('mission exige --mission-image existente')
    if not math.isfinite(args.yaw_enu):
        parser.error('--yaw-enu deve ser finito')
    if args.yaw_enu != 0. and not args.preflight_only:
        parser.error('--yaw-enu requer --preflight-only')
    px4_root, store, world_sdf, output = (
        getattr(args, name).resolve() for name in ('px4_root', 'model_store', 'world_sdf', 'output'))
    world = ET.parse(world_sdf).find('world')
    if world is None or not world.get('name'):
        parser.error('--world-sdf deve conter um world com nome')
    world_name = world.get('name')
    build = px4_root / 'build/px4_sitl_default'
    binary, etc = build / 'bin/px4', build / 'etc'
    if args.px4_binary:
        binary = args.px4_binary.resolve()
    parameters = build / 'rootfs/parameters.bson'
    models, plugins, server = store / 'models', store / 'plugins', store / 'server.config'
    if args.server_config:
        server = args.server_config.resolve()
    for path in (binary, parameters, server, models / 'x500_uerj/model.sdf'):
        if not path.is_file():
            parser.error(f'Arquivo necessário ausente: {path}')
    if not etc.is_dir() or not plugins.is_dir():
        parser.error('Diretórios etc do PX4 e plugins do Gazebo devem existir')
    if output.exists():
        parser.error('--output deve ser um diretório novo')
    output.mkdir(parents=True)
    rootfs = output / 'rootfs'
    rootfs.mkdir()
    shutil.copy2(parameters, rootfs / 'parameters.bson')

    bridges = output / 'bridges.yaml'
    entries = (
        ('/clock', f'/world/{world_name}/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
        ('/drone_inspetor/externo/lidar/scan', '/drone_inspetor/gz/lidar_2d_v2',
         'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
        ('/drone_inspetor/externo/lidar_down/scan',
         f'/world/{world_name}/model/x500_uerj_27/link/lidar_sensor_link/sensor/lidar/scan',
         'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
    )
    bridges.write_text(json.dumps([
        dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_type,
             gz_type_name=gz_type, direction='GZ_TO_ROS')
        for ros, gz, ros_type, gz_type in entries], indent=2))  # JSON também é YAML.
    overrides = dict(
        ROS_DOMAIN_ID='173', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(output / 'ros-logs'), GZ_PARTITION=f'drone-validation-{os.getpid()}',
        GZ_IP='127.0.0.1', GZ_SIM_RESOURCE_PATH=str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(plugins), GZ_SIM_SERVER_CONFIG_PATH=str(server),
        PX4_SIM_MODEL='gz_x500_uerj', PX4_SYS_AUTOSTART='4030', PX4_GZ_STANDALONE='1',
        PX4_GZ_MODELS=str(models), PX4_GZ_WORLD=world_name, PX4_GZ_MODEL_POSE='-70,-27,57',
        PX4_UXRCE_DDS_PORT='18889', PX4_UXRCE_DDS_NS='px4_27', SIM_GZ_HOME_LAT='-22.633890',
        SIM_GZ_HOME_LON='-40.093330', SIM_GZ_HOME_ALT='0',
    )
    # Um nome herdado faria o PX4 conectar em outro veículo em vez de criar o seu.
    os.environ.pop('PX4_GZ_MODEL_NAME', None)
    os.environ.update(overrides)
    # rcS já usa domínio/porta/namespace do ambiente. Reiniciar DDS aqui pode
    # perder HOME publicado antes da recriação do writer.
    transport_commands = 'param set UXRCE_DDS_SYNCT 0\n'
    inputs = [world_sdf, server, parameters, binary, bridges, Path(__file__).resolve(),
              Path(__file__).with_name('validate_sitl.py').resolve(),
              *sorted(models.glob('*/model.sdf'))]
    if args.gcs_heartbeat:
        inputs.append(Path(__file__).with_name('simulation_gcs.py').resolve())
    if args.scenario == 'mission':
        inputs.extend([args.mission_image.resolve(), Path(__file__).with_name('validate_mission.py').resolve()])
    manifest = dict(world=world_name, model='x500_uerj', instance=27, system_id=28,
                    output=str(output), px4_root=str(px4_root), model_store=str(store),
                    environment=overrides, unset_environment=['PX4_GZ_MODEL_NAME'],
                    inputs=[fingerprint(path) for path in inputs], processes=[],
                    transport_overrides=transport_commands.splitlines(),
                    parameter_policy='Cópia do parameters.bson; origem preservada',
                    scenario='preflight' if args.preflight_only else args.scenario, synthetic_sensors=False)
    manifest_path = output / 'manifest.json'
    processes = []

    def save_manifest():
        manifest_path.write_text(json.dumps(manifest, indent=2))

    def spawn(name, command, *, stdin=None):
        with (output / f'{name}.log').open('w') as log:
            process = subprocess.Popen(command, cwd=rootfs, stdin=stdin, stdout=log,
                                       stderr=subprocess.STDOUT, text=True, start_new_session=True)
        processes.append(process)
        manifest['processes'].append(dict(name=name, pid=process.pid, command=command))
        save_manifest()
        return process

    def check_children():
        for process, info in zip(processes, manifest['processes']):
            if process.poll() is not None:
                raise RuntimeError(f'{info["name"]} encerrou: ver {output / (info["name"] + ".log")}')

    save_manifest()
    try:
        spawn('gazebo', ['nvidia-run', 'gz', 'sim', '-r', '-s', str(world_sdf)])
        spawn('agent', [args.agent, 'udp4', '-p', '18889'])
        if args.gcs_heartbeat:
            spawn('gcs', [sys.executable, str(Path(__file__).with_name('simulation_gcs.py').resolve())])
        spawn('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
                         '-p', f'config_file:={bridges}'])
        px4 = spawn('px4', [str(binary), str(etc), '-i', '27', '-w', str(rootfs)],
                    stdin=subprocess.PIPE)
        deadline = time.monotonic() + 60
        while True:
            check_children()
            if 'Startup script returned successfully' in (output / 'px4.log').read_text(errors='replace'):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f'PX4 não iniciou em 60 s: ver {output / "px4.log"}')
            time.sleep(.2)
        px4.stdin.write(transport_commands)
        px4.stdin.flush()
        if args.yaw_enu != 0.:
            # Este PX4 só lê XYZ de MODEL_POSE. Rotação explícita, apenas no diagnóstico desarmado.
            request = ('name: "x500_uerj_27", position: {x: -70, y: -27, z: 57}, '
                       f'orientation: {{z: {math.sin(args.yaw_enu/2)}, w: {math.cos(args.yaw_enu/2)}}}')
            command = ['gz', 'service', '-s', f'/world/{world_name}/set_pose',
                       '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                       '--timeout', '5000', '--req', request]
            response = subprocess.check_output(command, text=True, timeout=10)
            manifest['diagnostic_rotation'] = dict(command=command, response=response)
            save_manifest()
            if 'data: true' not in response:
                raise RuntimeError('Gazebo não confirmou a rotação de diagnóstico')
            time.sleep(3)
        check_children()
        run_flight(output, px4, scenario=args.scenario, synthetic_sensors=False,
                   prearm_check=lambda drone: check_frame_alignment(drone, output),
                   preflight_only=args.preflight_only,
                   obstacle_control=(lambda present: obstacle_service(world_name, output, present))
                   if args.scenario == 'obstacle' else None,
                   mission_image=args.mission_image.resolve() if args.mission_image else None)
        manifest['result'] = 'completed'
    except BaseException as error:
        manifest.update(result='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        for process in reversed(processes):
            try:
                stop_process(process)
            except (OSError, subprocess.SubprocessError) as error:
                manifest.setdefault('cleanup_errors', []).append(str(error))
        for process, info in zip(processes, manifest['processes']):
            info['returncode'] = process.poll()
        manifest['source_parameters_after'] = fingerprint(parameters)
        save_manifest()


if __name__ == '__main__':
    main()
