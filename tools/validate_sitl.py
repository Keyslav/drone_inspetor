#!/usr/bin/env python3
"""Valida voo no PX4 SIH local, com sensores sintéticos e DDS isolado.

Este programa sempre cria sua própria instância do simulador. Não deve ser usado
como cliente de uma aeronave. Os obstáculos são raios sintéticos, não colisões
físicas do Gazebo. A dinâmica e as malhas de controle são do PX4 SIH.
"""

import argparse
from dataclasses import asdict
import faulthandler
from functools import wraps
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


class CallbackTimings:
    """Instrumentação do ensaio: tempo de parede/CPU e pausas entre callbacks."""

    def __init__(self):
        self.stats = {}
        self.lock = threading.Lock()
        self.last_control_start = None

    def wrap(self, callback):
        name = callback.__qualname__

        @wraps(callback)
        def measured(*args, **kwargs):
            start, cpu_start = time.monotonic(), time.thread_time()
            if callback.__name__ == 'tick_deslocamento_e_publish_setpoint':
                self.last_control_start = start
            try:
                return callback(*args, **kwargs)
            finally:
                wall, cpu = time.monotonic() - start, time.thread_time() - cpu_start
                with self.lock:
                    stat = self.stats.setdefault(name, {
                        'count': 0, 'max_wall_s': 0., 'max_cpu_s': 0., 'max_start_gap_s': 0.,
                        'last_start': start, 'slow_calls': [],
                    })
                    stat['count'] += 1
                    stat['max_wall_s'] = max(stat['max_wall_s'], wall)
                    stat['max_cpu_s'] = max(stat['max_cpu_s'], cpu)
                    stat['max_start_gap_s'] = max(stat['max_start_gap_s'], start - stat['last_start'])
                    stat['last_start'] = start
                    if wall > .1 and len(stat['slow_calls']) < 20:
                        stat['slow_calls'].append({'started': start, 'wall_s': wall, 'cpu_s': cpu})

        return measured


def flight_metrics(data, cruise):
    """Resume seguimento e cruzeiro contínuo; lacunas não contam como evidência."""
    metrics = []
    for command in data['commands']:
        if 'started' not in command:
            continue
        samples = [sample for sample in data['samples']
                   if command['started'] <= sample['time'] <= command['finished']]
        if not samples:
            continue
        longest = current = 0.
        previous = None
        speeds, errors, reference_speeds = [], [], []
        for sample in samples:
            speed = math.sqrt(sum(value ** 2 for value in sample['velocity']))
            speeds.append(speed)
            # Em AUTO, o PX4 produz sua própria referência. O último setpoint
            # ROS não mede erro de seguimento do pouso/retorno nativo.
            active = sample.get('reference_active', command['command'] not in ('LAND', 'RTL'))
            if active:
                errors.append(math.dist(sample['position'], sample['reference']))
                reference_speeds.append(abs(sample['reference_velocity']))
            at_cruise = (active and sample['phase'] == 'DESLOCANDO'
                         and abs(speed - cruise) <= .1 * cruise
                         and abs(sample['reference_velocity'] - cruise) <= .1 * cruise)
            if at_cruise:
                gap = sample['time'] - previous if previous is not None else 0.
                current = current + gap if gap <= .15 else 0.
                previous = sample['time']
                longest = max(longest, current)
            else:
                current, previous = 0., None
        clearances = [math.hypot(sample['position'][0] - x, sample['position'][1] - y) - .4
                      for sample in samples for x, y in sample['obstacles']]
        metrics.append({
            'command': command['command'], 'duration_s': command['finished'] - command['started'],
            'peak_measured_speed_m_s': max(speeds),
            'peak_reference_speed_m_s': max(reference_speeds, default=None),
            'maximum_tracking_error_m': max(errors, default=None),
            'continuous_cruise_s': longest,
            'minimum_obstacle_surface_distance_m': min(clearances) if clearances else None,
        })
    return metrics


def record_command(data, command, parameters, execute):
    """Preserva o intervalo de comandos que falham, inclusive por transporte."""
    entry = {'command': command, 'parameters': parameters, 'started': time.monotonic()}
    data['commands'].append(entry)
    try:
        result = execute()
        entry.update(success=result.success, message=result.message)
        return result
    except Exception as error:
        entry.update(success=False, message=str(error), error_type=type(error).__name__)
        raise
    finally:
        entry['finished'] = time.monotonic()


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)


def run_flight(output, px4_process, deceleration=None, diagnose_timing=False, scenario='full',
               synthetic_sensors=True, prearm_check=None, preflight_only=False, obstacle_control=None,
               mission_image=None):
    if not synthetic_sensors and (scenario not in ('land', 'cruise', 'obstacle', 'rtl', 'mission')
                                  or (scenario == 'obstacle' and obstacle_control is None)):
        raise ValueError('Cenário externo inválido ou sem controle do obstáculo Gazebo')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from px4_msgs.msg import VehicleStatus
    from drone_inspetor_msgs.action import DroneCommand
    from drone_inspetor.nodes.drone_node.drone_node import DroneNode
    from drone_inspetor.nodes.drone_node.fsm.drone.description import DroneFSMDescription as DS
    from drone_inspetor.ros_interfaces import Topics, create_publisher_from

    remaps = []
    for name in vars(Topics.PX4).values():
        if hasattr(name, 'name'):
            remaps.extend(['-r', f'{name.name}:=/px4_27{name.name}'])
    parameters = ['-p', 'fsm_timer_period:=0.1', '-p', 'px4_target_system_id:=28']
    if not synthetic_sensors:
        parameters.extend(['-p', 'use_sim_time:=true'])
    if deceleration is not None:
        parameters.extend(['-p', f'obstacle_deceleration:={deceleration}'])
    rclpy.init(args=['--ros-args', *remaps, *parameters])
    timings = CallbackTimings()

    class TimedDroneNode(DroneNode):
        def create_timer(self, period, callback, *args, **kwargs):
            return super().create_timer(period, timings.wrap(callback), *args, **kwargs)

        def create_subscription(self, msg_type, topic, callback, qos, *args, **kwargs):
            return super().create_subscription(msg_type, topic, timings.wrap(callback), qos, *args, **kwargs)

    drone = TimedDroneNode() if diagnose_timing else DroneNode()
    client_node = Node('sitl_validation')
    client = ActionClient(client_node, DroneCommand, Topics.Action.DRONE_COMMAND.name)
    lidar = create_publisher_from(client_node, Topics.Externo.LIDAR_SCAN) if synthetic_sensors else None
    down = create_publisher_from(client_node, Topics.Externo.LIDAR_DOWN_SCAN) if synthetic_sensors else None
    data = {'scenario': scenario, 'sensor_source': 'synthetic' if synthetic_sensors else 'gazebo',
            'samples': [], 'commands': [], 'obstacles': [], 'preflight_only': preflight_only,
            'parameters': {'cruise_velocity': drone.param_cruise_velocity,
                           'travel_acceleration': drone.param_travel_acceleration,
                           'obstacle_deceleration': drone.param_obstacle_deceleration,
                           'navigation': asdict(drone.navigation_config)}}
    circles = []
    extra_nodes = []
    running = True

    def publish_sensors():
        with drone._control_lock:
            position = drone.state_px4.local_position
            if position is None:
                return
            xyz = (position.x, position.y, position.z)
            yaw = drone.state_px4.current_yaw_rad
            reference = drone.trajectory.reference_position
            data['samples'].append({
                'time': time.monotonic(), 'position': xyz,
                'velocity': drone.trajectory.measured_velocity,
                'reference': reference,
                'reference_active': drone.state_px4.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                'reference_velocity': drone.trajectory_profile.velocity,
                'reference_acceleration': drone.trajectory_profile.accel,
                'tick_elapsed': drone.trajectory.last_tick_elapsed,
                'yaw': drone.state_px4.current_yaw_rad,
                'reference_yaw': drone.trajectory._yaw_reference,
                'state': drone.drone_fsm_context.state.name,
                'phase': drone.deslocamento_fsm_context.state.name,
                'detours': drone.trajectory._detours,
                'error': drone.trajectory.navigation_error,
                'decision': (asdict(drone.trajectory.last_decision)
                             if drone.trajectory.last_decision is not None else None),
                'obstacles': list(circles),
            })
        if not synthetic_sensors:
            return
        msg = LaserScan()
        msg.header.stamp = client_node.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic_lidar_flu'
        msg.angle_min = -3 * math.pi / 4
        msg.angle_increment = (3 * math.pi / 2) / 1079
        msg.angle_max = msg.angle_min + 1079 * msg.angle_increment
        msg.range_min, msg.range_max = .1, 30.
        ranges = []
        for index in range(1080):
            bearing = yaw - (msg.angle_min + index * msg.angle_increment)
            ux, uy = math.cos(bearing), math.sin(bearing)
            distance = math.inf
            for x, y in circles:
                dx, dy = x - xyz[0], y - xyz[1]
                along, lateral = dx * ux + dy * uy, abs(dx * uy - dy * ux)
                if along > 0 and lateral < .4:
                    distance = min(distance, max(.1, along - math.sqrt(.4 ** 2 - lateral ** 2)))
            ranges.append(distance)
        msg.ranges = ranges
        lidar.publish(msg)
        lower = LaserScan()
        lower.header = msg.header
        lower.angle_min = lower.angle_max = 0.
        lower.angle_increment = .01
        lower.range_min, lower.range_max = .1, 50.
        home = drone.state_px4.home_local_position
        lower.ranges = [max(.1, (home[2] if home else 0.) - xyz[2])]
        down.publish(lower)

    client_node.create_timer(.05, publish_sensors)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(drone)
    executor.add_node(client_node)

    def spin():
        while running and rclpy.ok():
            executor.spin_once(timeout_sec=.1)

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    def watch_timing():
        # Só no ensaio diagnosticado. Captura pilhas enquanto a pausa acontece,
        # sem registrar valores locais nem mudar os limites de reação do controle.
        captures, previous = 0, 0.
        with (output / 'executor-stacks.txt').open('w') as stream:
            while running:
                now, last = time.monotonic(), timings.last_control_start
                if last is not None and now - last > .25 and now - previous > .5 and captures < 10:
                    stream.write(f'\nmonotonic={now:.6f} control_gap={now - last:.6f}\n')
                    stream.flush()
                    faulthandler.dump_traceback(file=stream, all_threads=True)
                    previous, captures = now, captures + 1
                time.sleep(.05)

    watcher = threading.Thread(target=watch_timing, daemon=True) if diagnose_timing else None
    if watcher:
        watcher.start()

    def until(predicate, seconds, message):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if px4_process.poll() is not None:
                raise RuntimeError('PX4 SITL encerrou antes do cenário')
            if predicate():
                return
            time.sleep(.02)
        raise TimeoutError(message)

    def future_result(future, seconds=15):
        until(future.done, seconds, 'Prazo de resposta ROS excedido')
        return future.result()

    def goal(command, **values):
        request = DroneCommand.Goal()
        request.command = command
        for field in ('altitude', 'lat', 'lon', 'alt', 'yaw', 'focus_lat', 'focus_lon'):
            setattr(request, field, math.nan)
        for key, value in values.items():
            setattr(request, key, value)
        handle = future_result(client.send_goal_async(request))
        if not handle.accepted:
            raise RuntimeError(f'{command} rejeitado pelo DroneNode')
        return handle

    def complete(command, **values):
        print('COMMAND', command, values, flush=True)
        # O servidor pode frear após expirar o prazo de execução. O cliente deve
        # observar essa conclusão, sem ampliar o tempo de movimento autorizado.
        seconds = drone.command_timeouts[command] + drone.command_cancel_timeout + 10.

        def execute():
            handle = goal(command, **values)
            return future_result(handle.get_result_async(), seconds).result

        result = record_command(data, command, values, execute)
        print('RESULT', data['commands'][-1], flush=True)
        if not result.success:
            raise RuntimeError(result.message)

    try:
        until(lambda: drone.telemetry_fresh() and drone.state_px4.home_local_position is not None,
              30, 'Telemetria/HOME não recebido por DDS')
        print('TELEMETRY', drone.state_px4.local_position.x, flush=True)
        if not synthetic_sensors:
            until(lambda: drone.navigation_sensors.map.fresh(time.monotonic())
                  and drone.navigation_sensors.down_received is not None
                  and time.monotonic() - drone.navigation_sensors.down_received <
                  drone.navigation_config.sensor_timeout,
                  15, 'LiDARs reais não ficaram válidos no relógio simulado')
        time.sleep(2)
        if prearm_check is not None:
            data['preflight'] = prearm_check(drone)
        if preflight_only:
            data['success'] = True
            print('PREFLIGHT_PASS (sem ARM ou voo)', flush=True)
            return
        with drone._control_lock:
            drone.set_offboard_mode()
        until(lambda: drone.drone_fsm_context.state == DS.POUSADO_DESARMADO,
              15, 'OFFBOARD não confirmado')
        if scenario == 'mission':
            from validate_mission import run_mission
            data['mission'] = run_mission(output, drone, client_node, executor, until,
                                          mission_image, extra_nodes)
            data['success'] = True
            print('FLIGHT_PASS mission', flush=True)
            return
        complete('ARM')
        complete('TAKEOFF', altitude=3.)
        if scenario == 'land':
            complete('LAND')
            print('SITL_PASS land', flush=True)
            data['success'] = True
            return
        home = drone.state_px4
        latitude, longitude, altitude = home.home_global_lat, home.home_global_lon, home.home_global_alt + 3.
        target_lat = latitude + math.degrees(12. / 6371000.)
        if scenario == 'rtl':
            complete('GOTO', lat=target_lat, lon=longitude, alt=altitude)
            complete('RTL')
            if drone.state_px4.is_armed:
                raise RuntimeError('RTL terminou sem desarmar')
            data['success'] = True
            print('FLIGHT_PASS rtl', flush=True)
            return
        # Trecho longo para observar cruzeiro além da aceleração e da frenagem.
        cruise_lat = latitude + math.degrees(40. / 6371000.)
        if scenario in ('full', 'cruise'):
            complete('GOTO', lat=cruise_lat, lon=longitude, alt=altitude)
            complete('GOTO', lat=latitude, lon=longitude, alt=altitude)
        if scenario == 'cruise':
            complete('LAND')
            metrics = flight_metrics(data, drone.param_cruise_velocity)
            if any(item['continuous_cruise_s'] < 2. for item in metrics if item['command'] == 'GOTO'):
                raise RuntimeError('Ida/volta não manteve cruzeiro por 2s em cada trecho (tolerância 10%)')
            data['success'] = True
            print('FLIGHT_PASS cruise', flush=True)
            return
        circles.append((home.home_local_position[0] + 6., home.home_local_position[1]))
        data['obstacles'] = list(circles)
        if obstacle_control is not None:
            obstacle_control(True)
            time.sleep(1)
        complete('GOTO', lat=target_lat, lon=longitude, alt=altitude)
        if obstacle_control is not None:
            if not any(sample['detours'] > 0 for sample in data['samples']):
                raise RuntimeError('Obstáculo Gazebo não produziu desvio observado')
            obstacle_control(False)
        circles.clear()
        handle = goal('GOTO', lat=target_lat + math.degrees(30. / 6371000.),
                      lon=longitude, alt=altitude)
        until(lambda: drone.trajectory_profile.velocity > 1., 20, 'GOTO não acelerou')
        cancellation = future_result(handle.cancel_goal_async())
        if not cancellation.goals_canceling:
            raise RuntimeError('Cancelamento não aceito')
        result = future_result(handle.get_result_async(), 35).result
        if result.success or not drone.trajectory.stopped:
            raise RuntimeError('Cancelamento não confirmou parada')
        data['commands'].append({'command': 'CANCEL_GOTO', 'success': True})
        if not synthetic_sensors:
            # A plataforma não tem piso em toda a rota. Pousar na área já verificada do HOME.
            complete('GOTO', lat=latitude, lon=longitude, alt=altitude)
        complete('LAND')
        metrics = flight_metrics(data, drone.param_cruise_velocity)
        if scenario == 'full':
            first_goto = next(item for item in metrics if item['command'] == 'GOTO')
            if first_goto['continuous_cruise_s'] < 2.:
                raise RuntimeError('Trecho livre não manteve cruzeiro por 2s (tolerância 10%)')
        minimum_clearance = drone.navigation_config.vehicle_radius + drone.navigation_config.obstacle_margin
        if any(item['minimum_obstacle_surface_distance_m'] is not None
               and item['minimum_obstacle_surface_distance_m'] < minimum_clearance for item in metrics):
            raise RuntimeError('Percurso invadiu margem geométrica do obstáculo sintético')
        print('SITL_PASS', scenario, flush=True)
        data['success'] = True
    except Exception as error:
        data['success'] = False
        data['error'] = str(error)
        data['graph'] = client_node.get_topic_names_and_types()
        data['subscriptions'] = [subscription.topic_name for subscription in drone.subscriptions]
        print('SITL_FAIL', error, flush=True)
        raise
    finally:
        data['metrics'] = flight_metrics(data, drone.param_cruise_velocity)
        (output / 'flight.json').write_text(json.dumps(data, indent=2))
        print('METRICS', json.dumps(data['metrics']), flush=True)
        running = False
        spinner.join(timeout=2)
        if watcher:
            watcher.join(timeout=1)
        drone._is_drone_node_shutting_down = True
        executor.shutdown(timeout_sec=2)
        for node in reversed(extra_nodes):
            node.destroy_node()
        if diagnose_timing:
            with timings.lock:
                (output / 'callback-timing.json').write_text(json.dumps(timings.stats, indent=2))
        client.destroy()
        drone.destroy_node()
        client_node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root', type=Path, required=True)
    parser.add_argument('--agent', default='MicroXRCEAgent')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--deceleration', type=float,
                        help='Override experimental ROS, em m/s²; não altera arquivos do PX4')
    parser.add_argument('--diagnose-timing', action='store_true',
                        help='Registra duração/CPU dos callbacks e pilhas durante pausas do executor')
    parser.add_argument('--scenario', choices=('full', 'obstacle', 'land'), default='full',
                        help='full: cenário completo; obstacle: desvio/cancelamento/LAND; land: TAKEOFF/LAND')
    args = parser.parse_args()
    if args.deceleration is not None and (not math.isfinite(args.deceleration) or args.deceleration <= 0):
        parser.error('--deceleration deve ser finita e positiva')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    rootfs = output / 'rootfs'
    rootfs.mkdir(exist_ok=True)
    # Instância não padrão, namespace px4_27, portas locais e domínio exclusivo.
    os.environ.update(ROS_DOMAIN_ID='173', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      PX4_SIM_MODEL='sihsim_quadx', PX4_UXRCE_DDS_PORT='18889', ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(output / 'ros-logs'))
    processes = []
    try:
        with (output / 'agent.log').open('w') as log:
            processes.append(subprocess.Popen([args.agent, 'udp4', '-p', '18889'],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        binary = args.px4_root / 'build/px4_sitl_default/bin/px4'
        etc = args.px4_root / 'build/px4_sitl_default/etc'
        with (output / 'px4.log').open('w') as log:
            px4 = subprocess.Popen([str(binary), str(etc), '-i', '27', '-w', str(rootfs)],
                                   stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
            processes.append(px4)
        time.sleep(10)
        if px4.poll() is not None:
            raise RuntimeError(f'PX4 SITL não iniciou: ver {output / "px4.log"}')
        px4.stdin.write('param set UXRCE_DDS_PTCFG 1\nuxrce_dds_client stop\nuxrce_dds_client start -t udp -p 18889 -n px4_27\n')
        px4.stdin.flush()
        run_flight(output, px4, args.deceleration, args.diagnose_timing, args.scenario)
    finally:
        for process in reversed(processes):
            stop_process(process)


if __name__ == '__main__':
    main()
