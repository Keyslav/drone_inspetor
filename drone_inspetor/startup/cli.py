"""Menu de terminal e comandos para iniciar os três modos da aplicação."""

import argparse
import json
import shlex
import sys
import time

from .controller import (
    MODE_SPECS, StartupController, environment_status, node_defaults,
)


_STATUS_LABELS = {
    'ros2': 'ROS 2',
    'package': 'Pacote ROS',
    'gazebo': 'Gazebo',
    'px4': 'PX4',
    'micro_xrce_agent': 'MicroXRCEAgent',
    'qgroundcontrol': 'QGroundControl',
    'bridges': 'Bridges',
    'clock': '/clock',
    'application': 'Aplicação',
}


def _bridge_option(value):
    return {'auto': None, 'on': True, 'off': False}[value]


def _clock_option(value):
    return {'auto': None, 'sim': True, 'real': False}[value]


def _print_status(*, json_output=False):
    status = environment_status()
    if json_output:
        print(json.dumps(status, ensure_ascii=False, indent=2))
        return
    for key, info in status.items():
        if info['available']:
            marker = '✓'
        elif info['state'] == 'unknown':
            marker = '?'
        else:
            marker = '·'
        print(f'{marker} {_STATUS_LABELS[key]:<17} {info["detail"]}')


def _print_logs(controller):
    for line in controller.recent_logs():
        print(line, flush=True)


def _interactive_menu(controller):
    choices = {'1': 'sim', '2': 'companion', '3': 'dashboard'}
    print('Inicializador Drone Inspetor')
    while True:
        controller.poll()
        active = controller.active_session()
        if active:
            print(f'\nModo ativo: {active["mode"]} (PID {active["pid"]})')
        else:
            print('\nNenhum modo ativo pelo inicializador.')
        for number, mode in choices.items():
            print(f'{number}. Iniciar {MODE_SPECS[mode].label}')
        print('4. Estado do ambiente')
        print('5. Logs recentes')
        print('6. Parar modo ativo')
        print('0. Sair (encerra o modo iniciado neste menu)')
        try:
            choice = input('Escolha: ').strip()
        except (KeyboardInterrupt, EOFError):
            print()
            choice = '0'
        if choice in choices:
            mode = choices[choice]
            try:
                command = controller.start(mode)
            except (RuntimeError, ValueError) as exc:
                print(f'Erro: {exc}', file=sys.stderr)
            else:
                print('Iniciado:', shlex.join(command))
        elif choice == '4':
            _print_status()
        elif choice == '5':
            _print_logs(controller)
        elif choice == '6':
            controller.stop_active(wait=True)
            controller.poll()
            _print_logs(controller)
        elif choice == '0':
            if controller.running:
                controller.stop(wait=True)
            controller.poll()
            _print_logs(controller)
            return 0
        else:
            print('Opção inválida.')


def _foreground_start(controller, args):
    command = controller.start(args.mode, bridges=_bridge_option(args.bridges),
                               use_sim_time=_clock_option(args.time),
                               params_file=args.params_file,
                               missions_file=args.missions_file,
                               bridges_file=args.bridges_file,
                               **{f'with_{node}': getattr(args, f'with_{node}')
                                  for node in node_defaults(args.mode)})
    print('Iniciado:', shlex.join(command), flush=True)
    try:
        while True:
            _print_logs(controller)
            result = controller.poll()
            if result is not None:
                _print_logs(controller)
                return result
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nEncerrando modo...', flush=True)
        controller.stop(wait=True)
        controller.poll()
        _print_logs(controller)
        return 0


def main(argv=None):
    """Executa menu interativo ou comandos para scripts e terminais."""
    parser = argparse.ArgumentParser(
        prog='drone_inspetor_start',
        description='Inicializador Drone Inspetor')
    commands = parser.add_subparsers(dest='command')
    commands.add_parser('menu', help='Abrir menu interativo')
    start = commands.add_parser(
        'start', help='Iniciar um modo no terminal atual')
    start.add_argument('mode', choices=MODE_SPECS)
    start.add_argument(
        '--bridges', choices=('auto', 'on', 'off'), default='auto',
        help='auto detecta bridges; on inicia; off usa existentes')
    start.add_argument(
        '--time', choices=('auto', 'sim', 'real'), default='auto',
        help='relógio ROS: automático, simulado ou real')
    for name, description in (
            ('params', 'YAML de parâmetros dos nós'),
            ('missions', 'catálogo JSON de missões'),
            ('bridges', 'YAML de tópicos Gazebo–ROS')):
        start.add_argument(f'--{name}-file', metavar='ARQUIVO', help=description)
    for node in node_defaults('sim'):
        start.add_argument(
            f'--with-{node}', action=argparse.BooleanOptionalAction,
            default=None,
            help=f'habilitar/desabilitar {node}_node (padrão do perfil)')
    status = commands.add_parser(
        'status', help='Mostrar estado de infraestrutura')
    status.add_argument('--json', action='store_true', help='Saída JSON')
    commands.add_parser(
        'stop', help='Parar uma sessão iniciada pelo menu ou GUI')
    args = parser.parse_args(argv)

    if args.command in (None, 'menu') and not sys.stdin.isatty():
        parser.print_help(sys.stderr)
        return 2
    controller = StartupController()
    try:
        if args.command in (None, 'menu'):
            return _interactive_menu(controller)
        if args.command == 'status':
            _print_status(json_output=args.json)
            return 0
        if args.command == 'stop':
            active = controller.active_session()
            if not active:
                print('Nenhuma sessão gerenciada está ativa.')
                return 0
            controller.stop_active(wait=True)
            print(f'Sessão {active["mode"]} encerrada.')
            return 0
        return _foreground_start(controller, args)
    except (RuntimeError, ValueError) as exc:
        print(f'Erro: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
