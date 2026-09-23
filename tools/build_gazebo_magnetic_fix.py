#!/usr/bin/env python3
"""Cria PX4 corrigido e server.config isolados, reaproveitando build SITL existente.

Compatibilidade ensaiada: Gazebo 8.15/gz-sensors 8.2.2 e bridge legado descrito
em docs/VALIDACAO_GAZEBO.md. Não modifica fontes nem build original do PX4.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('px4-root', 'server-config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    root, server, out = (p.resolve() for p in (args.px4_root, args.server_config, args.output))
    if out.exists():
        parser.error('--output deve ser uma pasta nova')
    build = root / 'build/px4_sitl_default'
    src = root / 'src/modules/simulation/gz_bridge/GZBridge.cpp'
    original = src.read_text()
    old = ('report.x = -msg.field_tesla().y();\n\treport.y = -msg.field_tesla().x();'
           '\n\treport.z = msg.field_tesla().z();')
    new = ('report.x = msg.field_tesla().x();\n\treport.y = -msg.field_tesla().y();'
           '\n\treport.z = -msg.field_tesla().z();')
    if original.count(old) != 1:
        parser.error('Bridge não corresponde à versão legada validada; não aplicar automaticamente')
    tree = ET.parse(server)
    plugins = tree.findall('.//plugin[@name="gz::sim::systems::Magnetometer"]')
    if len(plugins) != 1:
        parser.error('Esperado exatamente um plugin Magnetometer')
    for key, value in (('use_units_gauss', 'true'), ('use_earth_frame_ned', 'false')):
        element = plugins[0].find(key)
        if element is None:
            element = ET.SubElement(plugins[0], key)
        element.text = value
    entry = next(x for x in json.loads((build / 'compile_commands.json').read_text())
                 if x['file'] == str(src))
    out.mkdir(parents=True)
    (out / 'GZBridge.cpp').write_text(original.replace(old, new))
    commands = []

    def run(command):
        commands.append(command)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2))
        subprocess.run(command, cwd=build, check=True)

    command = shlex.split(entry['command'])
    command[command.index('-o') + 1] = str(out / 'GZBridge.cpp.o')
    command[command.index('-c') + 1] = str(out / 'GZBridge.cpp')
    run(command + ['-I' + str(src.parent)])
    library = 'src/modules/simulation/gz_bridge/libmodules__simulation__gz_bridge.a'
    patched_library = out / Path(library).name
    shutil.copy2(build / library, patched_library)
    run(['ar', 'r', str(patched_library), str(out / 'GZBridge.cpp.o')])
    lines = subprocess.check_output(['ninja', '-t', 'commands', 'px4'], cwd=build, text=True)
    link = next(line for line in lines.splitlines() if ' -o bin/px4 ' in line)
    command = shlex.split(link.split('&&')[1])
    command[command.index('-o') + 1] = str(out / 'px4')
    command[command.index(library)] = str(patched_library)
    # CMake pode citar versão removida pelo apt; manter biblioteca pelo link .so.
    for i, value in enumerate(command):
        if value.startswith('/') and '.so.' in value and not Path(value).exists():
            replacement = Path(value.split('.so.')[0] + '.so')
            if not replacement.exists():
                raise FileNotFoundError(value)
            command[i] = str(replacement)
    run(command)
    tree.write(out / 'server.config', encoding='unicode')
    shutil.copy2(build / 'bin/px4-alias.sh', out / 'px4-alias.sh')
    for helper in (build / 'bin').glob('px4-*'):
        if helper.is_symlink():
            (out / helper.name).symlink_to('px4')
    paths = (src, server, build / library, out / 'px4', out / 'server.config')
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (out / 'hashes.json').write_text(json.dumps(hashes, indent=2))
    print(f'Cópia pronta: {out}; usar executável e server.config juntos.')


if __name__ == '__main__':
    main()
