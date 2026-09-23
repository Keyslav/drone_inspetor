"""Instalação única via ament_python; recursos ficam no diretório share."""

from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


PACKAGE = 'drone_inspetor'


def resource_files():
    """Preserva subdiretórios dos recursos, sem copiar módulos Python para share."""
    resources = [
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE}']),
        (f'share/{PACKAGE}', ['package.xml', 'README.md', 'INIT_SIMULACAO.md']),
        (f'share/{PACKAGE}/docs', glob('docs/*.md')),
        (f'share/{PACKAGE}/launch', glob(f'{PACKAGE}/launch/*_launch.py')),
    ]
    suffixes = {'.yaml', '.json', '.html', '.js', '.css', '.png', '.jpeg', '.jpg', '.sdf', '.pt'}
    for directory in ('config', 'models', 'assets', 'missions', 'redes_treinadas', 'gui'):
        paths_by_parent = {}
        for path in sorted((Path(PACKAGE) / directory).rglob('*')):
            if path.is_file() and path.suffix in suffixes:
                paths_by_parent.setdefault(path.parent, []).append(str(path))
        for parent, files in paths_by_parent.items():
            resources.append((str(Path('share') / parent), files))
    return resources


setup(
    name=PACKAGE,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=resource_files(),
    install_requires=[
        'setuptools', 'numpy>=1.26,<2', 'PyYAML', 'Pillow', 'opencv-python>=4.8,<4.12',
        'PyQt6', 'PyQt6-WebEngine', 'ultralytics==8.3.206',
        'torch==2.8.0', 'torchvision==0.23.0', 'ruckig==0.19.4',
    ],
    zip_safe=False,
    maintainer='Keyslav',
    maintainer_email='user@todo.todo',
    description='Controle ROS 2/PX4, percepção e dashboard para inspeção com drone.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'{node}_node = {PACKAGE}.nodes.{node}_node.{node}_node:main'
            for node in ('dashboard', 'camera', 'cv', 'depth', 'lidar', 'drone', 'mission', 'monitor')
        ] + [f'teste_drone_node = {PACKAGE}.scripts.teste_drone_node:main'],
    },
)
