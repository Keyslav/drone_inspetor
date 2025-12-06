from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_inspetor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('drone_inspetor', 'launch', '*_launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('drone_inspetor', 'config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('drone_inspetor', 'models', '*.sdf'))),
        (os.path.join('share', package_name, 'assets'), glob(os.path.join('drone_inspetor', 'assets', '*.jpeg')) + glob(os.path.join('drone_inspetor', 'assets', '*.png'))),
        (os.path.join('share', package_name, 'assets', 'icons'), glob(os.path.join('drone_inspetor', 'assets', 'icons', '*.png'))),
        # Adicionando os novos diretórios gui e nodes
        (os.path.join('share', package_name, 'gui'), glob(os.path.join('drone_inspetor', 'gui', '*.py'))),
        (os.path.join('share', package_name, 'gui'), glob(os.path.join('drone_inspetor', 'gui', '*.html'))),
        (os.path.join('share', package_name, 'nodes'), glob(os.path.join('drone_inspetor', 'nodes', '*.py'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('drone_inspetor', 'scripts', '*.py'))),
        # Copiar modelo YOLO best.pt para o diretório de instalação
        (os.path.join('share', package_name), ['best.pt']),
    ],
    install_requires=[
        'setuptools',
        'PyQt6',
        'PyQt6-WebEngine',
        'opencv-python',
        'numpy',
        'pillow',
        'pyyaml',
        'psutil' # Adicionado para GazeboChecker
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Pacote ROS2 para o dashboard de monitoramento do drone, com arquitetura modular e integração de controles, FSM e mapa.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = drone_inspetor.nodes.dashboard_node:main',
            'camera_node = drone_inspetor.nodes.camera_node:main',
            'cv_node = drone_inspetor.nodes.cv_node:main',
            'depth_node = drone_inspetor.nodes.depth_node:main',
            'lidar_node = drone_inspetor.nodes.lidar_node:main',
            'drone_node = drone_inspetor.nodes.drone_node:main',
            'fsm_node = drone_inspetor.nodes.fsm_node:main',
            'offboard_control = drone_inspetor.scripts.offboard_control:main',
            # 'mission_control = drone_inspetor.scripts.mission_control:main',
        ],
    },
)

