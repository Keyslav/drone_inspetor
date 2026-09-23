"""Launch declarativo com seleção de nós, tempo e bridges por argumentos."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


APPLICATION_NODES = ('camera', 'cv', 'depth', 'lidar', 'drone', 'mission', 'dashboard')


def create_launch(*, simulation=False, bridges=False, application=True):
    """Cria um contexto; argumentos substituem defaults sem editar os launchers."""
    share = get_package_share_directory('drone_inspetor')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value=str(simulation).lower()),
        DeclareLaunchArgument('bridges', default_value=str(bridges).lower()),
        DeclareLaunchArgument(
            'bridges_file',
            default_value=PathJoinSubstitution([share, 'config', 'ros_gz_bridges.yaml']),
            description='Arquivo YAML de bridges ROS/Gazebo para o mundo e a instância.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'param_ros.yaml']),
            description='Arquivo YAML aplicado a todos os nós da aplicação.',
        ),
        DeclareLaunchArgument(
            'missions_file', default_value='missions.json',
            description='Catálogo absoluto, ou relativo a share/drone_inspetor/missions.',
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]
    nodes = []
    for component in APPLICATION_NODES:
        option = f'with_{component}'
        declarations.append(DeclareLaunchArgument(
            option, default_value=str(application).lower(),
            description=f'Iniciar {component}_node.',
        ))
        extra = {}
        if component == 'dashboard':
            extra['on_exit'] = Shutdown(reason='Dashboard encerrado')
        nodes.append(Node(
            package='drone_inspetor', executable=f'{component}_node',
            name=f'{component}_node', output='screen', emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': use_sim_time,
                'missions_file': LaunchConfiguration('missions_file'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=IfCondition(LaunchConfiguration(option)), **extra,
        ))

    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('bridges')),
        parameters=[{
            'config_file': LaunchConfiguration('bridges_file'),
            'use_sim_time': use_sim_time,
        }],
    ))
    image_topics = (
        ('/drone_inspetor/gz/gimbal/camera', '/drone_inspetor/externo/camera'),
        ('/drone_inspetor/gz/depth_camera', '/drone_inspetor/externo/depth_camera'),
    )
    remappings = []
    for source, destination in image_topics:
        remappings.append((source, f'{destination}/image_raw'))
        remappings.extend(
            (f'{source}/{transport}', f'{destination}/{transport}')
            for transport in ('compressed', 'compressedDepth', 'theora', 'zstd')
        )
    nodes.append(Node(
        package='ros_gz_image', executable='image_bridge', name='image_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('bridges')),
        parameters=[{'use_sim_time': use_sim_time, 'qos': 'sensor_data'}],
        arguments=[topic for topic, _ in image_topics], remappings=remappings,
    ))
    return LaunchDescription([*declarations, *nodes])
