"""Integração DDS com produtores sintéticos; exige domínio ROS reservado ao teste.

Executar com DRONE_TEST_ISOLATED_ROS=1 e ROS_DOMAIN_ID=187. Nenhum nó de voo,
simulador, action de drone ou publicador /fmu/in é criado por este teste.
"""

import os
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('DRONE_TEST_ISOLATED_ROS') != '1' or
    os.environ.get('ROS_DOMAIN_ID') != '187',
    reason='Requer domínio DDS 187 reservado para integração sintética',
)


def test_real_ros_adapter_receives_frames_and_does_not_replay_mission(tmp_path):
    """Percorre interfaces reais e verifica entrega única também ao reconectar."""
    import cv2
    import numpy as np
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from drone_inspetor_msgs.msg import DroneStateMSG, MissionStateMSG
    from drone_inspetor_msgs.srv import CVModelsSRV
    from px4_msgs.msg import VehicleStatus
    from drone_inspetor.mobile_gateway.api import MobileAPI
    from drone_inspetor.mobile_gateway.ros_adapter import ROSAdapter
    from drone_inspetor.mobile_gateway.state import MobileState
    from drone_inspetor.ros_interfaces import Topics, create_publisher_from, create_subscription_from

    state = MobileState()
    adapter = ROSAdapter(state, media_dir=tmp_path, ros_args=[])
    producer = Node('mobile_synthetic_test')
    adapter.executor.add_node(producer)
    try:
        drone_pub = create_publisher_from(producer, Topics.Interno.DRONE_STATE)
        mission_pub = create_publisher_from(producer, Topics.Interno.MISSION_STATE)
        status_pub = create_publisher_from(producer, Topics.PX4.VEHICLE_STATUS)
        camera_pub = create_publisher_from(producer, Topics.Interno.CAMERA_COMPRESSED)
        received = []
        subscription = create_subscription_from(
            producer, Topics.Dashboard.MISSION_COMMANDS,
            lambda msg: received.append((msg.command, msg.mission)))

        def models(_request, response):
            response.models_data_json = '[]'
            response.current_object_model = ''
            response.current_anomaly_model = ''
            return response

        producer.create_service(CVModelsSRV, Topics.Service.CV_LIST_MODELS.name, models)
        drone, mission, status = DroneStateMSG(), MissionStateMSG(), VehicleStatus()
        mission.state_name = 'PRONTO'
        frame = CompressedImage()
        frame.format = 'jpeg'
        frame.data = cv2.imencode('.jpg', np.zeros((24, 32, 3), dtype=np.uint8))[1].tobytes()

        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            drone_pub.publish(drone)
            mission_pub.publish(mission)
            status_pub.publish(status)
            camera_pub.publish(frame)
            time.sleep(0.05)
            topics = state.snapshot()['topics']
            if (all(topics[key]['health'] == 'live' for key in ('drone', 'mission', 'status'))
                    and state.frame('camera') and adapter.mission_pub.get_subscription_count()
                    and adapter.model_client.service_is_ready()):
                break
        assert state.frame('camera').startswith(b'\xff\xd8')
        assert adapter.models()['models'] == []
        api = MobileAPI(state, adapter, {'Flare': {}}, enable_commands=True)
        command = {'id': 'synthetic_001', 'command': 'mission.start',
                   'args': {'mission': 'Flare'}, 'confirmed': True,
                   'nonce': api.snapshot()['command_nonce']}
        assert api.execute(command)['status'] == 'submitted'
        assert api.execute(command)['status'] == 'submitted'
        deadline = time.monotonic() + 2
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        assert received == [(1, 'Flare')]
        assert Topics.Dashboard.MISSION_COMMANDS.qos.durability == DurabilityPolicy.VOLATILE
        producer.destroy_subscription(subscription)
        create_subscription_from(producer, Topics.Dashboard.MISSION_COMMANDS,
                                 lambda msg: received.append((msg.command, msg.mission)))
        time.sleep(0.4)
        assert received == [(1, 'Flare')]
    finally:
        adapter.executor.remove_node(producer)
        producer.destroy_node()
        adapter.close()
