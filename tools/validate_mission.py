"""Missão integrada com voo Gazebo e imagem controlada, usando os nós reais."""

import json
import math
import time


def run_mission(output, drone, probe, executor, until, image_path, extra_nodes):
    import cv2
    import torch
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CompressedImage
    from drone_inspetor_msgs.msg import CVControlMSG, DashboardMissionCommandMSG
    from drone_inspetor.nodes.cv_node.cv_node import CVNode
    from drone_inspetor.nodes.mission_node.mission_node import MissionNode
    from drone_inspetor.ros_interfaces import Topics, create_publisher_from

    home = drone.state_px4
    definition = {'integrated': {'nome': 'Validação integrada', 'takeoff_altitude': 3.,
                  'tempo_de_permanencia': 2., 'pontos_de_inspecao': [{
                      'lat': home.home_global_lat + math.degrees(3. / 6371000.),
                      'lon': home.home_global_lon, 'alt': home.home_global_alt + 3.,
                      'ponto_de_deteccao': True, 'objeto_alvo': 'flare', 'tipos_anomalia': []}]}}
    mission_file = output / 'mission.json'
    mission_file.write_text(json.dumps(definition, indent=2))
    mission = MissionNode(parameter_overrides=[
        Parameter('missions_file', value=str(mission_file)),
        Parameter('missions_directory', value=str(output / 'mission-artifacts')),
        Parameter('mission_period', value=.2)])
    extra_nodes.append(mission)
    torch.set_num_threads(2)
    vision = CVNode(parameter_overrides=[Parameter('inference_device', value='cpu')])
    extra_nodes.append(vision)
    executor.add_node(mission)
    executor.add_node(vision)
    controls = create_publisher_from(probe, Topics.Dashboard.CV_CONTROL)
    commands = create_publisher_from(probe, Topics.Dashboard.MISSION_COMMANDS)
    camera = create_publisher_from(probe, Topics.Externo.CAMERA_COMPRESSED)
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError('Imagem de missão não decodificada')
    encoded = cv2.imencode('.jpg', image)[1].tobytes()
    states = []

    def frame():
        msg = CompressedImage()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.format, msg.data = 'jpeg', encoded
        camera.publish(msg)
        state = mission.mission_machine.current_state_id.name
        if not states or states[-1]['state'] != state:
            states.append(dict(state=state, time=time.monotonic()))
            (output / 'mission-states.json').write_text(json.dumps(states, indent=2))

    timer = probe.create_timer(.2, frame)
    try:
        until(lambda: controls.get_subscription_count() > 0, 10, 'CV sem assinante de controle')
        control = CVControlMSG()
        control.object_detection_model = 'flare_yolov8n_detection_300ep.pt'
        control.anomaly_detection_model = 'corrosion_yolo8n_detection.pt'
        controls.publish(control)
        until(lambda: vision._models.filenames == (control.object_detection_model,
                                                   control.anomaly_detection_model),
              15, 'Pesos leves reais não selecionados')
        until(lambda: mission.mission_machine.current_state_id.name == 'PRONTO',
              15, 'MissionNode não ficou PRONTO')
        message = DashboardMissionCommandMSG()
        message.command, message.mission = 1, 'integrated'
        commands.publish(message)
        until(lambda: any(x['state'] == 'RETORNANDO' for x in states)
              and not drone.state_px4.is_armed, 180, 'Missão não retornou/desarmou')
        observed = {x['state'] for x in states}
        if 'EXECUTANDO_INSPECIONANDO_ESCANEANDO' not in observed:
            raise RuntimeError('Missão não confirmou detecção/gravação')
        videos = list((output / 'mission-artifacts').rglob('*.mp4'))
        valid = []
        for path in videos:
            capture = cv2.VideoCapture(str(path))
            ok, _ = capture.read()
            capture.release()
            if ok:
                valid.append(str(path))
        if not valid:
            raise RuntimeError('Nenhum vídeo de inspeção decodificável')
        return dict(states=states, videos=valid, image=str(image_path),
                    scope='Voo real simulado; visão real sobre imagem de referência repetida')
    finally:
        timer.cancel()
