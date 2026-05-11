"""
Pacote nodes/ — cada subpacote é um nó ROS2 com seu próprio entry point `main`.

Nodes:
- dashboard_node/: orquestrador GUI ↔ ROS2
- camera_node/: câmera principal (raw, gravação, fotos)
- cv_node/: visão computacional YOLO (detecção de objetos + anomalias)
- depth_node/: câmera de profundidade + alertas de proximidade
- lidar_node/: LiDAR 2D horizontal + 1D inferior
- drone_node/: interface PX4 (com FSM interna em fsm/)
- mission_node/: máquina de missão hierárquica (com FSM em fsm/)
"""
