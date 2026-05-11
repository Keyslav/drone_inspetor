"""
drone_inspetor — pacote ROS2 para inspeção industrial autônoma com drone.

Subpacotes:
- common/: enums, tópicos, perfis QoS, constantes, utilitários
- nodes/: 7 entry points (drone_node, mission_node, dashboard_node, camera_node, cv_node, depth_node, lidar_node)
- publishers/, subscribers/, signals/: comunicação modular do dashboard
- gui/: interface PyQt6 (controles, mapa, visualização de estados)
"""
