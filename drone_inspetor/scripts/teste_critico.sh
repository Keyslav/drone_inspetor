#!/bin/bash
# Script de teste completo do drone_node com GOTO_FOCUS via ROS2 Actions

ACTION_NAME="/drone_inspetor/action/drone_command"
ACTION_TYPE="drone_inspetor_msgs/action/DroneCommand"

echo "=== Teste do drone_node com GOTO_FOCUS (via Actions) ==="

echo "1. Armando motores..."
ros2 action send_goal $ACTION_NAME $ACTION_TYPE "{command: 'ARM'}"
sleep 5

echo "2. Decolando para 10 metros..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'TAKEOFF', altitude: 10.0}"
sleep 5

echo "3. GOTO para ponto com yaw 0..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO', lat: -22.633978, lon: -40.092433, alt: 99.0, yaw: 0.0}"
sleep 5

echo "4. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO_FOCUS', lat: -22.633889, lon: -40.092326, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 5

echo "5. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO_FOCUS', lat: -22.633800, lon: -40.092433, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 5

echo "6. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO_FOCUS', lat: -22.633889, lon: -40.092504, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 5

echo "7. GOTO com apenas rotação yaw 0..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO', yaw: 0.0}"
sleep 5

echo "8. GOTO para altitude 102..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO', alt: 102.0}"
sleep 5

echo "9. GOTO_FOCUS para ponto mantendo foco..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'GOTO_FOCUS', lat: -22.633800774, lon: -40.09143355817096, alt: 102.0, focus_lat: -22.63388829505115, focus_lon: -40.09243555158753}"
sleep 5

echo "10. Retornando para base (RTL)..."
ros2 action send_goal --feedback $ACTION_NAME $ACTION_TYPE "{command: 'RTL'}"

echo "=== Teste concluído ==="
