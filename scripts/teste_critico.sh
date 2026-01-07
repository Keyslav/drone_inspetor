#!/bin/bash
# Script de teste completo do drone_node com GOTO_FOCUS

QOS_OPTS="--qos-durability transient_local --qos-reliability reliable"
TOPIC="/drone_inspetor/interno/fsm_node/drone_commands"
MSG_TYPE="drone_inspetor_msgs/msg/MissionCommandMSG"

echo "=== Teste do drone_node com GOTO_FOCUS ==="

echo "1. Armando motores..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'ARM'}"
sleep 5

echo "2. Decolando para 10 metros..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'TAKEOFF', altitude: 10.0}"
sleep 10

echo "3. GOTO para ponto com yaw específico..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO', lat: -22.634035, lon: -40.093549, alt: 66.0, yaw: 38.0}"
sleep 30

echo "4. GOTO_FOCUS para ponto mantendo foco..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO_FOCUS', lat: -22.635035, lon: -40.092549, alt: 66.0, focus_lat: -22.636035, focus_lon: -40.092549}"
sleep 30

echo "5. Retornando para base (RTL)..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'RTL'}"
sleep 20

echo "=== Teste concluído ==="
