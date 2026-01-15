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
sleep 20

echo "3. GOTO para ponto com yaw 0..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO',       lat: -22.633978, lon: -40.092433, alt: 99.0, yaw: 0.0}"
sleep 60

echo "4. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO_FOCUS', lat: -22.633889, lon: -40.092326, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 45

echo "5. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO_FOCUS', lat: -22.633800, lon: -40.092433, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 30

echo "6. GOTO_FOCUS para ponto mantendo foco no Flare..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO_FOCUS', lat: -22.633889, lon: -40.092504, alt: 99.0, focus_lat: -22.633888, focus_lon: -40.092435}"
sleep 30

echo "7. GOTO com apenas rotação yaw 0..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO', lat: nan, lon: nan, alt: nan, yaw: 0.0}"
sleep 30

echo "8. GOTO para altitude 102..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO', lat: nan, lon: nan, alt: 102.0, yaw: nan}"
sleep 30

echo "9. GOTO_FOCUS para ponto mantendo foco..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'GOTO_FOCUS', lat: -22.633800774, lon: -40.09143355817096, alt: 102.0, focus_lat: -22.63388829505115, focus_lon: -40.09243555158753}"
sleep 30

echo "9. Retornando para base (RTL)..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 'RTL'}"
sleep 60

echo "=== Teste concluído ==="

