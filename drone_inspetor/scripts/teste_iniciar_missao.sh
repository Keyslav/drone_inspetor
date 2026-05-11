#!/bin/bash
# Script para iniciar missão via Mission Node
# Envia comando INICIAR_MISSAO para o Mission Node

QOS_OPTS="--qos-durability transient_local --qos-reliability reliable"
TOPIC="/drone_inspetor/interno/dashboard_node/mission_commands"
MSG_TYPE="drone_inspetor_msgs/msg/DashboardMissionCommandMSG"

# Nome da missão (deve existir em missions.json)
MISSION_NAME="${1:-flare}"

echo "=== Iniciando Missão via Mission Node ==="
echo "Missão: $MISSION_NAME"
echo ""

echo "Enviando comando INICIAR_MISSAO (command=1)..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 1, mission: '$MISSION_NAME'}"

echo ""
echo "=== Comando enviado! ==="
echo "Acompanhe o progresso da máquina de missão via:"
echo "  ros2 topic echo /drone_inspetor/interno/mission_node/mission_state"
