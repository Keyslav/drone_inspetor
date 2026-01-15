#!/bin/bash
# Script para iniciar missão via FSM Node
# Envia comando INICIAR_MISSAO para o FSM Node

QOS_OPTS="--qos-durability transient_local --qos-reliability reliable"
TOPIC="/drone_inspetor/interno/dashboard_node/fsm_commands"
MSG_TYPE="drone_inspetor_msgs/msg/DashboardFsmCommandMSG"

# Nome da missão (deve existir em missions.json)
MISSION_NAME="${1:-flare}"

echo "=== Iniciando Missão via FSM ==="
echo "Missão: $MISSION_NAME"
echo ""

echo "Enviando comando INICIAR_MISSAO (command=1)..."
ros2 topic pub --once -w 2 $QOS_OPTS $TOPIC $MSG_TYPE \
  "{command: 1, mission: '$MISSION_NAME'}"

echo ""
echo "=== Comando enviado! ==="
echo "Acompanhe o progresso da FSM via:"
echo "  ros2 topic echo /drone_inspetor/interno/fsm_node/fsm_state"
