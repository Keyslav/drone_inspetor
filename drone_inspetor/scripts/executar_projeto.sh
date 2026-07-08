#!/usr/bin/env bash
# ============================================================
# executar_projeto.sh
# Sobe TODO o projeto DRONE_INSPETOR abrindo cada componente em
# seu proprio terminal, automaticamente. O terminal que chama
# este script NAO fica preso: ele retorna apos disparar tudo.
#
# Uso:  bash executar_projeto.sh
# ============================================================
set -u

WS="$HOME/ros2_ws"
PKG_SCRIPTS="$WS/src/drone_inspetor/drone_inspetor/scripts"
PX4_DIR="$HOME/PX4-Autopilot"

# Comando do PX4 (uma linha). Aspas duplas do MODEL_POSE sao preservadas.
PX4_CMD='cd '"$PX4_DIR"' && SIM_GZ_HOME_LAT=-22.633890 SIM_GZ_HOME_LON=-40.093330 SIM_GZ_HOME_ALT=0 GZ_IP=127.0.0.1 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=Plataforma_UERJ PX4_SYS_AUTOSTART=4030 PX4_GZ_MODEL_POSE="-70,-27,57" ./build/px4_sitl_default/bin/px4'

# ------------------------------------------------------------
# Detecta um emulador de terminal disponivel
detect_term() {
  for t in gnome-terminal konsole xfce4-terminal xterm; do
    command -v "$t" >/dev/null 2>&1 && { echo "$t"; return; }
  done
  echo ""
}
TERMEMU="$(detect_term)"
if [ -z "$TERMEMU" ]; then
  echo "ERRO: nenhum emulador de terminal encontrado."
  echo "Instale um:  sudo apt install gnome-terminal   (ou: sudo apt install xterm)"
  echo "Ou use a versao em tmux:  bash executar_projeto.sh --tmux"
  exit 1
fi

# Abre um comando em um novo terminal, SEM bloquear o terminal atual.
# Cada terminal carrega o ROS + o workspace e mantem a janela aberta ao final.
open_term() {
  local title="$1"; local cmd="$2"
  local full="source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; $cmd; echo; echo '=== $title terminou (feche a janela ou Ctrl+D) ==='; exec bash"
  case "$TERMEMU" in
    gnome-terminal) setsid gnome-terminal --title="$title" -- bash -lc "$full" >/dev/null 2>&1 & ;;
    konsole)        setsid konsole -p tabtitle="$title" -e bash -lc "$full"     >/dev/null 2>&1 & ;;
    xfce4-terminal) setsid xfce4-terminal --title="$title" -x bash -lc "$full"  >/dev/null 2>&1 & ;;
    xterm)          setsid xterm -T "$title" -e bash -lc "$full"                >/dev/null 2>&1 & ;;
  esac
  disown 2>/dev/null || true
}

# ------------------------------------------------------------
# Alternativa em tmux (util no WSL2 sem emulador grafico):
#   janelas dentro de uma sessao 'drone'; veja com: tmux attach -t drone
run_tmux() {
  command -v tmux >/dev/null 2>&1 || { echo "tmux nao instalado: sudo apt install tmux"; exit 1; }
  local SRC="source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash;"
  tmux kill-session -t drone 2>/dev/null || true
  tmux new-session  -d -s drone -n gazebo "$SRC cd $PKG_SCRIPTS && bash projeto_gazebo.sh; exec bash"
  sleep 5
  tmux new-window -t drone -n gui   "$SRC gz sim -g; exec bash"
  sleep 2
  tmux new-window -t drone -n px4   "$SRC $PX4_CMD; exec bash"
  sleep 3
  tmux new-window -t drone -n uxrce "$SRC MicroXRCEAgent udp4 -p 8888; exec bash"
  sleep 2
  tmux new-window -t drone -n app   "$SRC ros2 launch drone_inspetor ground_station_launch.py; exec bash"
  echo "Sessao tmux 'drone' criada. Veja com:  tmux attach -t drone"
  echo "(troque de janela: Ctrl+B depois 0..4 | encerre tudo: tmux kill-session -t drone)"
}

if [ "${1:-}" = "--tmux" ]; then
  run_tmux
  exit 0
fi

# ------------------------------------------------------------
echo "Emulador de terminal: $TERMEMU"
echo "Abrindo os terminais do projeto (o terminal atual segue livre)..."

echo "-> [1/5] Gazebo (mundo fisico, headless)"
open_term "1-Gazebo" "cd $PKG_SCRIPTS && bash projeto_gazebo.sh"
sleep 5

echo "-> [2/5] Gazebo GUI (opcional)"
open_term "2-GazeboGUI" "gz sim -g"
sleep 2

echo "-> [3/5] Firmware PX4"
open_term "3-PX4" "$PX4_CMD"
sleep 3

echo "-> [4/5] Middleware uXRCE-DDS (porta 8888)"
open_term "4-uXRCE" "MicroXRCEAgent udp4 -p 8888"
sleep 2

echo "-> [5/5] Aplicacao (ground station)"
open_term "5-App" "ros2 launch drone_inspetor ground_station_launch.py"

echo "Pronto: 5 terminais abertos. Este terminal esta livre para o proximo comando."
