#!/usr/bin/env bash
# ============================================================================
# run_granprix.sh — CapyTown Gran Prix: TODO con UN SOLO comando en RealVNC.
# ============================================================================
# ROS2 Humble SOLO existe dentro del contenedor Docker que crea
# ~/ros2_humble.sh (imagen yahboomtechnology/ros-humble:4.1.2). Ese contenedor
# ya trae montados: tu workspace (/root/yahboomcar_ws), la cámara (/dev/video0)
# y la red (--net=host, para el LiDAR y ROS2 DDS) — así que TODO (bringup,
# cámara, maze_solver, pare_detector, RViz) corre dentro de ESE MISMO
# contenedor, con varios `docker exec` en paralelo. Este script lo detecta
# solo (no hace falta anotar IDs a mano).
#
# ANTES de correr esto por primera vez en esta sesión de la Pi:
#   1) Abre OTRA terminal en el escritorio de RealVNC.
#   2) Corre:  sh ~/ros2_humble.sh
#   3) Espera a que te deje con un prompt (root@...). DÉJALA ABIERTA — esa
#      terminal ES el contenedor; si la cierras, el contenedor se apaga.
#   4) Vuelve a esta otra terminal y corre:  bash ~/run_granprix.sh
#
# Para DETENER la parte de tmux (cámara/código/estado — el contenedor en sí
# sigue vivo mientras no cierres la terminal del paso 2):
#   tmux kill-session -t granprix
# ============================================================================

IMAGE_FILTER="${IMAGE_FILTER:-yahboomtechnology/ros-humble}"
WS_DIR="${WS_DIR:-/root/yahboomcar_ws}"   # ruta del workspace DENTRO del contenedor (ver ros2_humble.sh)
WS_SETUP="source /opt/ros/humble/setup.bash && cd $WS_DIR && (source install/setup.bash 2>/dev/null || true)"
SESSION="granprix"

set -u

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux no está instalado. Instálalo una vez con:"
  echo "    sudo apt-get update && sudo apt-get install -y tmux"
  exit 1
fi

# ---------------------------------------------------------------------------
# Detecta el contenedor del robot (creado por ~/ros2_humble.sh). Permite
# forzar uno específico con:  CONTAINER=<id> bash run_granprix.sh
# ---------------------------------------------------------------------------
CONTAINER="${CONTAINER:-}"
if [ -z "$CONTAINER" ]; then
  CONTAINER="$(docker ps -q --filter "ancestor=${IMAGE_FILTER}" | head -n1)"
fi
if [ -z "$CONTAINER" ]; then
  # fallback: cualquier contenedor corriendo (por si el nombre de imagen cambió de tag)
  CONTAINER="$(docker ps -q | head -n1)"
fi

if [ -z "$CONTAINER" ]; then
  echo "!!! No encuentro ningún contenedor corriendo."
  echo "    Abre OTRA terminal en RealVNC, corre:  sh ~/ros2_humble.sh"
  echo "    y déjala abierta. Luego vuelve a correr:  bash ~/run_granprix.sh"
  exit 1
fi

echo "==> Usando contenedor: $CONTAINER (workspace: $WS_DIR)"
echo "    (para forzar otro:  CONTAINER=xxxx bash run_granprix.sh)"

# Reinicia limpio si ya había una corrida anterior abierta.
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "==> Lanzando CapyTown Gran Prix (sesión tmux '$SESSION')..."

# ---------------------------------------------------------------------------
# Panel 1: chasis + IMU + LiDAR (bringup)
# ---------------------------------------------------------------------------
tmux new-session -d -s "$SESSION" -n main
tmux send-keys -t "$SESSION:main" \
  "echo '[BRINGUP] chasis + IMU + LiDAR'; docker exec -it $CONTAINER bash -lc '$WS_SETUP && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 2: cámara (usb_cam)
# ---------------------------------------------------------------------------
tmux split-window -h -t "$SESSION:main"
tmux send-keys -t "$SESSION:main.1" \
  "echo '[CAMARA] usb_cam'; docker exec -it $CONTAINER bash -lc '$WS_SETUP && ros2 launch usb_cam camera.launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 3: código principal del Gran Prix (maze_solver + pare_detector + box_detector)
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.0"
tmux send-keys -t "$SESSION:main.2" \
  "echo '[GRAN PRIX] maze_solver + pare_detector + box_detector'; sleep 4; docker exec -it $CONTAINER bash -lc '$WS_SETUP && ros2 launch capytown_granprix_pkg granprix.launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 4: estado de la FSM en vivo (/maze_state) — "terminal de estado".
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.1"
tmux send-keys -t "$SESSION:main.3" \
  "echo '[ESTADO FSM] /maze_state (Ctrl+C aquí NO detiene el robot, solo el echo)'; sleep 8; docker exec -it $CONTAINER bash -lc '$WS_SETUP && ros2 topic echo /maze_state'" C-m

tmux select-layout -t "$SESSION:main" tiled

# ---------------------------------------------------------------------------
# Ventanas gráficas (se abren solas en el escritorio de RealVNC, aparte de
# tmux): cámara cruda, overlay de detección de PARE, y RViz.
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n guis
tmux send-keys -t "$SESSION:guis" \
  "echo 'Esperando 12s a que la cámara y el código principal arranquen...'; sleep 12; \
echo '[GUI] cámara cruda (/image_raw)'; \
docker exec -d $CONTAINER bash -lc '$WS_SETUP && ros2 run rqt_image_view rqt_image_view /image_raw'; \
sleep 2; \
echo '[GUI] overlay de detección de PARE (/pare/debug_image)'; \
docker exec -d $CONTAINER bash -lc '$WS_SETUP && ros2 run rqt_image_view rqt_image_view /pare/debug_image'; \
sleep 2; \
echo '[GUI] RViz (trayectoria /odom_raw + markers /granprix_markers)'; \
docker exec -d $CONTAINER bash -lc '$WS_SETUP && (rviz2 || echo \"rviz2 no está instalado en este contenedor\")'; \
echo; echo 'Listo. Si alguna ventana no aparece, reintenta ese mismo comando docker exec'; \
echo 'SIN el -d (para verlo interactivo y leer el error).'" C-m

tmux select-window -t "$SESSION:main"
echo "==> Listo. Adjuntando a la sesión tmux (Ctrl+B luego D para salir sin detener nada)..."
sleep 1
tmux attach -t "$SESSION"
