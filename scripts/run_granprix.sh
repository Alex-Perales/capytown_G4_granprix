#!/usr/bin/env bash
# ============================================================================
# run_granprix.sh — CapyTown Gran Prix: TODO con UN SOLO comando en RealVNC.
# ============================================================================
# Reemplaza las ~7 terminales de la guía "Comandos YahboomCar" por UNA sola
# ventana de terminal (tmux, dividida en paneles) + 3 ventanas gráficas que se
# abren solas (cámara cruda, overlay de detección de PARE, RViz).
#
# Uso (en la Pi, dentro de una terminal de RealVNC):
#   bash ~/yahboomcar_ws/src/capytown_granprix_pkg_scripts/run_granprix.sh
# (o donde hayas copiado este script — ver el scp más abajo)
#
# Para DETENER todo:
#   tmux kill-session -t granprix
#
# ---------------------------------------------------------------------------
# AJUSTA ESTOS 2 IDs DE CONTENEDOR SI SON DISTINTOS EN TU PI (`docker ps -a`):
#   CAM_CONTAINER  = el contenedor donde corre `ros2 launch usb_cam camera.launch.py`
#   MAIN_CONTAINER = el contenedor donde corre tu código (antes lane_node.py)
# Los valores de abajo son los que aparecen en tu guía "Comandos YahboomCar".
# ---------------------------------------------------------------------------
CAM_CONTAINER="${CAM_CONTAINER:-cc35232ac52b}"
MAIN_CONTAINER="${MAIN_CONTAINER:-b8734b1c0964}"

# Ruta del paquete YA compilado dentro del workspace de la Pi (ver scp/colcon
# build en las instrucciones que te di). Ajusta si usaste otro nombre/ruta.
WS_SETUP="source ~/yahboomcar_ws/install/setup.bash && source /opt/ros/humble/setup.bash"

SESSION="granprix"

set -u

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux no está instalado. Instálalo una vez con:"
  echo "    sudo apt-get update && sudo apt-get install -y tmux"
  exit 1
fi

# Reinicia limpio si ya había una corrida anterior abierta.
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "==> Lanzando CapyTown Gran Prix (sesión tmux '$SESSION')..."
echo "    CAM_CONTAINER=$CAM_CONTAINER  MAIN_CONTAINER=$MAIN_CONTAINER"
echo "    (cámbialos con: CAM_CONTAINER=xxx MAIN_CONTAINER=yyy bash run_granprix.sh)"

# ---------------------------------------------------------------------------
# Ventana/panel 1: chasis + IMU + LiDAR (bringup) — corre DIRECTO en la Pi,
# no en docker (igual que Terminal 1 de tu guía).
# ---------------------------------------------------------------------------
tmux new-session -d -s "$SESSION" -n main
tmux send-keys -t "$SESSION:main" \
  "echo '[BRINGUP] chasis + IMU + LiDAR'; sh ros2_humble.sh; $WS_SETUP && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 2: cámara (usb_cam) — dentro del contenedor de cámara.
# ---------------------------------------------------------------------------
tmux split-window -h -t "$SESSION:main"
tmux send-keys -t "$SESSION:main.1" \
  "echo '[CAMARA] usb_cam'; docker exec -it $CAM_CONTAINER bash -lc '$WS_SETUP && ros2 launch usb_cam camera.launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 3: código principal del Gran Prix (maze_solver + pare_detector +
# box_detector) — dentro del contenedor principal (antes corría lane_node.py).
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.0"
tmux send-keys -t "$SESSION:main.2" \
  "echo '[GRAN PRIX] maze_solver + pare_detector + box_detector'; docker exec -it $MAIN_CONTAINER bash -lc '$WS_SETUP && cd ~/yahboomcar_ws && ros2 launch capytown_granprix_pkg granprix.launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 4: estado de la FSM en vivo (/maze_state) — "terminal de estado".
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.1"
tmux send-keys -t "$SESSION:main.3" \
  "echo '[ESTADO FSM] /maze_state (Ctrl+C aquí NO detiene el robot, solo el echo)'; sleep 6; docker exec -it $MAIN_CONTAINER bash -lc '$WS_SETUP && ros2 topic echo /maze_state'" C-m

tmux select-layout -t "$SESSION:main" tiled

# ---------------------------------------------------------------------------
# Ventanas gráficas (se abren solas en el escritorio de RealVNC, aparte de
# tmux): cámara cruda, overlay de detección de PARE, y RViz.
# Se lanzan DESPUÉS de dar tiempo a que cámara+código principal arranquen.
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n guis
tmux send-keys -t "$SESSION:guis" \
  "echo 'Esperando 12s a que la cámara y el código principal arranquen...'; sleep 12; \
echo '[GUI] cámara cruda (/image_raw)'; \
docker exec -d $CAM_CONTAINER bash -lc '$WS_SETUP && ros2 run rqt_image_view rqt_image_view /image_raw'; \
sleep 2; \
echo '[GUI] overlay de detección de PARE (/pare/debug_image)'; \
docker exec -d $CAM_CONTAINER bash -lc '$WS_SETUP && ros2 run rqt_image_view rqt_image_view /pare/debug_image'; \
sleep 2; \
echo '[GUI] RViz (trayectoria /odom_raw + markers /granprix_markers)'; \
docker exec -d $MAIN_CONTAINER bash -lc '$WS_SETUP && (rviz2 || echo \"rviz2 no está instalado en este contenedor\")'; \
echo; echo 'Listo. Si alguna ventana no aparece: revisa que ese contenedor tenga DISPLAY/X11'; \
echo 'configurado (ya lo tenía para rqt_image_view en tu guía original), y reintenta'; \
echo 'corriendo el mismo comando docker exec SIN -d (para ver el error).'" C-m

tmux select-window -t "$SESSION:main"
echo "==> Listo. Adjuntando a la sesión tmux (Ctrl+B luego D para salir sin detener nada)..."
sleep 1
tmux attach -t "$SESSION"
