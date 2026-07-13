#!/usr/bin/env bash
# ============================================================================
# run_granprix.sh — CapyTown Gran Prix: TODO con UN SOLO comando en RealVNC.
# ============================================================================
# Replica EXACTAMENTE tu procedimiento de 5 terminales ("Comandos YahboomCar")
# en una sola ventana (tmux con paneles) + ventanas gráficas que se abren
# solas. Usa los MISMOS 3 contenedores fijos que ya te funcionaban:
#   CAM_CONTAINER   = cc35232ac52b  (cámara: camera.launch.py Y rqt_image_view)
#   MAIN_CONTAINER  = b8734b1c0964  (código principal: antes lane_node.py,
#                                    ahora maze_solver + pare_detector + box_detector)
#   SERVO_CONTAINER = 994f4c1a8dfc  (posición de la cámara, servo_s1/s2, una vez)
# El bringup (chasis/IMU/LiDAR) sigue corriendo DIRECTO en la Pi (sh
# ros2_humble.sh), igual que tu Terminal 1 original — no en docker.
#
# Si `docker ps -a` te muestra IDs distintos a los de arriba, cámbialos así:
#   CAM_CONTAINER=xxxx MAIN_CONTAINER=yyyy SERVO_CONTAINER=zzzz bash run_granprix.sh
#
# Para DETENER todo:
#   tmux kill-session -t granprix
# ============================================================================

CAM_CONTAINER="${CAM_CONTAINER:-cc35232ac52b}"
MAIN_CONTAINER="${MAIN_CONTAINER:-b8734b1c0964}"
SERVO_CONTAINER="${SERVO_CONTAINER:-994f4c1a8dfc}"
SERVO_S1="${SERVO_S1:-20}"
SERVO_S2="${SERVO_S2:--55}"

WS_SETUP="source ~/yahboomcar_ws/install/setup.bash && source /opt/ros/humble/setup.bash"
SESSION="granprix"

set -u

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux no está instalado. Instálalo una vez con:"
  echo "    sudo apt-get update && sudo apt-get install -y tmux"
  exit 1
fi

# Asegura que los 3 contenedores existan y estén corriendo (si estaban
# "Exited" desde un reinicio de la Pi, los reactiva; si ya corren, no hace nada).
ensure_running() {
  local id="$1"
  if ! docker ps -a --format '{{.ID}}' | grep -q "^${id:0:12}"; then
    echo "!!! No encuentro el contenedor $id (docker ps -a). Revisa el ID y ajusta la variable correspondiente."
    return 1
  fi
  if ! docker ps --format '{{.ID}}' | grep -q "^${id:0:12}"; then
    echo "==> Contenedor $id estaba detenido, arrancándolo..."
    docker start "$id" >/dev/null
    sleep 2
  fi
  return 0
}

echo "==> Verificando contenedores (CAM=$CAM_CONTAINER MAIN=$MAIN_CONTAINER SERVO=$SERVO_CONTAINER)..."
ensure_running "$CAM_CONTAINER" || exit 1
ensure_running "$MAIN_CONTAINER" || exit 1
ensure_running "$SERVO_CONTAINER" || echo "    (servo opcional — si falla, sigue igual sin posicionar la cámara)"

# Reinicia limpio si ya había una corrida anterior abierta.
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "==> Lanzando CapyTown Gran Prix (sesión tmux '$SESSION')..."

# ---------------------------------------------------------------------------
# Panel 1: chasis + IMU + LiDAR (bringup) — DIRECTO en la Pi, igual que
# tu Terminal 1 original (no docker).
# ---------------------------------------------------------------------------
tmux new-session -d -s "$SESSION" -n main
tmux send-keys -t "$SESSION:main" \
  "echo '[BRINGUP] chasis + IMU + LiDAR'; sh ~/ros2_humble.sh; $WS_SETUP && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" C-m

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
# box_detector) — dentro del contenedor principal (antes lane_node.py).
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.0"
tmux send-keys -t "$SESSION:main.2" \
  "echo '[GRAN PRIX] maze_solver + pare_detector + box_detector'; sleep 5; docker exec -it $MAIN_CONTAINER bash -lc '$WS_SETUP && cd ~/yahboomcar_ws && ros2 launch capytown_granprix_pkg granprix.launch.py'" C-m

sleep 1

# ---------------------------------------------------------------------------
# Panel 4: estado de la FSM en vivo (/maze_state) — "terminal de estado".
# ---------------------------------------------------------------------------
tmux split-window -v -t "$SESSION:main.1"
tmux send-keys -t "$SESSION:main.3" \
  "echo '[ESTADO FSM] /maze_state (Ctrl+C aquí NO detiene el robot, solo el echo)'; sleep 9; docker exec -it $MAIN_CONTAINER bash -lc '$WS_SETUP && ros2 topic echo /maze_state'" C-m

tmux select-layout -t "$SESSION:main" tiled

# ---------------------------------------------------------------------------
# Posición de la cámara (servo) — UNA sola vez al arrancar, igual que tu
# "Terminal opcional". No abre panel visible, solo publica y termina.
# ---------------------------------------------------------------------------
tmux new-window -t "$SESSION" -n servo
tmux send-keys -t "$SESSION:servo" \
  "echo '[SERVO] posicionando cámara (s1=$SERVO_S1 s2=$SERVO_S2)...'; sleep 3; \
docker exec -it $SERVO_CONTAINER bash -lc \"$WS_SETUP && ros2 topic pub /servo_s1 std_msgs/msg/Int32 'data: $SERVO_S1' --once && ros2 topic pub /servo_s2 std_msgs/msg/Int32 'data: $SERVO_S2' --once\"; \
echo '[SERVO] listo.'" C-m

# ---------------------------------------------------------------------------
# Ventanas gráficas (se abren solas en el escritorio de RealVNC, aparte de
# tmux): cámara cruda, overlay de detección de PARE, y RViz.
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
echo; echo 'Listo. Si alguna ventana no aparece, reintenta ese mismo comando docker exec'; \
echo 'SIN el -d (para verlo interactivo y leer el error).'" C-m

tmux select-window -t "$SESSION:main"
echo "==> Listo. Adjuntando a la sesión tmux (Ctrl+B luego D para salir sin detener nada)..."
sleep 1
tmux attach -t "$SESSION"
