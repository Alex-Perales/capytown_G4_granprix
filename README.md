# CapyTown Gran Prix — El Qhapaq Ñan: El Laberinto del Chaski

Paquete ROS 2 Humble para el reto final del curso: navegación **totalmente autónoma** de un laberinto (Yahboom ROSMASTER, Raspberry Pi 5), fusionando **LiDAR** (geometría: seguimiento de pared, intersecciones, dead-ends) y **cámara** (semántica: detección de la señal PARE). Equipo **CapyTown G4**.

## 1. El reto

- **Pista:** laberinto de 360×240 cm, rejilla 6×4 de celdas de 60 cm, paredes de MDF. Inicio en la esquina inferior izquierda, meta en la esquina superior derecha (esquinas opuestas), con múltiples rutas válidas y callejones sin salida a propósito.
- **Señales PARE:** 2–3 carteles rojos en algunas intersecciones; el robot debe detectarlos con la cámara y detenerse por completo (~3 s) antes de continuar. Sus posiciones cambian entre corridas.
- **Arbitraje cámara↔LiDAR:** la cámara manda para *detener* (PARE tiene prioridad de seguridad); el LiDAR manda para *moverse/centrar* en el pasillo.
- **Dos rondas:** Ronda 1 (exploración, sin conocer el trazado) y Ronda 2 (time attack, ruta más corta y más rápida).
- **Variante opcional "karpinchus":** 1–2 cajas en los pasillos que el robot debe detectar, detenerse (≥15 cm) y rodear sin perder el rumbo ni saltarse un PARE.

El enunciado completo del reto (pista, rúbrica, métricas exigidas, preguntas de defensa) está en `Doc/CapyTown_GranPrix_Laberinto.docx`.

## 2. Estructura del proyecto

```
capytown_G4_s13/
├── capytown_granprix_pkg/        # paquete ROS2 (ament_python)
│   ├── maze_solver.py            # nodo principal: FSM de navegación + fusión PARE + métricas + RViz
│   ├── pare_detector.py          # cámara -> HSV rojo/verde -> /pare_detectado, /meta_detectado
│   ├── box_detector.py           # censo de "karpinchus" (cajas) por LiDAR -> /cajas_avistadas
│   ├── scan_map_viewer.py        # visor Tk opcional: scan clasificado + recorrido (debug, no navega)
│   ├── web_dashboard.py          # panel web (http://<IP-robot>:8080/): LiDAR + cámara + pausa + tuning
│   ├── desktop_dashboard.py      # misma info que web_dashboard pero como ventana Tk nativa
│   ├── simple_camera.py          # publica /image_raw con cv2.VideoCapture (fallback si usb_cam falla)
│   ├── split_merge.py            # extracción de líneas del LiDAR (Split & Merge); aditivo, solo logging
│   └── loop_completion.py        # detección de vuelta completa; no se usa en este reto (heredado de RC-4)
├── config/granprix_params.yaml   # parámetros por nodo (editar aquí entre corridas)
├── launch/granprix.launch.py     # lanza maze_solver + pare_detector + box_detector (+ dashboards opcionales)
├── scripts/run_granprix.sh       # UN comando: bringup + cámara + código + estado FSM + GUIs (tmux)
├── web/dashboard_preview.html    # preview estático del dashboard
└── Doc/CapyTown_GranPrix_Laberinto.docx   # enunciado oficial del reto
```

## 3. Arquitectura de nodos

```
/scan ──┬─────────────────────────────────────────────────┐
        │                                                  ▼
/odom_raw ─────────────────────────────────────────► maze_solver.py ──► /cmd_vel  (único publicador)
                                                          ▲   │
/image_raw ──► pare_detector.py ──► /pare_detectado ─────┘   ├──► /maze_state (estado FSM en vivo)
                              └───► /meta_detectado ──────┘   ├──► /granprix_markers (RViz)
                                                                └──► metricas_granprix.csv
/scan + /odom_raw ──► box_detector.py ──► /cajas_avistadas ──► maze_solver.py (rodeo de karpinchus)
```

`maze_solver.py` es el **único** nodo que escribe en `/cmd_vel`: centraliza toda decisión de movimiento (evita comandos contradictorios). Reutiliza tal cual el controlador de seguimiento de pared / dead-end / rodeo de cajas del reto RC-4 y le agrega encima la fusión con la cámara (PARE), detección de META por odometría, métricas y markers de RViz.

### Máquina de estados (`maze_solver.py`)

| Estado | Significado |
|---|---|
| `FOLLOW_WALL` | sigue la pared (derecha por defecto) a `wall_target`, avanza mientras el frente esté libre |
| `TURN_IN` | frente bloqueado y lado seguido bloqueado → gira alejándose de la pared (90°, por odometría) |
| `TURN_OUT` | la pared seguida desapareció (esquina/apertura) → gira hacia ella y reacopla |
| `RECOVER` | encajonado por 3 lados (dead-end) → media vuelta (180°) |
| `PARAR_PARE` | cámara confirmó un cartel PARE → detención completa ~3 s |
| `AVANCE_META` / `META` | cámara confirmó el cartel META (o se entró al radio de meta por odometría) → detiene y registra la corrida |

Además: guardia de emergencia (frente muy cerca → frena o retrocede), watchdog de "atascado" (sin progreso posicional → retrocede), anti-cascada de giros (evita que dos `TURN_IN` de 90° se acumulen en un giro de 180° accidental), y el esquive `VEER` para la variante karpinchus (parar 3 s frente a la caja, rodear sin invertir el rumbo, reacoplar la pared).

### Seguimiento de pared

Dos métodos, con fallback automático:

1. **Regresión de línea** (`fit_wall_line`, principal): ajusta una recta por mínimos cuadrados a todos los puntos LiDAR del lado seguido (con rechazo iterativo de outliers), corrigiendo ángulo + distancia a la vez. Mucho más robusto al ruido que un solo rayo — evita que el robot "vaya en diagonal".
2. **Sector puro + alineación de 2 puntos** (`wall_align`, respaldo): si no hay suficientes puntos LiDAR para un ajuste confiable (pasillo abierto, esquina), cae de vuelta a este método más simple.

## 4. Parámetros y despliegue

Dependencias (además de ROS 2 Humble, ya presentes en la imagen del robot): `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, `cv_bridge`, `image_transport`, `numpy`, `opencv-python`.

Todos los parámetros de bajo nivel (distancia objetivo a la pared, ganancias, umbrales de frente/lado, geometría del robot, PARE/META, métricas) viven en `config/granprix_params.yaml` — editar ahí entre corridas, no en el código.

### Copiar el código al robot (sin compilar)

`colcon build`/`ros2 run` requieren "instalar" el paquete dentro del contenedor del robot; repetir eso en cada iteración puede terminar corrompiendo esa carpeta. Para desarrollo/pruebas, este paquete **no necesita compilarse**: no usa mensajes personalizados, así que cada nodo corre como script de Python plano (igual que `lane_node.py` en el flujo anterior del equipo).

Desde PowerShell (laptop):
```powershell
ssh pi@10.42.0.1 "rm -rf ~/yahboomcar_ws/src/capytown_G4_s13"
scp -r "C:\Users\alexp\Documents\GitHub\capytown_G4_s13" pi@10.42.0.1:~/yahboomcar_ws/src/
```
Para probar un cambio de código, basta con repetir el `scp` y volver a correr el nodo — nunca hace falta `colcon build` para iterar.

**Compilar de verdad (`colcon build`) solo es necesario** para `ros2 launch`/`ros2 run` (sección "todo junto" más abajo) — úsalo cuando el código ya esté probado, no en cada iteración:
```bash
cd ~/yahboomcar_ws
colcon build --packages-select capytown_granprix_pkg --symlink-install
source install/setup.bash
```

### Calibración de odometría

El odómetro del robot puede sobreestimar distancia y ángulo girado de forma consistente. `factor_dist_odom` y `factor_ang_odom` (en `granprix_params.yaml`, por defecto `1.0`) corrigen esa escala; el procedimiento de calibración en pista (empujar el robot a mano una distancia/ángulo conocido y comparar contra `/odom_raw`) está documentado en los comentarios junto a esos parámetros en `maze_solver.py`.

## 5. Ejecución

### Modo desarrollo — sin compilar, mínimas terminales (recomendado para iterar)

Dos terminales en el robot (RealVNC) + el navegador de la laptop para ver lo que ve el robot (reemplaza `rqt_image_view`/VNC).

**Terminal 1 — mantiene vivo el contenedor + bringup (chasis/IMU/LiDAR, driver del fabricante, ya compilado):**
```bash
sh ros2_humble.sh
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

**Terminal 2 — cámara + nodos propios, todo en una sola terminal con `&`:**
```bash
docker ps -a
docker exec -it <ID_DEL_CONTENEDOR> /bin/bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
cd ~/yahboomcar_ws/src/capytown_G4_s13

ros2 launch usb_cam camera.launch.py &
sleep 2
python3 -m capytown_granprix_pkg.pare_detector --ros-args --params-file config/granprix_params.yaml &
python3 -m capytown_granprix_pkg.box_detector --ros-args --params-file config/granprix_params.yaml &
python3 -m capytown_granprix_pkg.web_dashboard --ros-args --params-file config/granprix_params.yaml &
```
Abre en la laptop **http://10.42.0.1:8080/**: LiDAR, cámara con overlay de PARE, botón Pausar y ajuste de parámetros en caliente, sin terminal ni VNC extra.

Cuando ya probaste sensores y quieras que el robot navegue, en la misma Terminal 2:
```bash
python3 -m capytown_granprix_pkg.maze_solver --ros-args --params-file config/granprix_params.yaml &
```

Posición de cámara (opcional, una sola vez, misma terminal):
```bash
ros2 topic pub /servo_s1 std_msgs/msg/Int32 'data: 20' --once
ros2 topic pub /servo_s2 std_msgs/msg/Int32 'data: -55' --once
```

Para parar: `jobs` + `kill %1 %2 %3 %4`, o cerrar la pestaña de la Terminal 2 (no apaga el contenedor).

> `-m capytown_granprix_pkg.<nodo>` (en vez de `python3 <nodo>.py` suelto) es necesario porque `web_dashboard.py` importa funciones de `box_detector.py`/`maze_solver.py`; correrlo así desde la raíz del repo resuelve ese import sin necesidad de instalar el paquete. `--ros-args --params-file` carga `config/granprix_params.yaml` sin pasar por el launch file.

### Todo junto (requiere haber compilado, sección anterior)

**Un solo comando (tmux):**
```bash
bash ~/run_granprix.sh
```
Levanta bringup, cámara, `maze_solver` + `pare_detector` + `box_detector`, el estado de la FSM en vivo (`/maze_state`) y las ventanas de RViz / overlay de PARE. Para detener: `tmux kill-session -t granprix`.

**Manual / por partes:**
```bash
ros2 launch capytown_granprix_pkg granprix.launch.py \
  ronda:=1 side:=right meta_enabled:=false enable_karpinchus:=true show_dashboard:=true
```

Argumentos más comunes: `ronda` (1=exploración, 2=time attack), `side` (right/left), `meta_enabled`/`meta_x`/`meta_y`, `enable_karpinchus`, `show_map` (visor Tk), `show_dashboard` (panel web en `http://<IP-robot>:8080/`).

## 6. Métricas y entregables

Al llegar a META (o al cerrar el nodo) se registra una fila en `metricas_granprix.csv` con el esquema de la rúbrica: `ronda, llego_meta, tiempo_s, long_ruta_cm, long_optima_cm, eficiencia, colisiones, pare_reales, pare_detectados, pare_respetados, pare_falsos, dead_ends_visitados, karpinchus_rodeados`.

Entregables del reto (ver rúbrica completa en `Doc/CapyTown_GranPrix_Laberinto.docx`): código documentado con reutilización explícita de `box_detector` y el detector de PARE por cámara, CSV de métricas de ambas rondas, captura de RViz (trayectoria + markers de intersecciones/PARE), captura de la cámara con una detección de PARE, video de la corrida completa, paper IEEE corto y defensa individual por rol.

## 7. Créditos

Basado en el controlador de seguimiento de pared / dead-end / rodeo de cajas del Reto Clasificatorio 4 ("El Censo y el Guardián de las Cajas"), con la fusión de cámara (PARE/META), métricas y RViz agregadas para el Gran Prix, y el seguimiento de pared por regresión de línea y la calibración de escala de odometría sumados encima.
