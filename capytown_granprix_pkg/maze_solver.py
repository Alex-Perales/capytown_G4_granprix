#!/usr/bin/env python3
"""
CapyTown Gran Prix — maze_solver.py — fusión LiDAR (navegación) + cámara (PARE).
================================================================================
Cruza el laberinto ("El Qhapaq Ñan de CapyTown") con LiDAR (/scan) + odometría
(/odom_raw) para la geometría (seguimiento de pared, intersecciones, dead-ends),
y con una señal de cámara (/pare_detectado, std_msgs/Bool, publicada por
pare_detector.py) para la semántica (detener por completo ante un cartel PARE).

Este nodo es una ADAPTACIÓN de capytown_maze_pkg/maze_navigator.py (reto RC-4,
"El Censo y el Guardián de las Cajas", robot 9/Henry) — se reutiliza tal cual el
controlador de seguimiento de pared/dead-end/rodeo de cajas (ya depurado y
probado), y se añade ENCIMA la fusión con la cámara y las métricas del Gran Prix:

  * PARE (cámara): mientras `pare_flag` esté activo, la FSM entra en
    PARAR_PARE y se detiene ~pare_wait_t segundos ANTES de continuar —
    "la cámara manda para detener; el LiDAR manda para moverse/centrar"
    (regla de arbitraje del reto).
  * META: al entrar en un radio `meta_radius` de (meta_x, meta_y) en el marco
    de /odom_raw, el robot se detiene y se registra la corrida.
  * Métricas: registra metricas_granprix.csv con el esquema de la rúbrica
    (ronda, llego_meta, tiempo_s, long_ruta_cm, long_optima_cm, eficiencia,
    colisiones, pare_reales/detectados/respetados/falsos, dead_ends_visitados,
    karpinchus_rodeados).
  * Estado en vivo: publica /maze_state (std_msgs/String) para verlo con
    `ros2 topic echo /maze_state` en una terminal de estado.
  * RViz: publica /granprix_markers (visualization_msgs/MarkerArray) con un
    marcador por intersección tomada y por PARE respetado.

States:
  FOLLOW_WALL : keep the followed wall at TARGET_DIST using a PD controller on
                lateral error; drive forward while front is clear.
  TURN_IN     : front blocked AND followed-wall side blocked -> turn AWAY from
                the wall (into open space) by 90°, measured via odom yaw.
  TURN_OUT    : the followed wall disappeared (opening/corner) -> turn TOWARD
                the wall by 90° and creep forward to re-acquire it.
  RECOVER     : boxed in on 3 sides (dead-end) -> rotate 180°.
  PARAR_PARE  : cámara detectó un cartel PARE -> detención completa ~3s.

Design notes (lessons from CapyTown lane-controller):
  * Sign discipline: positive lateral error (too far from wall) -> steer toward
    wall; we keep the convention explicit and unit-test it.
  * Hysteresis on "front blocked" / "wall lost" so we don't chatter at borders.
  * All /scan ranges sanitized (drop nan/inf/<=0, clamp to range_max).
  * Turns are odom-yaw closed-loop (turn-by-effect), not timed.

Pure logic (sector extraction, state transition, control law) is importable and
unit-tested WITHOUT ROS via the helpers at the bottom.
"""
from __future__ import annotations
import csv
import math
import os

# ---- ROS imports guarded so the logic can be unit-tested without ROS ----
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32, Bool, String
    try:
        from visualization_msgs.msg import Marker, MarkerArray
        _HAVE_VIZ = True
    except Exception:
        _HAVE_VIZ = False
    _HAVE_ROS = True
except Exception:  # pragma: no cover - allows import on a machine without ROS
    _HAVE_ROS = False
    _HAVE_VIZ = False
    Node = object  # type: ignore
    Int32 = None  # type: ignore

# loop-completion — no se usa en el Gran Prix (la META reemplaza la "vuelta
# completa" del reto de cajas/lazo), pero se deja importado por si algún día
# se reutiliza este mismo paquete para ese otro reto.
try:
    from capytown_granprix_pkg.loop_completion import LoopCompletion
except Exception:
    try:
        from .loop_completion import LoopCompletion  # type: ignore
    except Exception:
        LoopCompletion = None  # type: ignore

# Split & Merge — extracción de líneas LiDAR; pure module, guarded, aditivo.
try:
    from capytown_granprix_pkg import split_merge as _sm
except Exception:
    try:
        from . import split_merge as _sm  # type: ignore
    except Exception:
        _sm = None  # type: ignore


def _sensor_qos():
    """BEST_EFFORT/volatile QoS — matches how LiDAR drivers & micro-ROS publish
    /scan and /odom_raw. A RELIABLE subscriber against a BEST_EFFORT publisher gets
    ZERO messages silently (the robot just never moves). This is the fix."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )


# ─────────────────────────── pure helpers ───────────────────────────
def sanitize(r, range_min, range_max):
    """Map one raw LiDAR range to a usable float (range_max if invalid/empty)."""
    if r is None:
        return range_max
    try:
        x = float(r)
    except (TypeError, ValueError):
        return range_max
    if math.isnan(x) or math.isinf(x) or x <= 0.0:
        return range_max
    if x < range_min:
        return range_min
    if x > range_max:
        return range_max
    return x


def sector_min(ranges, angle_min, angle_inc, lo_deg, hi_deg,
               range_min, range_max):
    """
    Minimum sanitized distance over [lo_deg, hi_deg] (degrees, robot frame,
    0°=forward, +CCW). Robust to a 360° MS200 scan and to wrap-around.
    Using the MINIMUM (closest obstacle) per sector is the safe choice for
    collision-aware wall following.
    """
    if not ranges:
        return range_max
    lo = math.radians(lo_deg)
    hi = math.radians(hi_deg)
    best = range_max
    n = len(ranges)
    for i in range(n):
        a = angle_min + i * angle_inc
        # normalize angle to [-pi, pi] for comparison
        aa = math.atan2(math.sin(a), math.cos(a))
        if lo <= aa <= hi:
            d = sanitize(ranges[i], range_min, range_max)
            if d < best:
                best = d
    return best


def sector_robust(ranges, angle_min, angle_inc, lo_deg, hi_deg,
                  range_min, range_max, drop=1, offset_deg=0.0):
    """
    Like sector_min but robust to a single spurious close ray: collect all
    sanitized ranges in the sector, drop the `drop` closest, return the min of
    the rest. A lone phantom return at 0.06 m no longer forces a false
    obstacle/stop (FABLE/ALICE pt2). Falls back to sector_min if too few rays.
    """
    if not ranges:
        return range_max
    lo = math.radians(lo_deg)
    hi = math.radians(hi_deg)
    vals = []
    for i in range(len(ranges)):
        a = angle_min + i * angle_inc - math.radians(offset_deg)
        aa = math.atan2(math.sin(a), math.cos(a))
        if lo <= aa <= hi:
            vals.append(sanitize(ranges[i], range_min, range_max))
    if not vals:
        return range_max
    vals.sort()
    if len(vals) > drop + 1:
        return vals[drop]        # k-th smallest after dropping `drop` closest
    return vals[0]


def yaw_from_quat(x, y, z, w):
    """Z-axis yaw from a quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def ang_diff(target, current):
    """Shortest signed angular difference target-current in [-pi, pi]."""
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def coerce_param(value, default):
    """Coerce ROS launch string overrides back to the DEFAULTS type."""
    if isinstance(default, bool):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)
    if isinstance(default, int) and not isinstance(default, bool):
        try:
            return int(value)
        except (TypeError, ValueError):
            return default
    if isinstance(default, float):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default
    return value


class Sectors:
    """Container for the three decision distances."""
    __slots__ = ("front", "left", "right")

    def __init__(self, front, left, right):
        self.front = front
        self.left = left
        self.right = right


def is_loop_boxes_mode(p) -> bool:
    """True for the current CapyTown boxes-loop challenge.

    That course is an oval/lazo around a central island with loose boxes in the
    corridor. It is not a maze with dead ends, so a front obstacle must not be
    interpreted as "boxed in -> 180° RECOVER".
    """
    return str(p.get("course_mode", "maze")).lower() in ("loop_boxes", "boxes_loop", "cajas")


def decide_state(prev_state, s: Sectors, p, front_blocked=None) -> str:
    """
    Pure state-transition function (testable). `p` exposes front_block,
    front_clear, wall_block, wall_lost, side ('right'/'left').
    `front_blocked`: if provided (the node computes it WITH hysteresis —
    enter at front_block, leave at front_clear), it is used directly; if None,
    falls back to a plain front<=front_block test (keeps unit tests stable).
    Priority: boxed->RECOVER, wall opening->TURN_OUT (take the corner, right-hand
    rule), front blocked->TURN_IN, else FOLLOW_WALL.
    """
    side = p["side"]
    wall = s.right if side == "right" else s.left
    other = s.left if side == "right" else s.right
    fb = (s.front <= p["front_block"]) if front_blocked is None else front_blocked

    boxed = (fb and wall <= p["wall_block"] and other <= p["wall_block"])
    if boxed:
        if is_loop_boxes_mode(p):
            return "TURN_IN"
        return "RECOVER"
    # opening on the followed side -> go grab the corner (TURN_OUT)
    if wall >= p["wall_lost"]:
        return "TURN_OUT"
    # front blocked -> turn away from wall
    if fb:
        return "TURN_IN"
    return "FOLLOW_WALL"


def follow_cmd(s: Sectors, p, prev_err=0.0, dt=0.0):
    """
    PD wall-follow control law -> (linear, angular). Pure/testable.
    error = wall_target - wall_dist ; positive error = too close -> steer away;
    negative = too far -> steer toward wall. Sign convention unit-tested.

    Damping (kd) + deadband added to kill the surf/weave a P-only law produces
    (it hunts around the setpoint). The derivative is fed via prev_err+dt, which
    the node tracks across ticks. Defaults prev_err=0.0, dt=0.0 keep this PURE
    and testable: with dt=0 the D term is exactly 0 -> unit tests unchanged.
    """
    side = p["side"]
    wall = s.right if side == "right" else s.left
    err = p["wall_target"] - wall              # +: too close, -: too far
    # deadband: ignore micro-error near target so we drive STRAIGHT instead of
    # hunting around the setpoint (a prime cause of the weave/"surf").
    db = p.get("deadband", 0.0)
    err_p = 0.0 if abs(err) < db else err
    # derivative term damps the oscillation; derr from the node-tracked prev_err.
    derr = ((err - prev_err) / dt) if dt > 1e-6 else 0.0
    # steer sign: for RIGHT-wall following, positive angular = turn left (away
    # from right wall) when too close (err>0). For LEFT-wall, mirror it.
    sign = 1.0 if side == "right" else -1.0
    angular = sign * (p["kp"] * err_p + p.get("kd", 0.0) * derr)
    # slow down when front is tight
    lin = p["v_max"]
    if s.front < p["front_slow"]:
        frac = max(0.0, (s.front - p["front_block"]) /
                   max(1e-3, (p["front_slow"] - p["front_block"])))
        lin = p["v_min"] + (p["v_max"] - p["v_min"]) * frac
    # clamp angular
    amax = p["w_max"]
    angular = clamp(angular, -amax, amax)
    return lin, angular


def should_hold_straight(s: Sectors, p, front_blocked=False) -> bool:
    """
    When there is no useful side wall and the front is clear, do not force a
    right-hand TURN_OUT. Hold the current odom heading and drive straight until
    a wall is reacquired. This prevents the "spin in open/ambiguous space"
    failure while keeping normal wall-following whenever a wall exists.
    """
    no_wall_visible = (
        p.get("straight_when_no_wall", True)
        and not front_blocked
        and s.front >= p["front_clear"]
        and s.left >= p["wall_lost"]
        and s.right >= p["wall_lost"]
    )
    if no_wall_visible:
        return True
    if not p.get("open_space_straight", True):
        return False
    obstacle_detect = max(p.get("obstacle_detect", p["front_slow"]), p["front_slow"])
    return (
        not front_blocked
        and s.front >= obstacle_detect
        and s.left >= p["wall_lost"]
        and s.right >= p["wall_lost"]
    )


def localized_front_obstacle(front, left_shoulder, right_shoulder, p) -> bool:
    """Return True when the frontal hit looks like a box, not a wall/corner.

    A wall across the corridor tends to occupy the center and both front
    shoulders at similar distance. A loose box is more localized: the center is
    closer, while the shoulders still see open space/wall behind it. This gate
    prevents the veer controller from stealing normal wall-follow corner logic.
    """
    detect = p.get("obstacle_detect", p["front_slow"])
    if front >= detect:
        return False
    margin = p.get("box_shoulder_margin", 0.12)
    if left_shoulder <= front + margin:
        return False
    if right_shoulder <= front + margin:
        return False
    return True


def wall_parallel_error(front_side, back_side, side, p):
    """Signed alignment error from two LiDAR hits on the followed wall.

    For the right wall, front<back means the nose is pointing into the wall and
    the correction must turn left (+angular). For the left wall the sign mirrors.
    Returns None when the side wall is not close enough to be trusted.
    """
    max_range = p.get("wall_align_max_range", p.get("wall_lost", 0.55))
    if front_side >= max_range or back_side >= max_range:
        return None
    sign = 1.0 if side == "right" else -1.0
    return sign * (back_side - front_side)


def wall_is_parallel(front_side, back_side, side, p) -> bool:
    err = wall_parallel_error(front_side, back_side, side, p)
    return err is not None and abs(err) <= p.get("wall_align_tol", 0.04)


def corner_pose_aligned(front, rear, followed_wall, p) -> bool:
    """True when a turn reached a corridor-corner pose.

    For Henry's right-wall course, a good post-corner pose often sees open front,
    a wall behind (the segment just left), and the followed wall on the right.
    That geometry is a better turn stop than blindly completing a nominal 90°.
    """
    if front < p.get("front_clear", 0.38):
        return False
    if rear > p.get("corner_rear_max", 0.45):
        return False
    if followed_wall > p.get("corner_side_max", p.get("wall_lost", 0.55)):
        return False
    return True


def heading_hold_cmd(current_yaw, target_yaw, p):
    """Drive forward while damping heading drift against an odom yaw reference."""
    err = ang_diff(target_yaw, current_yaw)
    limit = p.get("heading_w_max", min(0.45, p["w_max"]))
    ang = clamp(p.get("heading_kp", 1.2) * err, -limit, limit)
    return p["v_max"], ang


# ─────────────────────────── ROS node ───────────────────────────
DEFAULTS = dict(
    course_mode="maze",      # Gran Prix = laberinto real con dead-ends -> RECOVER 180 activo
    side="right",            # right-hand rule by default
    wall_target=0.20,        # LiDAR->wall distance confirmed for robot 9
    wall_block=0.14,         # side considered too close below this (m)
    wall_lost=0.55,          # side considered "opening" in ~60cm corridor
    wall_align_enabled=True, # use front/back side rays to keep robot parallel to right wall
    wall_align_tol=0.04,     # m: front-side vs back-side delta considered aligned
    wall_align_kp=1.2,       # angular correction per meter of side-wall skew
    wall_align_w_max=0.18,   # max angular correction from side-wall alignment
    wall_align_max_range=0.50,# only trust alignment while a side wall is actually visible
    corner_align_enabled=True,# stop 90° turns by geometry: front open + rear wall + side wall
    corner_rear_max=0.45,    # m: wall behind must be visible after a correct corner
    corner_side_max=0.38,    # m: followed wall must be visible on the side
    corner_min_turn_t=0.45,  # s: ignore corner detector at the very start of a turn
    front_block=0.30,        # front considered blocked below this (m)
    front_slow=0.50,         # start slowing when front below this (m)
    front_clear=0.38,        # hysteresis: front clear above this (m)
    kp=0.70, kd=1.60,        # PD suave: evita giro errático con target lateral corto
    deadband=0.03,           # m: narrow band around the 20cm LiDAR target
    v_max=0.18, v_min=0.06,  # m/s — más controlable en pasillo de ~60cm
    w_max=1.5,               # rad/s
    turn_speed=1.5,          # rad/s during 90/180 turns (subido de 0.9: + potencia de giro en esquinas, Henry)
    front_sector=12.0,       # +/- deg around 0 for FRONT; narrow avoids side-wall false TURN_IN
    front_angle_offset=0.0,  # deg: rota TODOS los sectores si el LiDAR está montado girado (0=el frente del LiDAR ya apunta al frente del robot). Si queda clavado en TURN_IN, probar 90/180/270 por efecto.
    side_sector_lo=50.0,     # side sector window (deg)
    side_sector_hi=100.0,
    range_min=0.12, range_max=8.0,
    control_hz=10.0,         # match MS200 ~10Hz (acting on stale scan -> overshoot)
    # --- physical guards (FABLE/JARVIS adversarial review) ---
    scan_timeout=0.5,        # s without /scan -> hard stop (blind-driving guard)
    require_odom=True,       # no odom -> no motion; turns and stuck watchdog need it
    odom_timeout=1.0,        # s without odom -> hard stop (stale pose guard)
    turn_timeout=6.0,        # s max per odom turn -> abort if odom frozen
    emerg_dist=0.15,         # m: front closer than this -> emergency stop
    sector_drop=2,           # robust sector: drop N closest rays (kill lone/dual phantom)
    front_block_persist=0.35,# s front must remain blocked before TURN_IN
    # --- positional stuck watchdog (ALICE) ---
    stuck_t=2.5,             # s of no positional progress while driving -> recover
    stuck_dpos=0.03,         # m: progress threshold
    recovery_t=1.0,          # s of reverse on stuck
    recover_persist=1.0,     # s que el dead-end debe PERSISTIR antes del 180° (un roce no dispara; FABLE)
    # --- loop completion (FABLE) wired into the FSM: auto-stop after one lap ---
    enable_loop_stop=True,   # stop for good after completing one loop of the circuit
    loop_away_dist=1.0,      # m to get away from START before arming the return
    loop_return_dist=0.40,   # m near START that counts as "returned"
    loop_min_path=6.0,       # m min path traveled to validate the lap (anti-jitter, ~loop perimeter; FABLE)
    # --- geometría del robot: los umbrales ADAPTATIVOS derivan de aquí, NO del laberinto ---
    # (robusto a cualquier ancho de pasillo el día de la prueba — punto de Henry; diseño NEXUS)
    robot_width=0.16,        # m, ancho del robot (Yahboom de Henry = 16cm)
    robot_length=0.22,       # m, largo del robot (22cm) — el clearance frontal usa el LARGO
    wall_clearance=0.12,     # m, borde->pared; da ~0.20m LiDAR->pared (Henry robot9)
    corridor_width=0.60,     # m, ancho aproximado del pasillo/laberinto (Henry robot9)
    veer_gain=1.2,           # ganancia del esquive lateral (VEER hacia el lado con más espacio)
    veer_min_w=0.55,         # giro minimo sostenido durante esquive (no micro-zigzag)
    veer_timeout=4.0,        # s máx del esquive comprometido antes de expirar (FABLE: si no libra, caer al flujo normal)
    veer_min_dist=0.85,      # m mínimo comprometido: margen extra contra roce de caja
    veer_min_t=2.3,          # s mínimo de rodeo aunque el frente se libere momentáneamente
    veer_out_angle=4.0,      # deg: tiny lane shift, not a box-circling turn
    veer_out_speed=0.10,     # m/s during OUT; shallow arc keeps the lane
    veer_out_kp=1.0,         # proportional yaw controller for gentle OUT arc
    veer_turn_speed=0.08,    # rad/s max during OUT; prevent 90/120-degree-looking dodge
    veer_pass_speed=0.08,    # m/s while committed alongside the box
    veer_max_yaw_delta=25.0, # deg: anti-180 guard during box bypass
    veer_finish_yaw_tol=25.0,# deg: parallel wall alone is not enough; heading must still match route
    veer_force_away_from_wall=True, # right-wall course: always dodge left, never into the wall box
    veer_back_enabled=False, # protrusion on right wall: pass straight, then let wall-follower reacquire
    veer_resume_t=0.0,       # protrusion mode: return control to wall-follower immediately
    veer_grace_t=0.8,        # suppress re-detecting the same protrusion while wall-follow reacquires
    post_veer_reacquire_t=3.0,    # s: after a right-wall box, do not take a fake TURN_OUT corner
    post_veer_reacquire_dist=0.30,# m: drive forward until the right wall is reacquired
    post_veer_wall_max=0.42,      # m: followed wall close enough to hand control back to wall-follow
    post_veer_w_max=0.10,         # rad/s: only gentle steering during reacquire, never a 90° turn
    veer_resume_speed=0.12,  # m/s durante RESUME/GRACE
    obstacle_detect=0.35,    # caja localizada al frente; no anticipar tanto que confunda pared
    obstacle_clear=0.65,     # frente suficientemente libre para terminar esquive
    box_shoulder_sector_lo=18.0, # shoulders used to distinguish localized box vs wall
    box_shoulder_sector_hi=45.0,
    box_shoulder_margin=0.12,
    enable_obstacle_veer=True, # variante karpinchus (opcional docente): rodear cajas en el pasillo
    disable_recover_180=False, # laberinto real: SÍ hay dead-ends -> permitir el 180° de RECOVER
    turn_in_max_accum_deg=135.0, # ANTI-CASCADA TURN_IN (ALICE): TURN_INs de 90 consecutivos SIN avance que sumen > esto = pocket -> reversa, no otro giro (evita 90+90=180). Cazado en log real de Henry.
    turn_in_reset_clear_t=3.0,   # s de FRENTE DESPEJADO sostenido que resetea el acumulador (escape real del rincon; > que el respiro ~2s del pocket). ALICE
    turn_in_restore_t=2.5,       # s de ventana para RESTAURAR el rumbo previo tras el corte anti-180 y seguir de frente en vez de retroceder (pedido Henry). ALICE
    box_stop_wait_t=3.0,         # s: GUARDIÁN — parar y ESPERAR frente a cada caja antes de rodear (rúbrica RC-4 IMP3). ALICE
    open_space_straight=False, # modo opcional; por defecto manda el wall-follower del laberinto
    straight_when_no_wall=True, # si no ve pared lateral útil, avanza recto hasta reacoplar
    heading_kp=1.2,          # correccion suave de rumbo recto
    heading_w_max=0.45,      # limite angular para no ondular en linea recta
    min_side_clearance=0.23, # lado libre minimo para esquivar una caja
    adaptive_geom=True,      # derivar umbrales de W/L (apagar = usar los fijos de arriba)
    debug_report_enabled=True, # write a decision report file on the robot
    debug_report_path="/tmp/capytown_maze_report.log",
    debug_report_period=0.5,   # s between periodic report lines
    # ---------------- Gran Prix: fusión cámara (PARE) ----------------
    pare_topic="/pare_detectado",      # std_msgs/Bool, publicado por pare_detector.py
    pare_wait_t=3.0,                   # s de detención completa ante un PARE (regla del reto)
    pare_cooldown_t=2.5,                # s de "no re-disparar" tras respetar un PARE (mismo cartel)
    # ---------------- Gran Prix: META (esquina opuesta a INICIO) ----------------
    meta_x=0.0,                        # m, coordenada X de META en el marco de odom_topic
    meta_y=0.0,                        # m, coordenada Y de META en el marco de odom_topic
    meta_radius=0.35,                  # m, radio de llegada considerado "META alcanzada"
    meta_enabled=False,                # activar solo cuando meta_x/meta_y se hayan medido en la pista
    # ---------------- Gran Prix: métricas (metricas_granprix.csv) ----------------
    ronda=1,                            # 1=exploración, 2=time attack
    run_id=1,
    metrics_csv="/tmp/metricas_granprix.csv",
    long_optima_cm=0.0,                 # cm, longitud de la ruta más corta (medida sobre el plano)
    pare_reales=0,                      # cuántas señales PARE hay realmente en esta corrida
    colisiones_manual=-1,               # -1 = sin dato (completar viendo el video/ros2 bag)
    pare_falsos_manual=-1,              # -1 = sin dato (completar viendo el video/ros2 bag)
    # ---------------- Gran Prix: estado en vivo + RViz ----------------
    maze_state_topic="/maze_state",     # std_msgs/String
    markers_topic="/granprix_markers",  # visualization_msgs/MarkerArray
    cerca_interseccion_topic="/cerca_interseccion",  # std_msgs/Bool -> zona de atención cámara
)


if _HAVE_ROS:
    class MazeSolver(Node):
        def __init__(self):
            super().__init__("maze_solver")
            self.p = dict(DEFAULTS)
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v)
                self.p[k] = coerce_param(self.get_parameter(k).value, v)
            if is_loop_boxes_mode(self.p):
                self.p["enable_obstacle_veer"] = True
                self.p["disable_recover_180"] = True
            self.scan_topic = self.declare_get("scan_topic", "/scan")
            self.odom_topic = self.declare_get("odom_topic", "/odom_raw")
            self.cmd_topic = self.declare_get("cmd_vel_topic", "/cmd_vel")
            self.count_topic = self.declare_get("count_topic", "/cajas_avistadas")
            # --- UMBRALES ADAPTATIVOS derivados de la geometría del robot (diseño NEXUS) ---
            # El LiDAR mide desde ~el centro. Anclar al robot (W/L), no al laberinto, hace que
            # funcione a cualquier ancho de pasillo (robusto el día de la prueba — Henry).
            W = float(self.p["robot_width"]); L = float(self.p["robot_length"])
            clearance = float(self.p.get("wall_clearance", 0.12))
            corridor = float(self.p.get("corridor_width", 0.60))
            self.robot_diag = math.hypot(W, L)            # pasillo mínimo para rotar en sitio (~0.27 con W/L de Henry)
            if self.p.get("adaptive_geom", True):
                # /scan mide desde el LiDAR (~centro), no desde el borde. Henry confirmó
                # 20cm de LiDAR a pared; con W=16cm eso equivale a ~12cm del borde.
                self.p["wall_target"] = max(W / 2 + clearance, W / 2 + 0.04)
                self.p["wall_block"]  = max(W / 2 + 0.04, min(self.p["wall_target"] - 0.03, W / 2 + clearance * 0.5))
                self.p["front_block"] = max(L / 2 + 0.18, self.p["front_block"])
                self.p["front_slow"]  = max(self.p["front_slow"], self.p["front_block"] + 0.18)
                self.p["front_clear"] = max(self.p["front_clear"], self.p["front_block"] + 0.08)
                # Keep obstacle_detect independent from front_slow. For the
                # boxes-loop challenge, too large a detect distance makes
                # normal walls/corners look like boxes.
                self.p["obstacle_clear"] = max(self.p.get("obstacle_clear", 0.0),
                                               self.p["obstacle_detect"] + 0.15)
                self.p["emerg_dist"]  = max(L / 2 + 0.05, self.p["emerg_dist"])
                # (el borde delantero está a L/2 del LiDAR; usar W/2 lateral dejaba chocar antes de parar)
                # En pasillo de ~60cm, una pared seguida a >~55cm ya es apertura real.
                self.p["wall_lost"] = min(self.p["wall_lost"], max(0.45, corridor - 0.05))
                self.get_logger().info(
                    f"[adaptive] W={W:.2f} L={L:.2f} diag={self.robot_diag:.2f} "
                    f"wall_target={self.p['wall_target']:.2f} wall_lost={self.p['wall_lost']:.2f} "
                    f"front_block={self.p['front_block']:.2f}")
            # loop-completion: no aplica al Gran Prix (la META lo reemplaza) -> deshabilitado.
            self.loop = None
            self.loop_done = False
            self.boxes_count = 0      # census from box_detector (/cajas_avistadas)

            # ---------------- Gran Prix: fusión PARE (cámara) ----------------
            self.pare_flag = False        # último valor recibido en /pare_detectado
            self.pare_phase = "IDLE"      # IDLE | STOPPING
            self.pare_stop_until = 0.0
            self.pare_cooldown_until = 0.0
            self.pare_detectados = 0
            self.pare_respetados = 0

            # ---------------- Gran Prix: META ----------------
            self.meta_reached = False
            self.start_time = None
            self.total_path_m = 0.0
            self._last_odom_pos = None
            self._metrics_written = False

            # ---------------- Gran Prix: contadores de rúbrica ----------------
            self.dead_ends_visitados = 0
            self.karpinchus_rodeados = 0

            # ---------------- Gran Prix: RViz markers ----------------
            self._marker_list = []
            self._marker_id = 0

            self.state = "FOLLOW_WALL"
            self.scan = None
            self.scan_stamp = None       # wall-clock(s) of last /scan (stale guard)
            self.yaw = 0.0
            self.odom_stamp = None       # wall-clock(s) of last odom (stale guard)
            self.turn_target = None      # target yaw during a turn
            self.turn_start = None       # wall-clock(s) turn began (watchdog)
            self.have_odom = False
            self.fault_reason = None     # set on non-recoverable safety faults
            self._warned_no_odom = False
            self._warned_odom_stale = False
            self.prev_front_blocked = False   # front hysteresis memory
            self.front_block_time = 0.0       # anti-falso TURN_IN: bloqueo frontal debe persistir
            self.turn_in_accum_deg = 0.0      # ANTI-CASCADA (ALICE): grados TURN_IN acumulados sin escapar del rincon
            self.front_clear_run = 0.0        # s de frente DESPEJADO sostenido -> resetea el acumulador (escape real, no avance-hacia-obstaculo)
            self.turn_in_start_yaw = None     # rumbo al INICIAR una secuencia de TURN_IN (para restaurar tras el corte, Henry)
            self.restore_until = 0.0          # ventana de restauracion de rumbo tras el corte anti-180
            self.restore_yaw_target = None
            self.box_stop_until = 0.0   # GUARDIÁN (rúbrica RC-4 IMP3): timer de la parada 3s frente a caja (ALICE)
            self.box_stopped = False    # ya cumplió la parada 3s para la caja actual (no re-parar durante el rodeo)
            # positional stuck watchdog (ALICE)
            self.pos = None
            self.ref_pos = None
            self.stuck_time = 0.0
            self.recovery = 0.0
            self.deadend_time = 0.0   # acumula s que el dead-end persiste (gate del 180°, FABLE)
            self.veering = False      # VEER-COMMIT: esquive sostenido hasta SALIR del obstáculo
            self.veer_phase = "IDLE"  # OUT -> PASS -> BACK -> resume/grace
            self.veer_sign = 0.0      # lado del esquive comprometido (+izq/-der)
            self.veer_start = None    # pos donde arrancó el esquive (medir avance por odom)
            self.veer_start_time = 0.0 # wall-clock(s) del inicio del esquive (timeout por tiempo, FABLE)
            self.veer_phase_time = 0.0
            self.veer_entry_yaw = None # yaw al iniciar rodeo; RESUME vuelve a este rumbo
            self.veer_out_yaw = None
            self.veer_resume_until = 0.0
            self.veer_grace_until = 0.0
            self.post_veer_until = 0.0
            self.post_veer_start = None
            self.heading_ref = None    # odom yaw reference for straight open-course driving
            self._report_last = 0.0
            self._report_last_key = None
            self.report_path = str(self.p.get("debug_report_path", "/tmp/capytown_maze_report.log"))
            if self.p.get("debug_report_enabled", True):
                try:
                    os.makedirs(os.path.dirname(self.report_path) or ".", exist_ok=True)
                    with open(self.report_path, "w", encoding="utf-8") as f:
                        f.write("# capytown_maze_pkg decision report\n")
                        f.write("# time label state phase yaw_deg front left right shL shR rf rb rear extra\n")
                except Exception as exc:
                    self.get_logger().warn(f"cannot create report file {self.report_path}: {exc}")

            qos = _sensor_qos()
            self.sub_scan = self.create_subscription(
                LaserScan, self.scan_topic, self.on_scan, qos)
            self.sub_odom = self.create_subscription(
                Odometry, self.odom_topic, self.on_odom, qos)
            if Int32 is not None:
                self.sub_count = self.create_subscription(
                    Int32, self.count_topic, self.on_count, 10)
            self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

            # ---------------- Gran Prix: fusión cámara + estado + RViz ----------------
            self.pare_topic = self.declare_get("pare_topic", "/pare_detectado")
            self.sub_pare = self.create_subscription(Bool, self.pare_topic, self.on_pare, 10)
            self.cerca_topic = self.declare_get("cerca_interseccion_topic", "/cerca_interseccion")
            self.pub_cerca = self.create_publisher(Bool, self.cerca_topic, 10)
            self.state_topic = self.declare_get("maze_state_topic", "/maze_state")
            self.pub_state = self.create_publisher(String, self.state_topic, 10)
            if _HAVE_VIZ:
                self.markers_topic = self.declare_get("markers_topic", "/granprix_markers")
                self.pub_markers = self.create_publisher(MarkerArray, self.markers_topic, 10)
            else:
                self.pub_markers = None

            # ---------------- Gran Prix: métricas ----------------
            self.ronda = int(self.p.get("ronda", 1))
            self.run_id = int(self.p.get("run_id", 1))
            self.metrics_csv = str(self.p.get("metrics_csv", "/tmp/metricas_granprix.csv"))
            self.meta_x = float(self.p.get("meta_x", 0.0))
            self.meta_y = float(self.p.get("meta_y", 0.0))
            self.meta_radius = float(self.p.get("meta_radius", 0.35))
            self.meta_enabled = bool(self.p.get("meta_enabled", False))

            self.timer = self.create_timer(1.0 / self.p["control_hz"], self.tick)
            self.get_logger().info(
                f"MazeSolver up: mode={self.p['course_mode']} side={self.p['side']} scan={self.scan_topic} "
                f"odom={self.odom_topic} cmd={self.cmd_topic} pare={self.pare_topic} "
                f"meta_enabled={self.meta_enabled} meta=({self.meta_x:.2f},{self.meta_y:.2f})±{self.meta_radius:.2f} "
                f"report={self.report_path}")

        def declare_get(self, name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        def now_s(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_scan(self, msg):
            self.scan = msg
            self.scan_stamp = self.now_s()
            # Split & Merge (JARVIS, reto Henry): extrae líneas (paredes) + cajas del scan.
            # ADITIVO — NO cambia el wall-follower (tick); expone self.sm_lines/self.sm_boxes
            # para censo/uso futuro. Fail-safe: nunca rompe on_scan si _sm falla o no está.
            if _sm is not None:
                try:
                    pts = _sm.scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                             getattr(msg, "range_min", 0.12),
                                             getattr(msg, "range_max", 12.0))
                    self.sm_lines = _sm.split_and_merge(pts)
                    self.sm_boxes = _sm.detect_boxes(self.sm_lines)
                    self._sm_ticks = getattr(self, "_sm_ticks", 0) + 1
                    if self._sm_ticks % 20 == 0:   # log throttled (~cada 2s), no floodear
                        self.get_logger().info(
                            f"[Split&Merge] lineas={len(self.sm_lines)} cajas={len(self.sm_boxes)}")
                except Exception:
                    pass

        def on_odom(self, msg):
            q = msg.pose.pose.orientation
            self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            pp = msg.pose.pose.position
            self.pos = (pp.x, pp.y)
            self.have_odom = True
            self.odom_stamp = self.now_s()
            if self.heading_ref is None:
                self.heading_ref = self.yaw
            self._warned_no_odom = False
            self._warned_odom_stale = False
            if self.start_time is None:
                self.start_time = self.now_s()

            # Gran Prix: acumula longitud de ruta recorrida (odometría) — para
            # long_ruta_cm / eficiencia en metricas_granprix.csv.
            if self._last_odom_pos is not None:
                d = math.hypot(pp.x - self._last_odom_pos[0], pp.y - self._last_odom_pos[1])
                # ignora saltos absurdos (glitch de odom) para no inflar la métrica
                if d < 0.5:
                    self.total_path_m += d
            self._last_odom_pos = (pp.x, pp.y)

            # Gran Prix: ¿llegó a META? (esquina opuesta a INICIO, medida en el marco odom)
            if self.meta_enabled and not self.meta_reached:
                if math.hypot(pp.x - self.meta_x, pp.y - self.meta_y) <= self.meta_radius:
                    self.meta_reached = True
                    self.get_logger().info("🏁 META alcanzada — deteniendo y registrando métricas.")
                    self.add_marker("META", (pp.x, pp.y), (0.1, 0.9, 0.2))

        def on_count(self, msg):
            """Census de cajas (box_detector -> /cajas_avistadas), para reportarlo al parar."""
            try:
                self.boxes_count = int(msg.data)
            except Exception:
                pass

        def on_pare(self, msg):
            """Cámara -> ¿hay un cartel PARE en el campo de visión ahora mismo?"""
            try:
                self.pare_flag = bool(msg.data)
            except Exception:
                self.pare_flag = False

        # ---------------- Gran Prix: helpers RViz / métricas ----------------
        def add_marker(self, ns, xy, color=(1.0, 0.5, 0.0)):
            if not self.pub_markers:
                return
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = self._marker_id
            self._marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 1.0
            self._marker_list.append(m)
            arr = MarkerArray()
            arr.markers = self._marker_list
            self.pub_markers.publish(arr)

        def write_metrics(self, llego_meta=None):
            """Registra una fila en metricas_granprix.csv (rúbrica del Gran Prix).
            Se llama al llegar a META o al cerrar el nodo (Ctrl+C) para que SIEMPRE
            quede registro, incluso de corridas incompletas."""
            if self._metrics_written:
                return
            self._metrics_written = True
            header = ["ronda", "llego_meta", "tiempo_s", "long_ruta_cm", "long_optima_cm",
                      "eficiencia", "colisiones", "pare_reales", "pare_detectados",
                      "pare_respetados", "pare_falsos", "dead_ends_visitados",
                      "karpinchus_rodeados"]
            tiempo_s = (self.now_s() - self.start_time) if self.start_time else 0.0
            long_ruta_cm = self.total_path_m * 100.0
            long_optima_cm = float(self.p.get("long_optima_cm", 0.0))
            eficiencia = (long_optima_cm / long_ruta_cm) if (long_optima_cm > 0 and long_ruta_cm > 0) else ""
            colisiones = self.p.get("colisiones_manual", -1)
            colisiones = "" if colisiones is None or int(colisiones) < 0 else int(colisiones)
            pare_falsos = self.p.get("pare_falsos_manual", -1)
            pare_falsos = "" if pare_falsos is None or int(pare_falsos) < 0 else int(pare_falsos)
            if llego_meta is None:
                llego_meta = "Sí" if self.meta_reached else "No"
            row = [self.ronda, llego_meta, f"{tiempo_s:.1f}", f"{long_ruta_cm:.1f}",
                   f"{long_optima_cm:.1f}" if long_optima_cm > 0 else "",
                   f"{eficiencia:.3f}" if eficiencia != "" else "",
                   colisiones, int(self.p.get("pare_reales", 0)), self.pare_detectados,
                   self.pare_respetados, pare_falsos, self.dead_ends_visitados,
                   self.karpinchus_rodeados]
            try:
                os.makedirs(os.path.dirname(self.metrics_csv) or ".", exist_ok=True)
                new = not os.path.exists(self.metrics_csv) or os.path.getsize(self.metrics_csv) == 0
                with open(self.metrics_csv, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    if new:
                        w.writerow(header)
                    w.writerow(row)
                self.get_logger().info(f"📊 metricas_granprix.csv <- {row} ({self.metrics_csv})")
            except Exception as exc:
                self.get_logger().error(f"No pude escribir {self.metrics_csv}: {exc}")

        def sectors(self) -> Sectors:
            m = self.scan
            p = self.p
            d = int(p["sector_drop"])
            off = float(p.get("front_angle_offset", 0.0))  # rota TODOS los sectores al frente FÍSICO del robot si el LiDAR está montado girado
            rr = list(m.ranges)
            front = sector_robust(rr, m.angle_min, m.angle_increment,
                                  -p["front_sector"], p["front_sector"],
                                  p["range_min"], p["range_max"], drop=d, offset_deg=off)
            left = sector_robust(rr, m.angle_min, m.angle_increment,
                                 p["side_sector_lo"], p["side_sector_hi"],
                                 p["range_min"], p["range_max"], drop=d, offset_deg=off)
            right = sector_robust(rr, m.angle_min, m.angle_increment,
                                  -p["side_sector_hi"], -p["side_sector_lo"],
                                  p["range_min"], p["range_max"], drop=d, offset_deg=off)
            return Sectors(front, left, right)

        def front_shoulders(self):
            """Distances in front-left/front-right shoulders for box-vs-wall gating."""
            m = self.scan
            p = self.p
            d = int(p["sector_drop"])
            off = float(p.get("front_angle_offset", 0.0))
            rr = list(m.ranges)
            lo = p.get("box_shoulder_sector_lo", 18.0)
            hi = p.get("box_shoulder_sector_hi", 45.0)
            left = sector_robust(rr, m.angle_min, m.angle_increment,
                                 lo, hi, p["range_min"], p["range_max"],
                                 drop=d, offset_deg=off)
            right = sector_robust(rr, m.angle_min, m.angle_increment,
                                  -hi, -lo, p["range_min"], p["range_max"],
                                  drop=d, offset_deg=off)
            return left, right

        def side_wall_profile(self):
            """Return side-wall/rear hits: right_front, right_back, left_front, left_back, rear.

            Equal front/back distances on a side wall mean the robot is parallel
            to that wall. This is the geometric angle gate Henry asked for.
            """
            m = self.scan
            p = self.p
            d = int(p["sector_drop"])
            off = float(p.get("front_angle_offset", 0.0))
            rr = list(m.ranges)
            right_front = sector_robust(rr, m.angle_min, m.angle_increment,
                                        -70.0, -45.0, p["range_min"], p["range_max"],
                                        drop=d, offset_deg=off)
            right_back = sector_robust(rr, m.angle_min, m.angle_increment,
                                       -135.0, -110.0, p["range_min"], p["range_max"],
                                       drop=d, offset_deg=off)
            left_front = sector_robust(rr, m.angle_min, m.angle_increment,
                                       45.0, 70.0, p["range_min"], p["range_max"],
                                       drop=d, offset_deg=off)
            left_back = sector_robust(rr, m.angle_min, m.angle_increment,
                                      110.0, 135.0, p["range_min"], p["range_max"],
                                      drop=d, offset_deg=off)
            rear_a = sector_robust(rr, m.angle_min, m.angle_increment,
                                   160.0, 180.0, p["range_min"], p["range_max"],
                                   drop=d, offset_deg=off)
            rear_b = sector_robust(rr, m.angle_min, m.angle_increment,
                                   -180.0, -160.0, p["range_min"], p["range_max"],
                                   drop=d, offset_deg=off)
            rear = min(rear_a, rear_b)
            return right_front, right_back, left_front, left_back, rear

        def report_decision(self, label, s=None, shoulders=None, profile=None,
                            extra="", force=False):
            if not self.p.get("debug_report_enabled", True):
                return
            now = self.now_s()
            key = (label, self.state, self.veer_phase, extra)
            period = self.p.get("debug_report_period", 0.5)
            if not force and key == self._report_last_key and (now - self._report_last) < period:
                return
            self._report_last = now
            self._report_last_key = key
            front = left = right = float("nan")
            sh_l = sh_r = float("nan")
            rf = rb = rear = float("nan")
            if s is not None:
                front, left, right = s.front, s.left, s.right
            if shoulders is not None:
                sh_l, sh_r = shoulders
            if profile is not None:
                rf, rb, _lf, _lb, rear = profile
            line = (
                f"{now:.3f} {label} state={self.state} phase={self.veer_phase} "
                f"yaw={math.degrees(self.yaw):.1f} front={front:.2f} left={left:.2f} right={right:.2f} "
                f"shL={sh_l:.2f} shR={sh_r:.2f} rf={rf:.2f} rb={rb:.2f} rear={rear:.2f} {extra}\n"
            )
            try:
                with open(self.report_path, "a", encoding="utf-8") as f:
                    f.write(line)
            except Exception:
                pass

        def publish(self, lin, ang):
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.pub.publish(t)

        def begin_turn(self, sign, degrees):
            """Arm an odom-closed-loop turn of `degrees` (sign=+CCW/-CW)."""
            self.turn_target = self.yaw + sign * math.radians(degrees)
            self._turn_sign = sign
            self.turn_start = self.now_s()

        def run_turn(self):
            """Return True while still turning; spins in place by odom."""
            if self.turn_target is None or not self.have_odom:
                return False
            # watchdog: if odom froze, err never converges -> abort, don't spin forever
            if (self.turn_start is not None and
                    (self.now_s() - self.turn_start) > self.p["turn_timeout"]):
                self.get_logger().error("turn timeout -> FAULT stop (restart node to clear)")
                self.turn_target = None
                self.turn_start = None
                self.state = "FAULT"
                self.fault_reason = "turn_timeout"
                self.publish(0.0, 0.0)
                return False
            err = ang_diff(self.turn_target, self.yaw)
            # Parar cuando ALCANZA o PASA el objetivo. Con turn_speed alto y ticks ~1s el giro
            # rota mucho por tick y SALTABA la ventana de 4 grados -> seguia hasta ~180 y solo
            # cortaba por timeout (bug REAL cazado en log v5: un solo TURN_IN de 90 rotaba a 170,
            # = el 180 de raiz). El chequeo de overshoot (err con signo opuesto al sentido de
            # giro = ya pasamos el objetivo) lo corta en ~90. ALICE.
            if abs(err) < math.radians(4.0) or (self._turn_sign * err < 0.0):
                self.turn_target = None
                self.turn_start = None
                self.ref_pos = self.pos      # reset stuck ref after turn completes
                return False
            self.publish(0.0, self._turn_sign * self.p["turn_speed"])
            return True

        def tick(self):
            if self.scan is None:
                return
            p = self.p
            # Gran Prix: publica el estado en vivo (para "Terminal de estado FSM")
            self.pub_state.publish(String(data=(
                f"estado={self.state} pare={self.pare_phase} "
                f"dead_ends={self.dead_ends_visitados} pare_ok={self.pare_respetados}/{self.pare_detectados} "
                f"karpinchus={self.karpinchus_rodeados} ruta_cm={self.total_path_m*100.0:.0f} "
                f"meta={'SI' if self.meta_reached else 'no'}"
            )))
            # Gran Prix: META alcanzada -> detener para siempre y registrar métricas UNA vez.
            if self.meta_reached:
                if not self._metrics_written:
                    self.write_metrics()
                self.state = "META"
                self.publish(0.0, 0.0)
                return
            if self.state == "FAULT":
                self.publish(0.0, 0.0)
                return
            # FABLE loop-completion wired: one full lap done -> STOP for good.
            if self.loop_done:
                if self.state != "DONE":
                    self.get_logger().info(
                        "✅ Vuelta completa — parando. Cajas censadas: %d" % self.boxes_count)
                    self.state = "DONE"
                self.publish(0.0, 0.0)
                return
            # FIX2: stale /scan -> hard stop (never drive on frozen LiDAR data)
            if (self.scan_stamp is not None and
                    (self.now_s() - self.scan_stamp) > p["scan_timeout"]):
                self.get_logger().warn("scan stale -> stop")
                self.publish(0.0, 0.0)
                return

            # Physical safety: this node depends on odom for turns and stuck
            # detection, so do not publish nonzero velocity without fresh odom.
            if p.get("require_odom", True):
                now = self.now_s()
                if not self.have_odom:
                    if not self._warned_no_odom:
                        self.get_logger().warn("no odom yet -> stop")
                        self._warned_no_odom = True
                    self.publish(0.0, 0.0)
                    return
                if (self.odom_stamp is None or
                        (now - self.odom_stamp) > p["odom_timeout"]):
                    if not self._warned_odom_stale:
                        self.get_logger().warn("odom stale -> stop")
                        self._warned_odom_stale = True
                    self.publish(0.0, 0.0)
                    return

            if not self.have_odom and self.state in ("TURN_IN", "TURN_OUT",
                                                     "RECOVER"):
                self.publish(0.0, 0.0)
                return
            s = self.sectors()
            shoulder_left, shoulder_right = self.front_shoulders()
            right_front, right_back, left_front, left_back, rear = self.side_wall_profile()
            side = p["side"]
            follow_wall = s.right if side == "right" else s.left
            dt = 1.0 / p["control_hz"]
            # DEBUG robot9 (Henry): cada ~1s imprime lo que LEE el robot → ver por qué cree "front bloqueado".
            _now = self.now_s()
            if _now - getattr(self, "_dbg_last", 0.0) >= 1.0:
                self._dbg_last = _now
                self.get_logger().info(
                    f"[DBG] front={s.front:.2f} shL={shoulder_left:.2f} shR={shoulder_right:.2f} "
                    f"left={s.left:.2f} right={s.right:.2f} rf/rb={right_front:.2f}/{right_back:.2f} "
                    f"rear={rear:.2f} "
                    f"state={self.state} "
                    f"front_block={p['front_block']:.2f} offset={float(p.get('front_angle_offset', 0.0)):.0f}deg")
                self.report_decision(
                    "TICK", s, (shoulder_left, shoulder_right),
                    (right_front, right_back, left_front, left_back, rear),
                    extra=(f"fb_raw={self.prev_front_blocked} "
                           f"post_left={max(0.0, self.post_veer_until - _now):.2f}"),
                    force=True)

            # finish any active turn first (turns publish linear=0)
            if self.state in ("TURN_IN", "TURN_OUT", "RECOVER"):
                turn_age = ((self.now_s() - self.turn_start)
                            if self.turn_start is not None else 0.0)
                if (self.state in ("TURN_IN", "TURN_OUT")
                        and p.get("corner_align_enabled", True)
                        and turn_age >= p.get("corner_min_turn_t", 0.45)
                        and corner_pose_aligned(s.front, rear, follow_wall, p)):
                    self.turn_target = None
                    self.turn_start = None
                    self.ref_pos = self.pos
                    self.heading_ref = self.yaw
                    self.prev_err = 0.0
                    self.state = "FOLLOW_WALL"
                    self.publish(p["v_min"], 0.0)
                    return
                if self.run_turn():
                    return
                if self.state == "FAULT":
                    self.publish(0.0, 0.0)
                    return
                if self.state == "TURN_OUT":
                    self.publish(p["v_min"], 0.0)   # creep to re-acquire corner
                self.state = "FOLLOW_WALL"
                self.ref_pos = self.pos
                self.heading_ref = self.yaw
                self.prev_err = 0.0   # reset D memory after a turn (no stale-derr spike)
                return

            # recovery reverse in progress (stuck watchdog)
            if self.recovery > 0.0:
                self.recovery -= dt
                self.publish(-p["v_min"], 0.0)
                if self.recovery <= 0.0:
                    self.ref_pos = self.pos
                    self.stuck_time = 0.0
                return

            # HEADING RESTORE (Henry): tras el corte anti-180, volver al rumbo que se tenia
            # ANTES de esquivar y SEGUIR DE FRENTE (no retroceder). Ventana acotada; si el
            # frente se pone muy cerca -> abortar a reversa (anti-choque/anti-loop).
            if self.now_s() < self.restore_until and self.restore_yaw_target is not None:
                if s.front < p["emerg_dist"]:
                    self.restore_until = 0.0
                    self.restore_yaw_target = None
                    self.recovery = p["recovery_t"]
                else:
                    lin, ang = heading_hold_cmd(self.yaw, self.restore_yaw_target, p)
                    self.prev_front_blocked = False
                    self.front_block_time = 0.0
                    self.state = "FOLLOW_WALL"
                    if (abs(ang_diff(self.restore_yaw_target, self.yaw)) < math.radians(8.0)
                            and s.front > p["front_clear"]):
                        self.restore_until = 0.0        # rumbo recuperado + frente libre -> listo
                        self.restore_yaw_target = None
                    self.publish(min(lin, p.get("veer_resume_speed", 0.12)), ang)
                    return

            # --- ancho del pasillo SENSADO en vivo + ¿cabe rotar en sitio? (adaptativo, NEXUS) ---
            # La rotación en sitio necesita pasillo >= diagonal del robot. Si no cabe, NUNCA rotar
            # (eso causaba el turn-timeout/FAULT en tramos angostos) -> retroceder.
            # ancho del pasillo = pared-a-pared. Dos rayos opuestos (±90°) desde el LiDAR YA SUMAN
            # la separación entre paredes -> NO sumar robot_width (bug de NEXUS: el +W inflaba 16cm
            # y anulaba el gate de la diagonal -> el robot rotaba en pasillos <27cm donde no cabe
            # = 'chocó al empezar a girar'). Cazado re-derivando tras el challenge de robot_width.
            ancho_actual = s.left + s.right
            can_rotate = ancho_actual >= self.robot_diag

            # FIX4: emergency stop -> frente MUY cerca. Si cabe rotar, gira; si no, reversa (no FAULT).
            # El emerg MANDA sobre el veer-commit (FABLE): cancela el esquive, no embiste por cumplirlo.
            if s.front < p["emerg_dist"]:
                self.veering = False   # la emergencia pisa el commit del esquive
                self.publish(0.0, 0.0)
                self.prev_front_blocked = True
                self.report_decision(
                    "EMERGENCY_FRONT", s, (shoulder_left, shoulder_right),
                    (right_front, right_back, left_front, left_back, rear),
                    extra=f"front<{p['emerg_dist']:.2f} can_rotate={can_rotate}",
                    force=True)
                # ANTI-CASCADA tambien en EMERGENCIA (ALICE): en espacio angosto el frente
                # se acerca mas -> este path de emergencia dispara los 90 sin el guard normal
                # -> cascadeaba a 180 ("se voltea sobre todo cuando el espacio es menor", Henry).
                # Mismo tope que el TURN_IN normal.
                if can_rotate and self.turn_in_accum_deg + 90.0 > p.get("turn_in_max_accum_deg", 135.0):
                    self.report_decision(
                        "EMERGENCY_CASCADE_BREAK", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra=f"accum={self.turn_in_accum_deg:.0f} -> reverse",
                        force=True)
                    if self.turn_in_start_yaw is not None:
                        self.restore_yaw_target = self.turn_in_start_yaw
                        self.restore_until = self.now_s() + p.get("turn_in_restore_t", 2.5)
                    else:
                        self.recovery = p["recovery_t"]
                    self.turn_in_accum_deg = 0.0
                    self.front_clear_run = 0.0
                elif can_rotate:
                    if self.turn_in_accum_deg == 0.0:
                        self.turn_in_start_yaw = self.yaw
                    self.turn_in_accum_deg += 90.0
                    self.state = "TURN_IN"
                    self.begin_turn(+1.0 if side == "right" else -1.0, 90.0)
                else:
                    self.recovery = p["recovery_t"]   # pasillo < diagonal: no cabe rotar -> retroceder
                return

            # ---------------- Gran Prix: fusión cámara (PARE) ----------------
            # "Zona de atención": la cámara prioriza la búsqueda de PARE cuando el LiDAR
            # indica que se aproxima una intersección (fusión por contexto) -> reduce falsos
            # positivos en tramos rectos largos. Se publica SIEMPRE (barato); pare_detector.py
            # decide si usarla (parámetro use_attention_gate).
            follow_wall_att = s.right if p["side"] == "right" else s.left
            cerca_interseccion = (s.front < p["front_slow"]) or (follow_wall_att >= p["wall_lost"] * 0.85)
            self.pub_cerca.publish(Bool(data=bool(cerca_interseccion)))

            # Regla de arbitraje del reto: "la cámara tiene prioridad para detener (seguridad/
            # regla); el LiDAR tiene prioridad para mover/centrar." PARE detectado -> la FSM
            # fuerza PARAR_PARE aunque el pasillo esté libre. Solo la EMERGENCIA de colisión
            # (arriba) puede pisar esto — un PARE nunca debe hacer que choquemos.
            now_pare = self.now_s()
            if self.pare_phase == "STOPPING":
                self.state = "PARAR_PARE"
                self.publish(0.0, 0.0)
                if now_pare >= self.pare_stop_until:
                    self.pare_phase = "IDLE"
                    self.pare_respetados += 1
                    self.pare_cooldown_until = now_pare + p.get("pare_cooldown_t", 2.5)
                    self.get_logger().info(
                        f"✅ PARE respetado ({self.pare_respetados}/{self.pare_detectados}) — reanudando")
                    if self.pos is not None:
                        self.add_marker("PARE", self.pos, (1.0, 0.0, 0.0))
                return
            if self.pare_flag and now_pare >= self.pare_cooldown_until:
                self.pare_phase = "STOPPING"
                self.state = "PARAR_PARE"
                self.pare_stop_until = now_pare + p.get("pare_wait_t", 3.0)
                self.pare_detectados += 1
                self.get_logger().warn(
                    f"🛑 PARE detectado -> deteniéndose {p.get('pare_wait_t', 3.0):.1f}s "
                    f"(#{self.pare_detectados})")
                self.publish(0.0, 0.0)
                return

            # stuck watchdog: no positional progress while following -> reverse
            if self.have_odom and self.pos is not None:
                if self.ref_pos is None:
                    self.ref_pos = self.pos
                moved = math.hypot(self.pos[0] - self.ref_pos[0],
                                   self.pos[1] - self.ref_pos[1])
                if moved > p["stuck_dpos"]:
                    self.ref_pos = self.pos
                    self.stuck_time = 0.0
                else:
                    self.stuck_time += dt
                    if self.stuck_time > p["stuck_t"]:
                        self.get_logger().warn("stuck -> recovery (reverse)")
                        self.recovery = p["recovery_t"]
                        self.stuck_time = 0.0
                        return

            # front hysteresis: enter blocked at front_block, leave at front_clear.
            # Then require persistence before handing "blocked" to the FSM. On robot 9
            # the front sector can briefly catch lateral wall/corner returns; without this
            # debounce, TURN_IN wins every tick and the robot spins in circles.
            if self.prev_front_blocked:
                raw_fb = s.front < p["front_clear"]
            else:
                raw_fb = s.front <= p["front_block"]
            self.prev_front_blocked = raw_fb
            if raw_fb:
                self.front_block_time += dt
                self.front_clear_run = 0.0
            else:
                self.front_block_time = 0.0
                self.front_clear_run += dt
                # ANTI-CASCADA (ALICE): el acumulador de TURN_IN solo se resetea cuando el
                # robot ESCAPO de verdad = frente despejado SOSTENIDO. En un pocket el frente
                # se abre solo ~2s entre giros; con umbral ~3s no falso-resetea -> el break cae
                # a tiempo (en el 2do giro) y no deja acumular hasta 180.
                if self.front_clear_run >= p.get("turn_in_reset_clear_t", 3.0):
                    self.turn_in_accum_deg = 0.0
            fb = raw_fb and self.front_block_time >= p.get("front_block_persist", 0.0)
            now_for_veer = self.now_s()

            # VEER_RESUME: for free boxes this can hold heading briefly. For
            # Henry's right-wall protrusions (veer_back_enabled=false), do NOT
            # keep heading-hold after the pass; immediately let wall-follow
            # reacquire the right wall while suppressing only re-detection.
            if now_for_veer < self.veer_resume_until:
                target = self.veer_entry_yaw if self.veer_entry_yaw is not None else self.yaw
                lin, ang = heading_hold_cmd(self.yaw, target, p)
                self.deadend_time = 0.0
                self.prev_front_blocked = False
                self.front_block_time = 0.0
                self.state = "FOLLOW_WALL"
                self.publish(min(lin, p.get("veer_resume_speed", 0.12)), ang)
                return

            if now_for_veer < self.veer_grace_until:
                self.deadend_time = 0.0
                self.prev_front_blocked = False
                self.front_block_time = 0.0
                self.state = "FOLLOW_WALL"
                if p.get("veer_back_enabled", False):
                    target = self.veer_entry_yaw if self.veer_entry_yaw is not None else self.yaw
                    lin, ang = heading_hold_cmd(self.yaw, target, p)
                    self.publish(min(lin, p.get("veer_resume_speed", 0.12)), ang)
                    return

            # POST-VEER REACQUIRE: a box attached/protruding from the right wall can
            # make the followed wall look "lost" immediately after the pass. If the
            # normal FSM sees that as a corner it triggers TURN_OUT (90°) and the
            # robot heads back in the opposite direction. For Henry's boxes-loop,
            # after a protrusion bypass we suppress FSM turns briefly and just
            # advance/gently reacquire the right wall.
            if now_for_veer < self.post_veer_until:
                post_moved = (
                    math.hypot(self.pos[0] - self.post_veer_start[0],
                               self.pos[1] - self.post_veer_start[1])
                    if (self.pos is not None and self.post_veer_start is not None)
                    else 0.0
                )
                wall_max = p.get("post_veer_wall_max", 0.42)
                min_dist = p.get("post_veer_reacquire_dist", 0.30)
                wall_reacquired = follow_wall <= wall_max
                if wall_reacquired and post_moved >= min_dist:
                    self.post_veer_until = 0.0
                    self.post_veer_start = None
                    self.report_decision(
                        "POST_VEER_DONE", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra=f"moved={post_moved:.2f} wall={follow_wall:.2f}",
                        force=True)
                else:
                    self.deadend_time = 0.0
                    self.prev_front_blocked = False
                    self.front_block_time = 0.0
                    self.state = "FOLLOW_WALL"
                    self.prev_err = 0.0
                    target = (self.veer_entry_yaw
                              if self.veer_entry_yaw is not None
                              else self.yaw)
                    _, ang = heading_hold_cmd(self.yaw, target, p)
                    if wall_reacquired:
                        # Once the wall is visible, let side geometry pull gently
                        # toward the right wall without allowing a turn state.
                        err = p["wall_target"] - follow_wall
                        sign = 1.0 if side == "right" else -1.0
                        ang += sign * p.get("kp", 0.7) * err
                    ang = clamp(ang,
                                -p.get("post_veer_w_max", 0.10),
                                p.get("post_veer_w_max", 0.10))
                    self.report_decision(
                        "POST_VEER_REACQUIRE", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra=(f"suppress_turnout=1 moved={post_moved:.2f}/{min_dist:.2f} "
                               f"wall={follow_wall:.2f}/{wall_max:.2f} ang={ang:.2f}"),
                        force=False)
                    self.publish(min(p.get("veer_resume_speed", 0.12), p["v_max"]), ang)
                    return

            # --- VEER&RESUME: obstáculo al frente con un lado LIBRE -> esquivar SIN rotar (NEXUS).
            # Paredes y obstáculos son cajas: la evasión sale del CLEARANCE, no de etiquetas. Veer
            # hacia el lado más despejado AVANZANDO -> rodea el obstáculo sin circundarlo ni invertir
            # rumbo (lo que disparaba el falso loop-completion) y funciona en angosto (no rota).
            # VEER-COMMIT: disparar el esquive ANTES (a front_slow, más anticipación) y SOSTENERLO
            # hasta que el cuerpo SALGA del obstáculo (por odom ~1 largo + frente despejado). Sin
            # commit, soltaba al primer respiro tick-a-tick y rozaba/empujaba la caja al pasar.
            if p.get("enable_obstacle_veer", False):
                free = max(s.left, s.right)
                can_veer = free > max(p.get("min_side_clearance", 0.23), p["robot_width"] / 2 + 0.10)
                veer_speed = max(p["v_min"], 0.12)
                looks_box = localized_front_obstacle(s.front, shoulder_left, shoulder_right, p)
                if self.veering:
                    moved = (math.hypot(self.pos[0] - self.veer_start[0], self.pos[1] - self.veer_start[1])
                             if (self.pos is not None and self.veer_start is not None) else 0.0)
                    veer_elapsed = self.now_s() - self.veer_start_time
                    min_dist = max(p.get("veer_min_dist", 0.85), p["robot_length"] + 0.20)
                    min_t = p.get("veer_min_t", 2.3)

                    def finish_veer():
                        self.karpinchus_rodeados += 1
                        if self.pos is not None:
                            self.add_marker("KARPINCHU", self.pos, (0.9, 0.6, 0.1))
                        self.veering = False
                        self.box_stopped = False   # próxima caja vuelve a disparar la parada 3s del guardián (ALICE)
                        self.box_stop_until = 0.0
                        self.veer_phase = "IDLE"
                        now = self.now_s()
                        resume_t = p.get("veer_resume_t", 0.0)
                        grace_t = p.get("veer_grace_t", 0.0)
                        self.veer_resume_until = now + resume_t
                        self.veer_grace_until = now + resume_t + grace_t
                        if not p.get("veer_back_enabled", False):
                            self.post_veer_until = now + p.get("post_veer_reacquire_t", 3.0)
                            self.post_veer_start = self.pos
                        self.box_stopped = False
                        self.box_stop_until = 0.0
                        self.prev_front_blocked = False
                        self.front_block_time = 0.0

                    # TIMEOUT/DIST cap: do not reverse into the box. If the
                    # bypass took too long, recover heading and let the main FSM
                    # decide from fresh sectors.
                    if (moved > max(0.90, 4.0 * p["robot_length"])) or (veer_elapsed > p["veer_timeout"]):
                        finish_veer()
                        return

                    if self.veer_entry_yaw is not None:
                        route_err = abs(ang_diff(self.yaw, self.veer_entry_yaw))
                        if route_err > math.radians(p.get("veer_max_yaw_delta", 25.0)):
                            finish_veer()
                            return

                    if self.veer_phase == "OUT":
                        target = self.veer_out_yaw if self.veer_out_yaw is not None else self.yaw
                        err = ang_diff(target, self.yaw)
                        if abs(err) > math.radians(5.0):
                            self.deadend_time = 0.0
                            max_w = p.get("veer_turn_speed", 0.12)
                            ang = clamp(p.get("veer_out_kp", 1.0) * err, -max_w, max_w)
                            self.publish(p.get("veer_out_speed", p["v_min"]), ang)
                            return
                        self.veer_phase = "PASS"
                        self.veer_phase_time = self.now_s()

                    if self.veer_phase == "PASS":
                        target = self.veer_out_yaw if self.veer_out_yaw is not None else self.yaw
                        lin, ang = heading_hold_cmd(self.yaw, target, p)
                        self.deadend_time = 0.0
                        self.prev_err = 0.0
                        if (
                            moved >= min_dist
                            and veer_elapsed >= min_t
                            and s.front > p.get("obstacle_clear", p["front_slow"] * 1.15)
                        ):
                            if not p.get("veer_back_enabled", False):
                                self.report_decision(
                                    "VEER_PASS_FINISH", s, (shoulder_left, shoulder_right),
                                    (right_front, right_back, left_front, left_back, rear),
                                    extra=f"moved={moved:.2f} elapsed={veer_elapsed:.2f}",
                                    force=True)
                                finish_veer()
                                return
                            self.veer_phase = "BACK"
                            self.veer_phase_time = self.now_s()
                        else:
                            self.publish(p.get("veer_pass_speed", veer_speed), ang)
                            return

                    if self.veer_phase == "BACK":
                        target = self.veer_entry_yaw if self.veer_entry_yaw is not None else self.yaw
                        err = ang_diff(target, self.yaw)
                        heading_ok = abs(err) <= math.radians(p.get("veer_finish_yaw_tol", 25.0))
                        follow_front = right_front if side == "right" else left_front
                        follow_back = right_back if side == "right" else left_back
                        if (p.get("wall_align_enabled", True) and
                                wall_is_parallel(follow_front, follow_back, side, p) and
                                heading_ok):
                            finish_veer()
                            return
                        if (p.get("corner_align_enabled", True) and
                                corner_pose_aligned(s.front, rear, follow_wall, p) and
                                heading_ok):
                            finish_veer()
                            return
                        if abs(err) <= math.radians(6.0):
                            finish_veer()
                            return
                        lin, ang = heading_hold_cmd(self.yaw, target, p)
                        self.deadend_time = 0.0
                        self.prev_err = 0.0
                        self.publish(min(lin, p.get("veer_resume_speed", 0.12)), ang)
                        return
                elif (now_for_veer >= self.veer_grace_until and
                      looks_box and can_veer):
                    # GUARDIÁN (rúbrica RC-4 IMP3): al detectar CAJA adelante, PARAR y ESPERAR
                    # box_stop_wait_t (3s) ANTES de rodear — "el guardián se detiene frente a
                    # cada caja". La parada ocurre a la distancia de disparo del veer (>15cm). ALICE
                    now_g = self.now_s()
                    if not self.box_stopped:
                        if self.box_stop_until == 0.0:
                            self.box_stop_until = now_g + p.get("box_stop_wait_t", 3.0)
                            self.report_decision(
                                "GUARD_STOP_BOX", s, (shoulder_left, shoulder_right),
                                (right_front, right_back, left_front, left_back, rear),
                                extra=f"parar+esperar {p.get('box_stop_wait_t', 3.0):.1f}s frente a caja",
                                force=True)
                        if now_g < self.box_stop_until:
                            self.publish(0.0, 0.0)   # DETENIDO frente a la caja (guardián)
                            return
                        self.box_stopped = True       # 3s cumplidos -> proceder a rodear
                        self.box_stop_until = 0.0
                    self.veering = True
                    self.veer_phase = "OUT"
                    if p.get("veer_force_away_from_wall", True):
                        self.veer_sign = 1.0 if side == "right" else -1.0
                    else:
                        self.veer_sign = 1.0 if s.left >= s.right else -1.0
                    self.veer_start = self.pos
                    self.veer_start_time = self.now_s()
                    self.veer_phase_time = self.veer_start_time
                    self.veer_entry_yaw = self.yaw
                    self.veer_out_yaw = self.yaw + self.veer_sign * math.radians(p.get("veer_out_angle", 4.0))
                    self.deadend_time = 0.0
                    self.publish(
                        p.get("veer_out_speed", p["v_min"]),
                        self.veer_sign * p.get("veer_turn_speed", 0.08))
                    self.report_decision(
                        "VEER_START", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra=(f"sign={self.veer_sign:.0f} out_angle={p.get('veer_out_angle', 4.0):.1f} "
                               f"can_veer={can_veer} looks_box={looks_box}"),
                        force=True)
                    self.prev_err = 0.0
                    return
            else:
                self.veering = False
                self.veer_phase = "IDLE"

            new_state = decide_state(self.state, s, p, front_blocked=fb)
            if should_hold_straight(s, p, front_blocked=fb):
                if self.heading_ref is None:
                    self.heading_ref = self.yaw
                lin, ang = heading_hold_cmd(self.yaw, self.heading_ref, p)
                self.state = "FOLLOW_WALL"
                self.prev_err = 0.0
                self.publish(lin, ang)
                return
            if new_state == "RECOVER":
                if p.get("disable_recover_180", False) or is_loop_boxes_mode(p):
                    self.get_logger().warn(
                        "boxed reading in loop_boxes -> no 180 RECOVER; using short reverse")
                    self.report_decision(
                        "RECOVER_BLOCKED_LOOP_BOXES", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra="disable_recover_180=1 reverse_short=1",
                        force=True)
                    self.deadend_time = 0.0
                    self.recovery = p["recovery_t"]
                    return
                # PERSISTENCIA (FABLE): el dead-end debe SOSTENERSE antes del 180°, así un roce
                # momentáneo con una caja no invierte el rumbo (falso dead-end).
                self.deadend_time += dt
                if self.deadend_time < p["recover_persist"]:
                    self.publish(0.0, 0.0)   # esperar a confirmar el dead-end real
                    return
                self.deadend_time = 0.0
                if can_rotate:
                    self.state = "RECOVER"
                    self.dead_ends_visitados += 1
                    if self.pos is not None:
                        self.add_marker("DEAD_END", self.pos, (0.6, 0.0, 0.6))
                    self.get_logger().info(f"🚧 Callejón sin salida #{self.dead_ends_visitados} -> 180°")
                    self.begin_turn(+1.0, 180.0)
                else:
                    self.recovery = p["recovery_t"]   # dead-end en pasillo angosto -> reversa, no rotar
                return
            self.deadend_time = 0.0   # cualquier otro estado -> el dead-end no persistió
            if new_state == "TURN_IN":
                # ANTI-CASCADA TURN_IN (ALICE): dos TURN_IN de 90 consecutivos SIN
                # ESCAPAR del rincon se suman a 180 y el robot termina mirando atras
                # (bug real cazado en el log de Henry: yaw 6->65->158, phase=IDLE, sin
                # veer). El acumulador se RESETEA por FRENTE-DESPEJADO SOSTENIDO (arriba,
                # en la histeresis) -> solo cuando el robot escapo de verdad, NO cuando
                # apenas avanza hacia el obstaculo (ese avance euclidiano falso-reseteaba
                # y dejaba el break tarde -> ~180; residual cazado por JARVIS y probado
                # con el 2do log). Si ya giro >= cap sin escapar -> reversa corta.
                cap = p.get("turn_in_max_accum_deg", 135.0)
                self.report_decision(
                    "TURN_IN_REQUEST", s, (shoulder_left, shoulder_right),
                    (right_front, right_back, left_front, left_back, rear),
                    extra=f"fb={fb} can_rotate={can_rotate} accum={self.turn_in_accum_deg:.0f}",
                    force=True)
                if self.turn_in_accum_deg + 90.0 > cap:
                    # trinquete detectado: otro TURN_IN pasaria del cap (->180) -> reversa
                    self.report_decision(
                        "TURN_IN_CASCADE_BREAK", s, (shoulder_left, shoulder_right),
                        (right_front, right_back, left_front, left_back, rear),
                        extra=f"accum={self.turn_in_accum_deg:.0f} cap={cap:.0f} -> reverse",
                        force=True)
                    self.turn_in_accum_deg = 0.0
                    self.front_clear_run = 0.0
                    if self.turn_in_start_yaw is not None:
                        self.restore_yaw_target = self.turn_in_start_yaw
                        self.restore_until = self.now_s() + p.get("turn_in_restore_t", 2.5)
                    else:
                        self.recovery = p["recovery_t"]
                    return
                if can_rotate:
                    if self.turn_in_accum_deg == 0.0:
                        self.turn_in_start_yaw = self.yaw
                        if self.pos is not None:
                            self.add_marker("INTERSECCION", self.pos, (0.1, 0.4, 0.9))
                    self.turn_in_accum_deg += 90.0
                    self.state = "TURN_IN"
                    self.begin_turn(+1.0 if side == "right" else -1.0, 90.0)
                else:
                    self.recovery = p["recovery_t"]
                return
            if new_state == "TURN_OUT":
                self.report_decision(
                    "TURN_OUT_REQUEST", s, (shoulder_left, shoulder_right),
                    (right_front, right_back, left_front, left_back, rear),
                    extra=f"follow_wall={follow_wall:.2f} wall_lost={p['wall_lost']:.2f}",
                    force=True)
                if self.pos is not None:
                    self.add_marker("INTERSECCION", self.pos, (0.1, 0.4, 0.9))
                self.state = "TURN_OUT"
                self.begin_turn(-1.0 if side == "right" else +1.0, 90.0)
                return
            self.state = "FOLLOW_WALL"
            # track lateral error across ticks so follow_cmd can apply D-damping
            wall = s.right if side == "right" else s.left
            err = p["wall_target"] - wall
            lin, ang = follow_cmd(s, p, prev_err=getattr(self, "prev_err", 0.0), dt=dt)
            if p.get("wall_align_enabled", True):
                follow_front = right_front if side == "right" else left_front
                follow_back = right_back if side == "right" else left_back
                align_err = wall_parallel_error(follow_front, follow_back, side, p)
                if align_err is not None:
                    corr = clamp(p.get("wall_align_kp", 1.2) * align_err,
                                 -p.get("wall_align_w_max", 0.18),
                                 p.get("wall_align_w_max", 0.18))
                    ang = clamp(ang + corr, -p["w_max"], p["w_max"])
            self.prev_err = err
            if abs(ang) < 1e-3:
                self.heading_ref = self.yaw
            self.publish(lin, ang)

        def stop(self):
            self.publish(0.0, 0.0)


    def main(args=None):
        rclpy.init(args=args)
        node = MazeSolver()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop()
            node.write_metrics()   # Ctrl+C u otro cierre -> registra la corrida igual (aunque incompleta)
            node.destroy_node()
            rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) not available in this environment.")


if __name__ == "__main__":
    main()
