#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
web_dashboard.py — Panel web en vivo (UNA sola pestaña de navegador) para el
CapyTown Gran Prix.
=========================================================================
Sirve por HTTP, en una sola página, TODO lo que antes necesitaba varias
ventanas gráficas: el scan del LiDAR clasificado (PARED/CAJA), el recorrido
del robot, y la cámara con la segmentación de PARE — más un botón de
PAUSA/REANUDAR y un formulario para ajustar en caliente un puñado de
parámetros clave (sin reiniciar nodos).

NO reemplaza scan_map_viewer.py (ventana Tk en el escritorio de RealVNC) — es
una alternativa que no abre NINGUNA ventana gráfica nueva en la sesión VNC;
todo se ve en el navegador (incluso desde tu laptop, conectado a la misma
red del robot).

Activar:
    ros2 launch capytown_granprix_pkg granprix.launch.py show_dashboard:=true
o suelto:
    ros2 run capytown_granprix_pkg web_dashboard

Luego abre en un navegador (en el Pi, o en tu laptop conectada a la misma
red del robot):
    http://<IP-del-robot>:8080/
"""
from __future__ import annotations
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
    _HAVE_ROS = True
except Exception:  # pragma: no cover
    _HAVE_ROS = False
    Node = object  # type: ignore

try:
    import cv2
    import numpy as np
    _HAVE_CV = True
except Exception:  # pragma: no cover
    _HAVE_CV = False

try:
    from cv_bridge import CvBridge
    _HAVE_BRIDGE = True
except Exception:  # pragma: no cover
    _HAVE_BRIDGE = False

try:
    import matplotlib
    matplotlib.use("Agg")  # sin ventana — solo renderiza a buffer, para servirlo por HTTP
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:  # pragma: no cover
    _HAVE_MPL = False

# Reutiliza los mismos helpers puros que scan_map_viewer.py — una sola
# implementación de la clasificación CAJA/PARED y los sectores de decisión.
from capytown_granprix_pkg.box_detector import detect_boxes_in_scan
from capytown_granprix_pkg.maze_solver import sanitize, sector_robust

PORT = 8080
BG = "#0b1120"
FG = "white"
COLOR_PARED = "#3fa7ff"
COLOR_CAJA = "#ffa500"

# Parámetros editables desde el dashboard — set curado a propósito (no
# cualquier parámetro) para no exponer un control que pueda romper la FSM.
EDITABLE_PARAMS = {
    "maze_solver": {
        "v_max": float, "side": str,
    },
    "pare_detector": {
        "use_attention_gate": bool, "min_area": int,
    },
}


def _jpeg_from_bgr(frame, quality=70):
    if not _HAVE_CV or frame is None:
        return None
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


def _fig_to_jpeg(fig, quality=70):
    """Renderiza una Figure de matplotlib (backend Agg) a bytes JPEG, sin
    tocar disco ni abrir ninguna ventana."""
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())
    bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)
    return _jpeg_from_bgr(bgr, quality)


if _HAVE_ROS:
    class WebDashboard(Node):
        def __init__(self):
            super().__init__("web_dashboard")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom_raw")
            self.declare_parameter("state_topic", "/maze_state")
            self.declare_parameter("camera_topic", "/pare/debug_image")
            self.declare_parameter("pause_topic", "/dashboard_pause")
            self.declare_parameter("http_port", PORT)

            scan_t = self.get_parameter("scan_topic").value
            odom_t = self.get_parameter("odom_topic").value
            state_t = self.get_parameter("state_topic").value
            cam_t = self.get_parameter("camera_topic").value
            self.pause_topic = self.get_parameter("pause_topic").value
            self.http_port = int(self.get_parameter("http_port").value)

            qos = qos_profile_sensor_data
            self.create_subscription(LaserScan, scan_t, self._on_scan, qos)
            self.create_subscription(Odometry, odom_t, self._on_odom, qos)
            self.create_subscription(String, state_t, self._on_state, 10)
            self.create_subscription(Image, cam_t, self._on_camera, qos)
            self.pub_pause = self.create_publisher(Bool, self.pause_topic, 10)

            self.bridge = CvBridge() if _HAVE_BRIDGE else None
            self.scan = None
            self.state_text = "esperando /maze_state..."
            self.pos = (0.0, 0.0)
            self.trail_x: list[float] = []
            self.trail_y: list[float] = []
            self.paused = False

            self._lock = threading.Lock()
            self._camera_jpg = None
            self._scan_jpg = None
            self._path_jpg = None

            # figuras reusadas (Agg, sin ventana) — evita recrearlas cada frame
            self._fig_scan, self._ax_scan = plt.subplots(figsize=(5, 5)) if _HAVE_MPL else (None, None)
            self._fig_path, self._ax_path = plt.subplots(figsize=(5, 5)) if _HAVE_MPL else (None, None)
            if _HAVE_MPL:
                for fig in (self._fig_scan, self._fig_path):
                    fig.patch.set_facecolor(BG)

            # pre-crea los clientes de parámetros (una vez, en el hilo principal)
            self._param_clients = {}
            if _HAVE_ROS:
                for node_name in EDITABLE_PARAMS:
                    self._param_clients[node_name] = self.create_client(
                        SetParameters, f"/{node_name}/set_parameters")

            self.create_timer(0.3, self._render_plots)
            self.create_timer(1.0, self._republish_pause)

            self.get_logger().info(
                f"web_dashboard listo: http://0.0.0.0:{self.http_port}/  "
                f"(scan={scan_t} odom={odom_t} state={state_t} camara={cam_t})")

        # ---------------- suscripciones ----------------
        def _on_scan(self, msg):
            self.scan = msg

        def _on_state(self, msg):
            self.state_text = msg.data

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            self.pos = (p.x, p.y)
            self.trail_x.append(p.x)
            self.trail_y.append(p.y)
            if len(self.trail_x) > 5000:
                self.trail_x = self.trail_x[-5000:]
                self.trail_y = self.trail_y[-5000:]

        def _on_camera(self, msg):
            if not (_HAVE_CV and _HAVE_BRIDGE and self.bridge is not None):
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            jpg = _jpeg_from_bgr(frame)
            if jpg is not None:
                with self._lock:
                    self._camera_jpg = jpg

        # ---------------- render periódico (hilo principal de rclpy) ----------------
        def _render_plots(self):
            if not _HAVE_MPL:
                return
            self._draw_scan()
            self._draw_path()

        def _draw_scan(self):
            ax = self._ax_scan
            ax.clear()
            ax.set_facecolor(BG)
            ax.set_title("Marco robot — scan + clasificación", color=FG, fontsize=9)
            m = self.scan
            if m is None:
                ax.text(0.5, 0.5, "esperando /scan...", color=FG, ha="center",
                         transform=ax.transAxes)
            else:
                ranges = list(m.ranges)
                rmin = getattr(m, "range_min", 0.12) or 0.12
                rmax = getattr(m, "range_max", 8.0) or 8.0
                xs, ys = [], []
                for i, r in enumerate(ranges):
                    d = sanitize(r, rmin, rmax)
                    if d >= rmax:
                        continue
                    a = m.angle_min + i * m.angle_increment
                    xs.append(d * math.cos(a))
                    ys.append(d * math.sin(a))
                ax.scatter(xs, ys, s=4, c=COLOR_PARED)
                boxes = detect_boxes_in_scan(ranges, m.angle_min, m.angle_increment, rmin, rmax)
                for dist, ang in boxes:
                    ax.scatter([dist * math.cos(ang)], [dist * math.sin(ang)], s=60, c=COLOR_CAJA)
                front = sector_robust(ranges, m.angle_min, m.angle_increment, -12.0, 12.0, rmin, rmax, drop=1)
                ax.annotate(f"{front*100:.0f}cm", (0, min(front, rmax * 0.9)), color="yellow", ha="center")
                ax.plot(0, 0, marker="s", markersize=9, color=FG)
                ax.set_xlim(-1.5, 1.5)
                ax.set_ylim(-1.5, 1.5)
            ax.tick_params(colors=FG, labelsize=7)
            jpg = _fig_to_jpeg(self._fig_scan)
            if jpg is not None:
                with self._lock:
                    self._scan_jpg = jpg

        def _draw_path(self):
            ax = self._ax_path
            ax.clear()
            ax.set_facecolor(BG)
            ax.set_title("Recorrido del robot (marco odom)", color=FG, fontsize=9)
            if self.trail_x:
                ax.plot(self.trail_x, self.trail_y, color=COLOR_PARED, linewidth=1.5)
                ax.plot(self.trail_x[0], self.trail_y[0], marker="o", color="lime", markersize=7)
                ax.plot(self.pos[0], self.pos[1], marker="o", color="red", markersize=7)
            else:
                ax.text(0.5, 0.5, "esperando /odom_raw...", color=FG, ha="center",
                         transform=ax.transAxes)
            ax.set_aspect("equal", adjustable="datalim")
            ax.tick_params(colors=FG, labelsize=7)
            jpg = _fig_to_jpeg(self._fig_path)
            if jpg is not None:
                with self._lock:
                    self._path_jpg = jpg

        # ---------------- pausa ----------------
        def _republish_pause(self):
            self.pub_pause.publish(Bool(data=self.paused))

        def toggle_pause(self):
            self.paused = not self.paused
            self.pub_pause.publish(Bool(data=self.paused))
            return self.paused

        # ---------------- parámetros ----------------
        def set_param(self, node_name, param_name, raw_value):
            spec = EDITABLE_PARAMS.get(node_name)
            if not spec or param_name not in spec:
                return False, "parámetro no editable desde el dashboard"
            client = self._param_clients.get(node_name)
            if client is None or not client.wait_for_service(timeout_sec=2.0):
                return False, f"servicio /{node_name}/set_parameters no disponible"
            py_type = spec[param_name]
            pv = ParameterValue()
            try:
                if py_type is bool:
                    pv.type = ParameterType.PARAMETER_BOOL
                    pv.bool_value = str(raw_value).strip().lower() in ("1", "true", "yes", "on")
                elif py_type is int:
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(raw_value)
                elif py_type is float:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(raw_value)
                else:
                    pv.type = ParameterType.PARAMETER_STRING
                    pv.string_value = str(raw_value)
            except (TypeError, ValueError):
                return False, f"valor inválido para {param_name} (se espera {py_type.__name__})"

            req = SetParameters.Request()
            req.parameters = [ParameterMsg(name=param_name, value=pv)]
            future = client.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0) < 3.0:
                time.sleep(0.02)
            if not future.done():
                return False, "timeout esperando respuesta del nodo"
            res = future.result()
            if res is None or not res.results:
                return False, "sin resultado"
            r0 = res.results[0]
            return bool(r0.successful), (r0.reason or ("ok" if r0.successful else "rechazado"))

        # ---------------- snapshot para /api/state ----------------
        def get_state(self):
            with self._lock:
                return {
                    "fsm": self.state_text,
                    "paused": self.paused,
                    "pos": {"x": self.pos[0], "y": self.pos[1]},
                }

        def get_jpeg(self, which):
            with self._lock:
                return {"camera": self._camera_jpg, "scan": self._scan_jpg,
                        "path": self._path_jpg}.get(which)


    INDEX_HTML = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<title>CapyTown Gran Prix — Dashboard</title>
<style>
  body { background:#0b1120; color:#e6edf5; font-family: system-ui, sans-serif; margin:0; padding:16px; }
  h1 { font-size:1.2rem; margin:0 0 12px; }
  .grid { display:grid; grid-template-columns: 1fr 1fr; gap:12px; }
  .panel { background:#0f1830; border:1px solid #22314f; border-radius:8px; padding:8px; }
  .panel img { width:100%; display:block; border-radius:4px; background:#000; }
  .wide { grid-column: 1 / -1; }
  #fsm { font-family: monospace; color:#7CFC7C; white-space: pre-wrap; }
  button { background:#2563eb; color:white; border:none; border-radius:6px; padding:8px 16px;
           font-size:0.95rem; cursor:pointer; }
  button.paused { background:#dc2626; }
  form.params { display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-top:8px; }
  select, input { background:#0b1120; color:#e6edf5; border:1px solid #22314f; border-radius:4px;
                  padding:4px 6px; }
  #paramMsg { margin-left:8px; font-size:0.85rem; }
</style></head>
<body>
  <h1>CapyTown Gran Prix — Dashboard en vivo</h1>
  <div class="grid">
    <div class="panel"><img src="/stream/camera" alt="camara"></div>
    <div class="panel"><img src="/stream/scan" alt="scan"></div>
    <div class="panel wide"><img src="/stream/path" alt="recorrido"></div>
    <div class="panel wide">
      <div id="fsm">FSM: —</div>
      <button id="pauseBtn" onclick="togglePause()">Pausar</button>
      <form class="params" onsubmit="return setParam(event)">
        <select id="pNode">
          <option value="maze_solver">maze_solver</option>
          <option value="pare_detector">pare_detector</option>
        </select>
        <select id="pName">
          <option value="v_max">v_max</option>
          <option value="side">side</option>
        </select>
        <input id="pValue" placeholder="valor (ej. 0.12 / right / true)">
        <button type="submit">Aplicar</button>
        <span id="paramMsg"></span>
      </form>
    </div>
  </div>
<script>
const paramsByNode = {
  maze_solver: ["v_max", "side"],
  pare_detector: ["use_attention_gate", "min_area"],
};
const pNode = document.getElementById("pNode");
const pName = document.getElementById("pName");
function refreshNames() {
  pName.innerHTML = "";
  for (const n of paramsByNode[pNode.value]) {
    const o = document.createElement("option"); o.value = n; o.textContent = n; pName.appendChild(o);
  }
}
pNode.addEventListener("change", refreshNames);
refreshNames();

async function togglePause() {
  const r = await fetch("/api/pause", {method: "POST"});
  const j = await r.json();
  const btn = document.getElementById("pauseBtn");
  btn.textContent = j.paused ? "Reanudar" : "Pausar";
  btn.classList.toggle("paused", j.paused);
}
async function setParam(ev) {
  ev.preventDefault();
  const body = {node: pNode.value, name: pName.value, value: document.getElementById("pValue").value};
  const r = await fetch("/api/param", {method: "POST", headers: {"Content-Type": "application/json"},
                                        body: JSON.stringify(body)});
  const j = await r.json();
  document.getElementById("paramMsg").textContent = j.ok ? "✅ " + j.msg : "❌ " + j.msg;
  return false;
}
async function pollState() {
  try {
    const r = await fetch("/api/state");
    const j = await r.json();
    document.getElementById("fsm").textContent =
      "FSM: " + j.fsm + "\\npos=(" + j.pos.x.toFixed(2) + ", " + j.pos.y.toFixed(2) + ")" +
      "  |  " + (j.paused ? "PAUSADO" : "corriendo");
  } catch (e) {}
  setTimeout(pollState, 1000);
}
pollState();
</script>
</body></html>"""


    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # silencia el log de acceso HTTP en la consola del nodo

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self._text(200, INDEX_HTML, "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(self.server.node.get_state())
            elif path in ("/stream/camera", "/stream/scan", "/stream/path"):
                self._mjpeg(path.split("/")[-1])
            else:
                self.send_error(404)

        def do_POST(self):
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b""
            if path == "/api/pause":
                paused = self.server.node.toggle_pause()
                self._json({"paused": paused})
            elif path == "/api/param":
                try:
                    data = json.loads(body or b"{}")
                except json.JSONDecodeError:
                    data = {}
                ok, msg = self.server.node.set_param(
                    data.get("node"), data.get("name"), data.get("value"))
                self._json({"ok": ok, "msg": msg})
            else:
                self.send_error(404)

        def _text(self, code, text, ctype):
            b = text.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def _json(self, obj):
            self._text(200, json.dumps(obj), "application/json")

        def _mjpeg(self, which):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    jpg = self.server.node.get_jpeg(which)
                    if jpg:
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass


    def main(args=None):
        if not (_HAVE_CV and _HAVE_MPL):
            missing = [n for n, ok in (("opencv-python", _HAVE_CV), ("matplotlib", _HAVE_MPL)) if not ok]
            raise SystemExit(f"web_dashboard necesita: {', '.join(missing)}")
        rclpy.init(args=args)
        node = WebDashboard()
        server = ThreadingHTTPServer(("0.0.0.0", node.http_port), _Handler)
        server.node = node
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown()
            node.destroy_node()
            rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) no disponible aquí.")


if __name__ == "__main__":
    main()
