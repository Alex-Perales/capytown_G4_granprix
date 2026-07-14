#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
desktop_dashboard.py — Panel de escritorio (ventana nativa Tkinter, SIN
navegador) para el CapyTown Gran Prix.
==========================================================================
Misma información que web_dashboard.py (cámara, LiDAR clasificado, mapa del
recorrido, botón Pausar/Reanudar y calibración de parámetros en caliente),
pero como una sola ventana de escritorio en el mismo escritorio de RealVNC —
no abre ningún puerto HTTP ni requiere Chromium.

Activar:
    ros2 run capytown_granprix_pkg desktop_dashboard
"""
from __future__ import annotations
import math
import time
import tkinter as tk
from tkinter import ttk

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
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    _HAVE_MPL = True
except Exception:  # pragma: no cover
    _HAVE_MPL = False

from capytown_granprix_pkg.box_detector import detect_boxes_in_scan
from capytown_granprix_pkg.maze_solver import sanitize, sector_robust
from capytown_granprix_pkg.web_dashboard import (
    EDITABLE_PARAMS, COLOR_WALL, MAZE_EXTENT, MAZE_WALLS,
)

BG = "#0b1120"
FG = "white"
PANEL_BG = "#0f1830"
COLOR_PARED = "#3fa7ff"
COLOR_CAJA = "#ffa500"


if _HAVE_ROS:
    class DesktopDashboardNode(Node):
        def __init__(self):
            super().__init__("desktop_dashboard")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom_raw")
            self.declare_parameter("state_topic", "/maze_state")
            self.declare_parameter("camera_topic", "/pare/debug_image")
            self.declare_parameter("pause_topic", "/dashboard_pause")

            scan_t = self.get_parameter("scan_topic").value
            odom_t = self.get_parameter("odom_topic").value
            state_t = self.get_parameter("state_topic").value
            cam_t = self.get_parameter("camera_topic").value
            self.pause_topic = self.get_parameter("pause_topic").value

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
            self.camera_frame = None  # numpy RGB, listo para imshow

            self._param_clients = {}
            for node_name in EDITABLE_PARAMS:
                self._param_clients[node_name] = self.create_client(
                    SetParameters, f"/{node_name}/set_parameters")

            self.create_timer(1.0, self._republish_pause)

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
            self.camera_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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


if _HAVE_MPL:
    class DesktopDashboardApp:
        """Ventana Tk: header (título/estado/pausa/calibración) + 3 columnas.

        No depende de rclpy directamente (solo de un objeto `node` con la
        interfaz de DesktopDashboardNode) para poder probarse localmente,
        sin ROS2, con un nodo de mentira (ver scripts/preview_dashboard.py).
        """

        def __init__(self, node):
            self.node = node
            self.root = tk.Tk()
            self.root.title("CapyTown Gran Prix — Dashboard")
            self.root.configure(bg=BG)
            self.root.geometry("1280x760")

            self._build_header()
            self._build_body()
            self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

        # ---------------- header ----------------
        def _build_header(self):
            header = tk.Frame(self.root, bg=PANEL_BG)
            header.pack(side=tk.TOP, fill=tk.X)

            tk.Label(header, text="CapyTown Gran Prix — Dashboard", bg=PANEL_BG, fg=FG,
                     font=("TkDefaultFont", 12, "bold")).pack(side=tk.LEFT, padx=12, pady=8)

            self.fsm_var = tk.StringVar(value="FSM: —")
            tk.Label(header, textvariable=self.fsm_var, bg=PANEL_BG, fg="#7CFC7C",
                     font=("Courier", 9)).pack(side=tk.LEFT, padx=12)

            self.pause_btn = tk.Button(header, text="Pausar", command=self._toggle_pause,
                                        bg="#2563eb", fg="white", relief=tk.FLAT, padx=12, pady=4)
            self.pause_btn.pack(side=tk.LEFT, padx=12)

            form = tk.Frame(header, bg=PANEL_BG)
            form.pack(side=tk.LEFT, padx=12)

            self.node_var = tk.StringVar(value=next(iter(EDITABLE_PARAMS)))
            node_cb = ttk.Combobox(form, textvariable=self.node_var, state="readonly",
                                    values=list(EDITABLE_PARAMS.keys()), width=13)
            node_cb.grid(row=0, column=0, padx=4)
            node_cb.bind("<<ComboboxSelected>>", self._refresh_param_names)

            self.param_var = tk.StringVar()
            self.param_cb = ttk.Combobox(form, textvariable=self.param_var, state="readonly", width=16)
            self.param_cb.grid(row=0, column=1, padx=4)
            self._refresh_param_names()

            self.value_var = tk.StringVar()
            tk.Entry(form, textvariable=self.value_var, width=14,
                     bg=BG, fg=FG, insertbackground=FG).grid(row=0, column=2, padx=4)

            tk.Button(form, text="Aplicar", command=self._apply_param,
                      bg="#2563eb", fg="white", relief=tk.FLAT, padx=10).grid(row=0, column=3, padx=4)

            self.msg_var = tk.StringVar()
            tk.Label(form, textvariable=self.msg_var, bg=PANEL_BG, fg=FG,
                     font=("TkDefaultFont", 8)).grid(row=0, column=4, padx=8)

        def _refresh_param_names(self, _evt=None):
            names = list(EDITABLE_PARAMS.get(self.node_var.get(), {}).keys())
            self.param_cb["values"] = names
            if names:
                self.param_var.set(names[0])

        def _toggle_pause(self):
            paused = self.node.toggle_pause()
            self.pause_btn.configure(text="Reanudar" if paused else "Pausar",
                                      bg="#dc2626" if paused else "#2563eb")

        def _apply_param(self):
            ok, msg = self.node.set_param(self.node_var.get(), self.param_var.get(), self.value_var.get())
            self.msg_var.set(("OK " if ok else "ERROR ") + msg)

        # ---------------- body: 3 columnas ----------------
        def _build_body(self):
            body = tk.Frame(self.root, bg=BG)
            body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
            for i in range(3):
                body.columnconfigure(i, weight=1)
            body.rowconfigure(0, weight=1)

            self.fig_cam, self.ax_cam, self.canvas_cam = self._make_panel(body, 0, "Cámara")
            self.fig_scan, self.ax_scan, self.canvas_scan = self._make_panel(body, 1, "LiDAR, segmentación")
            self.fig_path, self.ax_path, self.canvas_path = self._make_panel(body, 2, "Mapa del recorrido")

        def _make_panel(self, parent, col, title):
            frame = tk.Frame(parent, bg=PANEL_BG, highlightbackground="#22314f", highlightthickness=1)
            frame.grid(row=0, column=col, sticky="nsew", padx=6, pady=6)
            tk.Label(frame, text=title.upper(), bg=PANEL_BG, fg="#9fb3d9",
                     font=("TkDefaultFont", 9, "bold")).pack(side=tk.TOP, pady=4)
            fig = plt.Figure(figsize=(4, 4))
            fig.patch.set_facecolor(BG)
            ax = fig.add_subplot(111)
            canvas = FigureCanvasTkAgg(fig, master=frame)
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
            return fig, ax, canvas

        # ---------------- loop ----------------
        def tick(self):
            if _HAVE_ROS:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            self._draw_camera()
            self._draw_scan()
            self._draw_path()
            self.fsm_var.set(
                f"FSM: {self.node.state_text}   pos=({self.node.pos[0]:.2f}, {self.node.pos[1]:.2f})"
                f"   {'PAUSADO' if self.node.paused else 'corriendo'}")
            self.root.after(200, self.tick)

        def _draw_camera(self):
            ax = self.ax_cam
            ax.clear()
            ax.set_facecolor(BG)
            ax.axis("off")
            frame = self.node.camera_frame
            if frame is not None:
                ax.imshow(frame)
            else:
                ax.text(0.5, 0.5, "esperando cámara...", color=FG, ha="center", va="center",
                        transform=ax.transAxes)
            self.canvas_cam.draw_idle()

        def _draw_scan(self):
            ax = self.ax_scan
            ax.clear()
            ax.set_facecolor(BG)
            m = self.node.scan
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
            ax.axis("off")
            self.canvas_scan.draw_idle()

        def _draw_path(self):
            ax = self.ax_path
            ax.clear()
            ax.set_facecolor(BG)

            xmin, xmax, ymin, ymax = MAZE_EXTENT
            for gx in (0.6, 1.2, 1.8):
                ax.plot([gx, gx], [ymin, ymax], color="#1e2b45", linewidth=1, linestyle=(0, (2, 3)))
            for gy in (0.6, 1.2, 1.8, 2.4, 3.0):
                ax.plot([xmin, xmax], [gy, gy], color="#1e2b45", linewidth=1, linestyle=(0, (2, 3)))
            for x1, y1, x2, y2 in MAZE_WALLS:
                for lw, a in ((9, 0.12), (6, 0.20), (4, 0.32)):
                    ax.plot([x1, x2], [y1, y2], color=COLOR_WALL, linewidth=lw,
                            alpha=a, solid_capstyle="round")
                ax.plot([x1, x2], [y1, y2], color=COLOR_WALL, linewidth=2.2, solid_capstyle="round")

            if self.node.trail_x:
                ax.plot(self.node.trail_x, self.node.trail_y, color="white", linewidth=1.2, alpha=0.6)
                ax.plot(self.node.trail_x[0], self.node.trail_y[0], marker="o", color="lime", markersize=10)
                ax.plot(self.node.pos[0], self.node.pos[1], marker="o", color="red", markersize=10)
            else:
                ax.plot(0, 0, marker="o", color="lime", markersize=10)
                ax.text(0.5, -0.08, "esperando /odom_raw...", color=FG, ha="center",
                        transform=ax.transAxes, fontsize=8)

            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
            ax.set_aspect("equal", adjustable="box")
            ax.axis("off")
            self.canvas_path.draw_idle()

        def run(self):
            self.tick()
            self.root.mainloop()


if _HAVE_ROS:
    def main(args=None):
        if not _HAVE_MPL:
            raise SystemExit("desktop_dashboard necesita matplotlib.")
        rclpy.init(args=args)
        node = DesktopDashboardNode()
        app = DesktopDashboardApp(node)
        try:
            app.run()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) no disponible aquí.")


if __name__ == "__main__":
    main()
