#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_map_viewer.py — Visor en vivo OPCIONAL para el CapyTown Gran Prix.
=========================================================================
Panel de depuración/visualización — NO participa en la navegación (no publica
/cmd_vel, no toca la FSM de maze_solver.py). Dos paneles en una sola ventana:

  1. "Marco robot — scan + clasificación": el /scan del LiDAR en el marco del
     robot, clasificado en PARED (azul, reutiliza detect_boxes_in_scan() de
     box_detector.py para separar) vs CAJA/karpinchu (naranja), con las 3
     distancias de sector (frente/izq/der) que usa maze_solver.py para decidir,
     y el estado FSM en vivo (/maze_state).
  2. "Recorrido del robot": la trayectoria acumulada en el marco de /odom_raw
     (world frame), desde INICIO hasta la posición actual.

Es OPCIONAL — se activa con:
    ros2 launch capytown_granprix_pkg granprix.launch.py show_map:=true
o suelto:
    ros2 run capytown_granprix_pkg scan_map_viewer
"""
from __future__ import annotations
import math

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    _HAVE_ROS = True
except Exception:  # pragma: no cover
    _HAVE_ROS = False
    Node = object  # type: ignore

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    _HAVE_MPL = True
except Exception:  # pragma: no cover
    _HAVE_MPL = False

# Reutiliza los helpers PUROS ya probados de los otros nodos — no reinventa
# la clasificación CAJA/PARED ni los sectores de decisión.
from capytown_granprix_pkg.box_detector import detect_boxes_in_scan
from capytown_granprix_pkg.maze_solver import sanitize, sector_robust

BG = "#0b1120"
FG = "white"
COLOR_PARED = "#3fa7ff"
COLOR_CAJA = "#ffa500"


if _HAVE_ROS:
    class ScanMapViewer(Node):
        def __init__(self):
            super().__init__("scan_map_viewer")
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom_raw")
            self.declare_parameter("state_topic", "/maze_state")
            self.declare_parameter("range_min", 0.12)
            self.declare_parameter("range_max", 8.0)
            self.declare_parameter("trail_max_points", 5000)

            scan_t = self.get_parameter("scan_topic").value
            odom_t = self.get_parameter("odom_topic").value
            state_t = self.get_parameter("state_topic").value
            self.range_min = float(self.get_parameter("range_min").value)
            self.range_max = float(self.get_parameter("range_max").value)
            self.trail_max = int(self.get_parameter("trail_max_points").value)

            qos = qos_profile_sensor_data
            self.create_subscription(LaserScan, scan_t, self._on_scan, qos)
            self.create_subscription(Odometry, odom_t, self._on_odom, qos)
            self.create_subscription(String, state_t, self._on_state, 10)

            self.scan = None
            self.state_text = "esperando /maze_state..."
            self.pos = (0.0, 0.0)
            self.trail_x: list[float] = []
            self.trail_y: list[float] = []

            self.get_logger().info(
                f"scan_map_viewer listo: scan={scan_t} odom={odom_t} state={state_t}")

        def _on_scan(self, msg):
            self.scan = msg

        def _on_state(self, msg):
            self.state_text = msg.data

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            self.pos = (p.x, p.y)
            self.trail_x.append(p.x)
            self.trail_y.append(p.y)
            if len(self.trail_x) > self.trail_max:
                self.trail_x = self.trail_x[-self.trail_max:]
                self.trail_y = self.trail_y[-self.trail_max:]

        # ---------------- dibujo ----------------
        def draw(self, ax_scan, ax_path):
            self._draw_scan(ax_scan)
            self._draw_path(ax_path)

        def _draw_scan(self, ax):
            ax.clear()
            ax.set_facecolor(BG)
            ax.set_title("Marco robot — scan + clasificación", color=FG)
            m = self.scan
            if m is None:
                ax.text(0.5, 0.5, "esperando /scan...", color=FG, ha="center",
                         transform=ax.transAxes)
                return
            ranges = list(m.ranges)
            rmin = getattr(m, "range_min", self.range_min) or self.range_min
            rmax = getattr(m, "range_max", self.range_max) or self.range_max

            xs, ys = [], []
            for i, r in enumerate(ranges):
                d = sanitize(r, rmin, rmax)
                if d >= rmax:
                    continue
                a = m.angle_min + i * m.angle_increment
                xs.append(d * math.cos(a))
                ys.append(d * math.sin(a))
            ax.scatter(xs, ys, s=4, c=COLOR_PARED, label="PARED")

            boxes = detect_boxes_in_scan(ranges, m.angle_min, m.angle_increment, rmin, rmax)
            for dist, ang in boxes:
                ax.scatter([dist * math.cos(ang)], [dist * math.sin(ang)],
                           s=60, c=COLOR_CAJA, label="CAJA")

            front = sector_robust(ranges, m.angle_min, m.angle_increment,
                                   -12.0, 12.0, rmin, rmax, drop=1)
            left = sector_robust(ranges, m.angle_min, m.angle_increment,
                                  50.0, 100.0, rmin, rmax, drop=1)
            right = sector_robust(ranges, m.angle_min, m.angle_increment,
                                   -100.0, -50.0, rmin, rmax, drop=1)
            ax.annotate(f"{front*100:.0f}cm", (0, min(front, rmax * 0.9)),
                        color="yellow", ha="center")
            ax.annotate(f"{left*100:.0f}cm", (min(left, rmax * 0.9), 0),
                        color="yellow", va="center", rotation=90)
            ax.annotate(f"{right*100:.0f}cm", (-min(right, rmax * 0.9), 0),
                        color="yellow", va="center", rotation=90)

            ax.plot(0, 0, marker="s", markersize=10, color=FG)
            ax.set_xlim(-1.5, 1.5)
            ax.set_ylim(-1.5, 1.5)
            ax.set_xlabel("x [m]", color=FG)
            ax.set_ylabel("y [m]", color=FG)
            ax.tick_params(colors=FG)
            handles = [plt.Line2D([0], [0], marker="o", color="none",
                                   markerfacecolor=c, label=l)
                       for c, l in ((COLOR_PARED, "PARED"), (COLOR_CAJA, "CAJA"))]
            ax.legend(handles=handles, loc="upper right", fontsize=8, facecolor=BG,
                      labelcolor=FG)
            ax.text(0.02, 0.02, f"FSM: {self.state_text}", color="lime",
                     transform=ax.transAxes, fontsize=7, family="monospace")

        def _draw_path(self, ax):
            ax.clear()
            ax.set_facecolor(BG)
            ax.set_title("Recorrido del robot (marco odom)", color=FG)
            if self.trail_x:
                ax.plot(self.trail_x, self.trail_y, color=COLOR_PARED, linewidth=1.5)
                ax.plot(self.trail_x[0], self.trail_y[0], marker="o", color="lime",
                        markersize=8, label="INICIO")
                ax.plot(self.pos[0], self.pos[1], marker="o", color="red",
                        markersize=8, label="ACTUAL")
                ax.legend(loc="upper right", fontsize=8, facecolor=BG, labelcolor=FG)
            else:
                ax.text(0.5, 0.5, "esperando /odom_raw...", color=FG, ha="center",
                         transform=ax.transAxes)
            ax.set_xlabel("x [m]", color=FG)
            ax.set_ylabel("y [m]", color=FG)
            ax.tick_params(colors=FG)
            ax.set_aspect("equal", adjustable="datalim")


    def main(args=None):
        if not _HAVE_MPL:
            raise SystemExit("matplotlib no disponible — instala matplotlib para usar scan_map_viewer.")
        rclpy.init(args=args)
        node = ScanMapViewer()
        fig, (ax_scan, ax_path) = plt.subplots(1, 2, figsize=(11, 5.5))
        fig.patch.set_facecolor(BG)
        fig.canvas.manager.set_window_title("CapyTown Gran Prix — scan_map_viewer")

        def update(_frame):
            rclpy.spin_once(node, timeout_sec=0.0)
            node.draw(ax_scan, ax_path)
            return []

        ani = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)
        try:
            plt.show()
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
