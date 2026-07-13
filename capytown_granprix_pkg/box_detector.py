#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
box_detector.py — Censo de karpinchus (LiDAR) para el CapyTown Gran Prix.
===========================================================================
REUTILIZADO tal cual del reto RC-4 "El Censo y el Guardián de las Cajas" (JARVIS) —
el Gran Prix pide explícitamente reutilizar este nodo (TPACK: "Reutilización de los
nodos del RC: box_detector") para la VARIANTE OPCIONAL "karpinchus": 1-2 cajas
(karpinchus dormidos) puestas en los pasillos del laberinto, que el robot debe
detectar, detener frente a ellas y rodear sin perder el rumbo ni saltarse un PARE
(maze_solver.py hace la detención/rodeo; este nodo hace el CENSO — cuenta y ubica
las cajas vistas). No corre por defecto si `enable_karpinchus:=false` en el launch.

Arquitectura: LiDAR→/scan→[box_detector]→/cajas_avistadas→[maze_solver.py].
Detecta las CAJAS (20x20cm, geometría heredada del reto RC-4) y las CENSA sin
duplicar, usando /odom para deduplicar por posición en el mundo.

Enchufa al paquete SIN tocar maze_solver.py (nodo separado, arbitrado por /cajas_avistadas).

Algoritmo (solo /scan + /odom, numpy, apto edge):
  1. Segmenta el /scan en tramos continuos (saltos de rango = bordes).
  2. Una CAJA = un tramo cuya ANCHURA física ≈ 0.20m (no una pared larga) y que SALE de la línea
     de pared (protuberancia discreta). Se calcula la anchura = distancia * (Δángulo del tramo).
  3. Proyecta el centro de la caja a coordenadas del MUNDO (con la pose /odom) y deduplica:
     una caja nueva solo cuenta si está a > BOX_MERGE_DIST de las ya vistas.
  4. Publica /cajas_avistadas (std_msgs/Int32 = total) — el FSM lo consume.
"""
from __future__ import annotations
import math
import os
import csv

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int32
    from geometry_msgs.msg import PoseArray, Pose   # IMP2: publica POSICIONES del censo
    _HAVE_ROS = True
except Exception:
    _HAVE_ROS = False

# ── Parámetros (sintonizables) ──
BOX_SIZE = 0.20          # m, lado de la caja
BOX_TOL = 0.08           # m, tolerancia de anchura (caja entre 0.12 y 0.28 m)
RANGE_JUMP = 0.12        # m, salto de rango que separa segmentos (borde)
BOX_MERGE_DIST = 0.30    # m, dos detecciones más cerca que esto = la MISMA caja
MIN_PROTRUSION = 0.05    # m, la caja debe sobresalir al menos esto respecto a sus vecinos


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def detect_boxes_in_scan(ranges, angle_min, angle_inc, range_min, range_max):
    """Pure/testable. Devuelve lista de (dist, ang) de cajas candidatas en el frame del LiDAR."""
    import numpy as np
    r = np.asarray(ranges, dtype=float)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    boxes = []
    n = len(r)
    i = 0
    while i < n:
        if not valid[i]:
            i += 1; continue
        # crece un segmento continuo (sin saltos de rango)
        j = i
        while j + 1 < n and valid[j + 1] and abs(r[j + 1] - r[j]) < RANGE_JUMP:
            j += 1
        seg = r[i:j + 1]
        if len(seg) >= 2:
            mid = (i + j) // 2
            dist = float(np.median(seg))
            width = dist * (len(seg) * angle_inc)              # anchura física del tramo
            # vecinos a los lados del segmento (fondo de pared)
            left = r[i - 1] if i - 1 >= 0 and valid[i - 1] else np.inf
            right = r[j + 1] if j + 1 < n and valid[j + 1] else np.inf
            backdrop = min(left, right)
            protrudes = (backdrop - dist) > MIN_PROTRUSION if math.isfinite(backdrop) else True
            is_box = (abs(width - BOX_SIZE) <= BOX_TOL) and protrudes
            if is_box:
                ang = angle_min + mid * angle_inc
                boxes.append((dist, ang))
        i = j + 1
    return boxes


def census_metrics(detected, ground_truth, match_dist=0.30):
    """Pure/testable. Métricas de la rúbrica (IMP2/DEF3) del censo.
    detected: [(x,y), ...] cajas censadas (mundo).  ground_truth: [(x,y), ...] cajas reales (cinta).
    Matching greedy 1-a-1 por cercanía (≤ match_dist = 30cm de la rúbrica).
    Devuelve dict: VP (matcheadas), FP (detectadas sin GT), FN (GT no detectadas),
    tasa_deteccion = VP/(VP+FN), error_pos_prom (m) de los VP.
    """
    gt = list(ground_truth)
    used = [False] * len(gt)
    vp, errs = 0, []
    for (dx, dy) in detected:
        best, best_d = -1, match_dist + 1e-9
        for k, (gx, gy) in enumerate(gt):
            if used[k]:
                continue
            d = math.hypot(dx - gx, dy - gy)
            if d < best_d:
                best, best_d = k, d
        if best >= 0:
            used[best] = True
            vp += 1
            errs.append(best_d)
    fp = len(detected) - vp
    fn = len(gt) - vp
    rate = vp / (vp + fn) if (vp + fn) > 0 else 0.0
    err = (sum(errs) / len(errs)) if errs else 0.0
    return {"VP": vp, "FP": fp, "FN": fn, "tasa_deteccion": rate, "error_pos_prom": err}


if _HAVE_ROS:
    class BoxDetector(Node):
        def __init__(self):
            super().__init__("box_detector")
            self.scan = None
            self.pose = (0.0, 0.0, 0.0)
            self.seen_world = []          # cajas únicas censadas (x,y mundo)
            # Topics parametrizables: en este robot la odom es /odom_raw (no /odom). Override:
            #   ros2 run capytown_maze_pkg box_detector --ros-args -p odom_topic:=/odom_raw
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("odom_topic", "/odom")
            scan_t = self.get_parameter("scan_topic").value
            odom_t = self.get_parameter("odom_topic").value
            qos = qos_profile_sensor_data
            self.create_subscription(LaserScan, scan_t, self._cb_scan, qos)
            self.create_subscription(Odometry, odom_t, self._cb_odom, qos)
            self.get_logger().info(f"box_detector: scan={scan_t} odom={odom_t}")
            self.pub = self.create_publisher(Int32, "/cajas_avistadas", 10)          # conteo (FSM lo consume)
            self.pub_pos = self.create_publisher(PoseArray, "/cajas_avistadas_pos", 10)  # IMP2: POSICIONES
            # --- métricas de la rúbrica (IMP2/IMP4/DEF3) ---
            #   ground_truth = lista PLANA [x1,y1, x2,y2, ...] de las cajas reales (medidas con cinta, en marco odom)
            #   run_id = número de corrida (1-10) ; metrics_csv = archivo acumulado
            self.declare_parameter("ground_truth", [])          # p.ej. -p ground_truth:="[1.0,0.5, 2.0,-0.5]"
            self.declare_parameter("run_id", 0)
            self.declare_parameter("metrics_csv", "/tmp/metricas_lidar.csv")
            self.declare_parameter("detections_csv", "/tmp/capytown_box_detections.csv")
            gt_flat = list(self.get_parameter("ground_truth").value or [])
            self.ground_truth = [(gt_flat[i], gt_flat[i + 1]) for i in range(0, len(gt_flat) - 1, 2)]
            self.run_id = int(self.get_parameter("run_id").value)
            self.metrics_csv = str(self.get_parameter("metrics_csv").value)
            self.detections_csv = str(self.get_parameter("detections_csv").value)
            self._ensure_csv(self.detections_csv, [
                "time", "run_id", "box_id", "x", "y", "dist_lidar", "angle_rad",
            ])
            self.create_timer(0.2, self._tick)   # 5 Hz
            self.get_logger().info(
                f"box_detector listo — censando cajas en /scan. GT={len(self.ground_truth)} cajas, "
                f"run={self.run_id}, csv={self.metrics_csv}, detecciones={self.detections_csv}")

        def _cb_scan(self, m): self.scan = m
        def _cb_odom(self, m):
            p = m.pose.pose
            self.pose = (p.position.x, p.position.y,
                         yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

        def _tick(self):
            if self.scan is None: return
            s = self.scan
            cand = detect_boxes_in_scan(list(s.ranges), s.angle_min, s.angle_increment,
                                        s.range_min, s.range_max)
            x0, y0, yaw = self.pose
            for dist, ang in cand:
                wx = x0 + dist * math.cos(yaw + ang)
                wy = y0 + dist * math.sin(yaw + ang)
                if all(math.hypot(wx - sx, wy - sy) > BOX_MERGE_DIST for sx, sy in self.seen_world):
                    self.seen_world.append((wx, wy))
                    box_id = len(self.seen_world)
                    self.get_logger().info(f"📦 Caja #{box_id} censada en ({wx:.2f},{wy:.2f})")
                    self._append_detection(box_id, wx, wy, dist, ang)
            self.pub.publish(Int32(data=len(self.seen_world)))
            # IMP2: publica las POSICIONES censadas (marco odom) como PoseArray
            pa = PoseArray()
            pa.header.frame_id = "odom"
            pa.header.stamp = self.get_clock().now().to_msg()
            for (sx, sy) in self.seen_world:
                pose = Pose()
                pose.position.x, pose.position.y = float(sx), float(sy)
                pose.orientation.w = 1.0
                pa.poses.append(pose)
            self.pub_pos.publish(pa)

        def _ensure_csv(self, path, header):
            if not path:
                return
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            if not os.path.exists(path) or os.path.getsize(path) == 0:
                with open(path, "w", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(header)

        def _append_detection(self, box_id, x, y, dist, ang):
            if not self.detections_csv:
                return
            self._ensure_csv(self.detections_csv, [
                "time", "run_id", "box_id", "x", "y", "dist_lidar", "angle_rad",
            ])
            with open(self.detections_csv, "a", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow([
                    f"{self.get_clock().now().nanoseconds / 1e9:.3f}",
                    self.run_id,
                    box_id,
                    f"{x:.3f}",
                    f"{y:.3f}",
                    f"{dist:.3f}",
                    f"{ang:.4f}",
                ])

        def write_metrics(self):
            """Al terminar la corrida: compara el censo vs ground_truth y agrega una fila a metricas_lidar.csv (IMP2/DEF3)."""
            header = ["run_id", "cajas_reales", "cajas_censadas", "VP", "FP", "FN",
                      "tasa_deteccion", "error_pos_prom_m", "posiciones_censadas", "observaciones"]
            m = census_metrics(self.seen_world, self.ground_truth)
            try:
                os.makedirs(os.path.dirname(self.metrics_csv) or ".", exist_ok=True)
                new = not os.path.exists(self.metrics_csv) or os.path.getsize(self.metrics_csv) == 0
                with open(self.metrics_csv, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    if new:
                        w.writerow(header)
                    if self.ground_truth:
                        obs = "auto_con_ground_truth"
                        cajas_reales = len(self.ground_truth)
                        vp, fp, fn = m["VP"], m["FP"], m["FN"]
                        tasa = f"{m['tasa_deteccion']:.3f}"
                        err = f"{m['error_pos_prom']:.3f}"
                    else:
                        obs = "sin_ground_truth; completar VP/FP/FN/error con cinta o video"
                        cajas_reales = ""
                        vp = fp = fn = tasa = err = ""
                    w.writerow([self.run_id, cajas_reales, len(self.seen_world), vp, fp, fn,
                                tasa, err,
                                ";".join(f"({x:.2f},{y:.2f})" for x, y in self.seen_world),
                                obs])
                self.get_logger().info(
                    f"📊 Métricas corrida {self.run_id}: cajas={len(self.seen_world)} → {self.metrics_csv}")
            except Exception as exc:
                self.get_logger().error(f"No pude escribir {self.metrics_csv}: {exc}")

    def main(args=None):
        rclpy.init(args=args)
        node = BoxDetector()
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        finally:
            node.write_metrics()   # al cerrar la corrida (Ctrl+C) → escribe VP/FP/FN a metricas_lidar.csv
            node.destroy_node(); rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) no disponible aquí.")


if __name__ == "__main__":
    main()
