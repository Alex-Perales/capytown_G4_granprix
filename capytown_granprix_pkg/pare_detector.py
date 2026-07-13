#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pare_detector.py — Detección de la señal PARE (cámara) para el CapyTown Gran Prix.
====================================================================================
Pieza NUEVA (no existía en retos anteriores del equipo — RC-4/box_detector y
maze_navigator eran solo LiDAR). Aporta la SEMÁNTICA que el LiDAR no puede ver:
"¿hay un cartel/cinta roja de PARE en la intersección que tengo enfrente?".

Arquitectura: /image_raw (cámara, usb_cam) -> [pare_detector] -> /pare_detectado
(std_msgs/Bool), consumido por maze_solver.py, que fuerza la parada de ~3s.

Algoritmo (HSV + contornos, clásico y barato — corre bien en un Pi 5):
  1. Convertir BGR -> HSV.
  2. Segmentar ROJO con DOS rangos HSV (el rojo envuelve el 0/179 del canal H
     en OpenCV) + limpieza morfológica (open/close) para quitar ruido.
  3. Buscar contornos y filtrar por ÁREA MÍNIMA, relación de aspecto (~cuadrado/
     octogonal, no una franja delgada), "extent" (área contorno / área caja
     delimitadora) y "solidity" (área contorno / área de su casco convexo) para
     descartar reflejos/franjas rojas y siluetas no convexas (ropa, piel) que
     no son el cartel — un rombo/hexágono real es casi convexo.
  4. DEBOUNCE temporal: exige que la mayoría de los últimos N frames tengan una
     detección válida antes de publicar True (evita falsos positivos de 1 frame
     por un destello/reflejo).
  5. "Zona de atención" (fusión por contexto, pedida en el reto): si
     use_attention_gate=True, solo se evalúa la detección mientras
     /cerca_interseccion (Bool, publicado por maze_solver.py cuando el LiDAR
     indica que se aproxima una intersección) esté en True. Esto reduce
     falsos positivos en tramos rectos largos donde no hay PARE posible.

Publica también /pare/debug_image (sensor_msgs/Image) con la FORMA ROJA real
resaltada (tinte translúcido + contorno, no un rectángulo) y una etiqueta
"PARE" encima — para verlo con `rqt_image_view` (o `ros2 run rqt_image_view
rqt_image_view /pare/debug_image`).
"""
from __future__ import annotations
import collections

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
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


# ── Rangos HSV del rojo (OpenCV: H en [0,179]) — el rojo "envuelve" el 0/179 ──
# Cinta/cartel rojo estilo CapyTown; ajustar en cancha con --ros-args si la luz
# del salón cambia mucho (más saturado/oscuro -> subir/bajar V y S mínimos).
HSV_RED_LO1 = (0, 90, 60)
HSV_RED_HI1 = (10, 255, 255)
HSV_RED_LO2 = (170, 90, 60)
HSV_RED_HI2 = (179, 255, 255)


def _best_valid_contour(contours, min_area=350, min_extent=0.45,
                         aspect_lo=0.55, aspect_hi=1.8, min_solidity=0.75):
    """Filtra contornos de la máscara roja para quedarnos solo con formas
    compactas tipo cartel (rombo/hexágono/octágono): área mínima, relación de
    aspecto ~cuadrada, "extent" (área/caja) y "solidity" (área/casco convexo).
    Un rombo/hexágono es casi convexo (solidity alta); una silueta de ropa/piel
    (mangas, cuello, hombros) tiene entrantes que bajan mucho la solidity —
    esto es lo que evita que una sudadera o la cara cuenten como PARE.
    Devuelve (bbox, area, contour) del mejor candidato, o (None, 0.0, None)."""
    best = None
    best_contour = None
    best_area = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / float(h)
        extent = area / float(w * h)
        if not (aspect_lo <= aspect <= aspect_hi):
            continue
        if extent < min_extent:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or (area / hull_area) < min_solidity:
            continue
        if area > best_area:
            best_area = area
            best = (x, y, w, h)
            best_contour = c
    return best, best_area, best_contour


def detect_red_sign(bgr_image, min_area=350, min_extent=0.45,
                     aspect_lo=0.55, aspect_hi=1.8, min_solidity=0.75):
    """Pure/testable (dado un array numpy BGR). Devuelve (found, bbox, area, mask, contour).
    bbox = (x, y, w, h) del mejor candidato, contour = su contorno real (para resaltar
    la forma exacta en vez de un rectángulo), o (None, None) si no hay ninguno válido."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array(HSV_RED_LO1), np.array(HSV_RED_HI1))
    m2 = cv2.inRange(hsv, np.array(HSV_RED_LO2), np.array(HSV_RED_HI2))
    mask = cv2.bitwise_or(m1, m2)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area, best_contour = _best_valid_contour(
        contours, min_area, min_extent, aspect_lo, aspect_hi, min_solidity)
    return (best is not None), best, best_area, mask, best_contour


def draw_pare_highlight(frame, contour, label="PARE"):
    """Resalta la FORMA ROJA real (no un rectángulo) con un tinte translúcido +
    contorno, y pone `label` en una etiqueta arriba. Devuelve una copia del frame."""
    if contour is None:
        return frame.copy()
    overlay = frame.copy()
    cv2.drawContours(overlay, [contour], -1, (0, 0, 255), thickness=cv2.FILLED)
    out = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)
    cv2.drawContours(out, [contour], -1, (0, 0, 255), 2)

    x, y, _, _ = cv2.boundingRect(contour)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    text_y = max(th + 10, y - 10)
    cv2.rectangle(out, (x, text_y - th - 8), (x + tw + 10, text_y + 4), (0, 0, 255), -1)
    cv2.putText(out, label, (x + 5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return out


if _HAVE_ROS:
    class PareDetector(Node):
        def __init__(self):
            super().__init__("pare_detector")
            if not _HAVE_CV:
                self.get_logger().error(
                    "opencv-python no disponible -> pare_detector no puede procesar imágenes.")
            if not _HAVE_BRIDGE:
                self.get_logger().error(
                    "cv_bridge no disponible -> pare_detector no puede convertir sensor_msgs/Image.")

            self.declare_parameter("image_topic", "/image_raw")
            self.declare_parameter("pare_topic", "/pare_detectado")
            self.declare_parameter("debug_topic", "/pare/debug_image")
            self.declare_parameter("cerca_interseccion_topic", "/cerca_interseccion")
            self.declare_parameter("use_attention_gate", True)
            self.declare_parameter("min_area", 350)
            self.declare_parameter("min_extent", 0.45)
            self.declare_parameter("aspect_lo", 0.55)
            self.declare_parameter("aspect_hi", 1.8)
            self.declare_parameter("debounce_frames", 5)
            self.declare_parameter("debounce_ratio", 0.6)   # >=60% de los últimos N frames

            self.image_topic = self.get_parameter("image_topic").value
            self.pare_topic = self.get_parameter("pare_topic").value
            self.debug_topic = self.get_parameter("debug_topic").value
            self.cerca_topic = self.get_parameter("cerca_interseccion_topic").value
            self.use_attention_gate = bool(self.get_parameter("use_attention_gate").value)
            self.min_area = int(self.get_parameter("min_area").value)
            self.min_extent = float(self.get_parameter("min_extent").value)
            self.aspect_lo = float(self.get_parameter("aspect_lo").value)
            self.aspect_hi = float(self.get_parameter("aspect_hi").value)
            n = int(self.get_parameter("debounce_frames").value)
            self.debounce_ratio = float(self.get_parameter("debounce_ratio").value)
            self._recent = collections.deque(maxlen=max(1, n))

            self.cerca_interseccion = not self.use_attention_gate  # si no hay gate, siempre "atento"
            self.bridge = CvBridge() if _HAVE_BRIDGE else None

            self.sub_img = self.create_subscription(
                Image, self.image_topic, self.on_image, qos_profile_sensor_data)
            self.sub_cerca = self.create_subscription(
                Bool, self.cerca_topic, self.on_cerca, 10)
            self.pub_pare = self.create_publisher(Bool, self.pare_topic, 10)
            self.pub_debug = self.create_publisher(Image, self.debug_topic, 10)

            self.get_logger().info(
                f"pare_detector listo: image={self.image_topic} -> pare={self.pare_topic} "
                f"debug={self.debug_topic} attention_gate={self.use_attention_gate} "
                f"(cerca={self.cerca_topic})")

        def on_cerca(self, msg):
            try:
                self.cerca_interseccion = bool(msg.data)
            except Exception:
                pass

        def on_image(self, msg):
            if not (_HAVE_CV and _HAVE_BRIDGE and self.bridge is not None):
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warn(f"no pude convertir la imagen: {exc}")
                return

            # Zona de atención: si el gate está activo y NO estamos cerca de una
            # intersección, no evaluamos (evita falsos positivos en tramo recto).
            evaluate = self.cerca_interseccion or not self.use_attention_gate
            found, _, area, _, contour = (False, None, 0.0, None, None)
            if evaluate:
                found, _, area, _, contour = detect_red_sign(
                    frame, self.min_area, self.min_extent, self.aspect_lo, self.aspect_hi)

            self._recent.append(1 if found else 0)
            ratio = sum(self._recent) / float(len(self._recent))
            pare_now = ratio >= self.debounce_ratio and len(self._recent) >= 1

            self.pub_pare.publish(Bool(data=bool(pare_now)))
            # header propagado (mismo timestamp/frame_id de la imagen de entrada) —
            # igual que lane_detector.py (out.header = header_msg.header), para que
            # RViz/rqt puedan sincronizar el debug_image con la imagen cruda.
            self._publish_debug(frame, contour, area, pare_now, evaluate, msg.header)

        def _publish_debug(self, frame, contour, area, pare_now, evaluate, header=None):
            try:
                if contour is not None:
                    label = "PARE" if pare_now else f"PARE? area={area:.0f}"
                    dbg = draw_pare_highlight(frame, contour, label)
                else:
                    dbg = frame.copy()
                    label = "buscando (zona de atencion)" if evaluate else "sin atencion (lejos de interseccion)"
                    cv2.putText(dbg, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (255, 255, 255), 2)
                out = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                if header is not None:
                    out.header = header
                self.pub_debug.publish(out)
            except Exception as exc:
                self.get_logger().warn(f"no pude publicar debug_image: {exc}")


    def main(args=None):
        rclpy.init(args=args)
        node = PareDetector()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) no disponible aquí.")


# ============================================================================
# CALIBRADOR HSV INTERACTIVO — mismo flujo que lane_node.py del equipo
# (python3 lane_node.py --calibrar --source 0): sliders en vivo, "s" guarda
# YAML, "q" sale. Útil para ajustar HSV_RED_LO/HI a la luz real del salón
# sin tener que levantar ROS2.
#   python3 pare_detector.py --calibrar --source 0
# ============================================================================
def _nothing(_):
    pass


CALIB_WINDOW = "Calibrador PARE (banda 1 + banda 2)"

_B1_NAMES = ("B1 H min", "B1 H max", "B1 S min", "B1 S max", "B1 V min", "B1 V max")
_B2_NAMES = ("B2 H min", "B2 H max", "B2 S min", "B2 S max", "B2 V min", "B2 V max")


def _make_trackbars(seed1_lo, seed1_hi, seed2_lo, seed2_hi):
    """Una sola ventana con los 12 sliders (banda 1 y banda 2 juntos), para no
    tener que andar moviendo dos ventanas separadas."""
    cv2.namedWindow(CALIB_WINDOW, cv2.WINDOW_NORMAL)
    vals1 = (seed1_lo[0], seed1_hi[0], seed1_lo[1], seed1_hi[1], seed1_lo[2], seed1_hi[2])
    vals2 = (seed2_lo[0], seed2_hi[0], seed2_lo[1], seed2_hi[1], seed2_lo[2], seed2_hi[2])
    maxs = (179, 179, 255, 255, 255, 255)
    for n, v, mx in zip(_B1_NAMES, vals1, maxs):
        cv2.createTrackbar(n, CALIB_WINDOW, v, mx, _nothing)
    for n, v, mx in zip(_B2_NAMES, vals2, maxs):
        cv2.createTrackbar(n, CALIB_WINDOW, v, mx, _nothing)


def _read_trackbars():
    red1 = tuple(cv2.getTrackbarPos(k, CALIB_WINDOW) for k in _B1_NAMES)
    red2 = tuple(cv2.getTrackbarPos(k, CALIB_WINDOW) for k in _B2_NAMES)
    return red1, red2


def _yaml_block(red1, red2, min_area, min_extent, aspect_lo, aspect_hi):
    return f"""pare_detector:
  ros__parameters:
    # rojo1 (banda baja, H cerca de 0) y rojo2 (banda alta, H cerca de 179)
    hsv_red1_h_min: {red1[0]}
    hsv_red1_h_max: {red1[1]}
    hsv_red1_s_min: {red1[2]}
    hsv_red1_s_max: {red1[3]}
    hsv_red1_v_min: {red1[4]}
    hsv_red1_v_max: {red1[5]}
    hsv_red2_h_min: {red2[0]}
    hsv_red2_h_max: {red2[1]}
    hsv_red2_s_min: {red2[2]}
    hsv_red2_s_max: {red2[3]}
    hsv_red2_v_min: {red2[4]}
    hsv_red2_v_max: {red2[5]}
    min_area: {min_area}
    min_extent: {min_extent}
    aspect_lo: {aspect_lo}
    aspect_hi: {aspect_hi}
"""


def run_calibrator(source):
    if not _HAVE_CV:
        raise SystemExit("opencv-python no disponible — instala opencv-python para calibrar.")
    cap = None
    static_img = None
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        img = cv2.imread(source)
        if img is not None:
            static_img = img
        else:
            cap = cv2.VideoCapture(source)
    if static_img is None and (cap is None or not cap.isOpened()):
        raise SystemExit(f"No se pudo abrir la fuente: {source}")

    # Valores iniciales de los sliders — calibrados por el equipo en cancha real.
    _make_trackbars(seed1_lo=(37, 90, 94), seed1_hi=(34, 118, 255),
                     seed2_lo=(66, 90, 60), seed2_hi=(179, 255, 255))

    last_print = 0.0
    red1 = red2 = None
    print("\n[CALIBRADOR PARE] Ajusta los sliders hasta que solo quede el cartel/cinta "
          "roja en las mascaras. 's' guarda YAML, 'q' sale.\n")

    import time as _time
    while True:
        frame = static_img.copy() if static_img is not None else None
        if frame is None:
            ok, frame = cap.read()
            if not ok:
                break
        frame = cv2.resize(frame, (640, 480))

        red1, red2 = _read_trackbars()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array((red1[0], red1[2], red1[4])),
                         np.array((red1[1], red1[3], red1[5])))
        m2 = cv2.inRange(hsv, np.array((red2[0], red2[2], red2[4])),
                         np.array((red2[1], red2[3], red2[5])))
        mask = cv2.bitwise_or(m1, m2)
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, _, best_contour = _best_valid_contour(contours)

        view = draw_pare_highlight(frame, best_contour, "PARE")
        cv2.imshow("Camara (candidato en rojo)", view)
        cv2.imshow("Mascara rojo combinada", mask)

        if _time.time() - last_print > 1.0:
            print("--- Valores actuales ---")
            print(_yaml_block(red1, red2, 350, 0.45, 0.55, 1.8))
            last_print = _time.time()

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            with open("hsv_pare_calibrado.yaml", "w") as f:
                f.write(_yaml_block(red1, red2, 350, 0.45, 0.55, 1.8))
            print("[CALIBRADOR PARE] Guardado en hsv_pare_calibrado.yaml")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    if red1 and red2:
        print("\n=== YAML FINAL (copiar los rangos a HSV_RED_LO1/HI1/LO2/HI2 en pare_detector.py) ===")
        print(_yaml_block(red1, red2, 350, 0.45, 0.55, 1.8))


if __name__ == "__main__":
    import argparse as _argparse
    _parser = _argparse.ArgumentParser(description="CapyTown Gran Prix — pare_detector")
    _parser.add_argument("--calibrar", action="store_true",
                         help="Modo calibracion HSV interactivo (sin ROS2)")
    _parser.add_argument("--source", default="0",
                         help="Indice de camara (0,1,...) o ruta a video/imagen")
    _parsed, _ = _parser.parse_known_args()
    if _parsed.calibrar:
        run_calibrator(_parsed.source)
    else:
        main()
