#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_camera.py — Publica /image_raw con OpenCV directo (cv2.VideoCapture).
==============================================================================
Alternativa a `usb_cam` cuando su binario crashea al convertir YUYV/MJPEG a
RGB (bug de swscale: "terminate called after throwing an instance of 'char*'",
visto en este Raspberry Pi). cv2.VideoCapture decodifica la cámara por su
cuenta (V4L2 + libjpeg), evitando por completo esa ruta de conversión rota.

Publica el mismo tópico que usb_cam (/image_raw, sensor_msgs/Image, bgr8),
así que pare_detector.py y todo lo demás lo consumen exactamente igual.

    ros2 run capytown_granprix_pkg simple_camera
    ros2 run capytown_granprix_pkg simple_camera --ros-args -p image_width:=640 -p image_height:=480
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
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


if _HAVE_ROS:
    class SimpleCamera(Node):
        def __init__(self):
            super().__init__("simple_camera")
            self.declare_parameter("video_device", "/dev/video0")
            self.declare_parameter("image_topic", "/image_raw")
            self.declare_parameter("image_width", 320)
            self.declare_parameter("image_height", 240)
            self.declare_parameter("framerate", 30.0)

            device = str(self.get_parameter("video_device").value)
            topic = str(self.get_parameter("image_topic").value)
            width = int(self.get_parameter("image_width").value)
            height = int(self.get_parameter("image_height").value)
            fps = float(self.get_parameter("framerate").value)

            if not (_HAVE_CV and _HAVE_BRIDGE):
                raise SystemExit("simple_camera necesita opencv-python y cv_bridge.")

            index = int(device.replace("/dev/video", "")) if device.startswith("/dev/video") else device
            self.cap = cv2.VideoCapture(index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self.cap.isOpened():
                raise SystemExit(f"No pude abrir la cámara {device}")

            self.bridge = CvBridge()
            self.pub = self.create_publisher(Image, topic, 10)
            self.create_timer(1.0 / max(fps, 1.0), self._tick)
            self.get_logger().info(
                f"simple_camera listo: {device} {width}x{height}@{fps:.0f}fps -> {topic}")

        def _tick(self):
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn("no pude leer un frame de la cámara")
                return
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.pub.publish(msg)

        def destroy_node(self):
            try:
                self.cap.release()
            except Exception:
                pass
            super().destroy_node()


    def main(args=None):
        rclpy.init(args=args)
        node = SimpleCamera()
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


if __name__ == "__main__":
    main()
