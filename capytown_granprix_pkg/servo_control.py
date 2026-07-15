#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
servo_control.py — Posiciona los 2 servos de la cámara (pan/tilt) al arrancar.
================================================================================
Yahboom ROSMASTER R2 expone su mástil de cámara como dos servos controlados
por std_msgs/Int32 en /servo_s1 (pan, horizontal) y /servo_s2 (tilt, vertical).
Hasta ahora esto se hacía a mano con `ros2 topic pub ... --once`, pero un
--once puede perderse si el publisher se cierra antes de que el driver de
Yahboom (yahboomcar_bringup) alcance a descubrirlo (problema típico de
discovery en ROS2). Este nodo publica varias veces seguidas y recién ahí
se cierra solo, así que es más confiable que el --once manual.

No necesita compilar nada (sin colcon), se corre igual que el resto:

    python3 -m capytown_granprix_pkg.servo_control --ros-args \
        -p pan_angle:=20 -p tilt_angle:=-55

O con los valores por defecto del proyecto (mismos que README/run_granprix.sh):

    python3 -m capytown_granprix_pkg.servo_control
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32
    _HAVE_ROS = True
except Exception:  # pragma: no cover
    _HAVE_ROS = False
    Node = object  # type: ignore


if _HAVE_ROS:
    class ServoControl(Node):
        def __init__(self):
            super().__init__("servo_control")
            self.declare_parameter("pan_angle", 20)     # servo_s1: horizontal (-, izq / +, der)
            self.declare_parameter("tilt_angle", -55)   # servo_s2: vertical (-, abajo / +, arriba)
            self.declare_parameter("pan_topic", "/servo_s1")
            self.declare_parameter("tilt_topic", "/servo_s2")
            self.declare_parameter("repeat_count", 5)     # nº de veces que reenvía el ángulo
            self.declare_parameter("repeat_interval", 0.3)  # s entre reenvíos

            self.pan_angle = int(self.get_parameter("pan_angle").value)
            self.tilt_angle = int(self.get_parameter("tilt_angle").value)
            pan_topic = str(self.get_parameter("pan_topic").value)
            tilt_topic = str(self.get_parameter("tilt_topic").value)
            self.repeat_left = int(self.get_parameter("repeat_count").value)
            interval = float(self.get_parameter("repeat_interval").value)

            self.pub_pan = self.create_publisher(Int32, pan_topic, 10)
            self.pub_tilt = self.create_publisher(Int32, tilt_topic, 10)

            self.get_logger().info(
                f"servo_control: pan={self.pan_angle} ({pan_topic})  "
                f"tilt={self.tilt_angle} ({tilt_topic})  "
                f"x{self.repeat_left} cada {interval:.1f}s")

            self.timer = self.create_timer(interval, self._tick)

        def _tick(self):
            self.pub_pan.publish(Int32(data=self.pan_angle))
            self.pub_tilt.publish(Int32(data=self.tilt_angle))
            self.repeat_left -= 1
            if self.repeat_left <= 0:
                self.get_logger().info("servo_control: posición enviada, listo. Cerrando.")
                self.timer.cancel()
                rclpy.shutdown()

    def main(args=None):
        rclpy.init(args=args)
        node = ServoControl()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
else:
    def main(args=None):
        raise SystemExit("ROS2 (rclpy) no disponible aquí.")


if __name__ == "__main__":
    main()
