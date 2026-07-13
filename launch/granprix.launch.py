#!/usr/bin/env python3
"""Lanza TODO el software del CapyTown Gran Prix con un solo comando:

    ros2 launch capytown_granprix_pkg granprix.launch.py

Arranca:
  * maze_solver      — LiDAR wall-following + fusión PARE + métricas + RViz markers
  * pare_detector    — cámara -> HSV rojo -> /pare_detectado
  * box_detector     — censo de karpinchus (opcional; no estorba si no hay cajas)
  * scan_map_viewer  — visor OPCIONAL (apagado por defecto): ventana Tk en el
                        escritorio de RealVNC con el scan clasificado PARED/CAJA
                        + el recorrido. No participa en la navegación.
  * web_dashboard    — panel OPCIONAL (apagado por defecto) por navegador, en
                        UNA sola pestaña: LiDAR clasificado + recorrido + cámara
                        con segmentación de PARE, botón de Pausa y ajuste de
                        parámetros en caliente. No abre ninguna ventana gráfica
                        en la sesión VNC — se ve en http://<IP-del-robot>:8080/.

Los parámetros finos viven en config/granprix_params.yaml (ronda, meta_x/y,
pare_reales, long_optima_cm, etc.) — editar ese archivo entre corridas en vez
de este launch file.

Argumentos de línea de comandos más comunes (sobre-escriben el YAML):
    ros2 launch capytown_granprix_pkg granprix.launch.py ronda:=2 meta_enabled:=true
    ros2 launch capytown_granprix_pkg granprix.launch.py side:=left
    ros2 launch capytown_granprix_pkg granprix.launch.py enable_karpinchus:=false
    ros2 launch capytown_granprix_pkg granprix.launch.py show_map:=true
    ros2 launch capytown_granprix_pkg granprix.launch.py show_dashboard:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("capytown_granprix_pkg")
    params_yaml = os.path.join(pkg, "config", "granprix_params.yaml")

    ronda = DeclareLaunchArgument("ronda", default_value="1")
    run_id = DeclareLaunchArgument("run_id", default_value="1")
    side = DeclareLaunchArgument("side", default_value="right")
    meta_enabled = DeclareLaunchArgument("meta_enabled", default_value="false")
    meta_x = DeclareLaunchArgument("meta_x", default_value="3.0")
    meta_y = DeclareLaunchArgument("meta_y", default_value="2.0")
    enable_karpinchus = DeclareLaunchArgument("enable_karpinchus", default_value="true")
    show_map = DeclareLaunchArgument("show_map", default_value="false")
    show_dashboard = DeclareLaunchArgument("show_dashboard", default_value="false")

    maze_solver_node = Node(
        package="capytown_granprix_pkg",
        executable="maze_solver",
        name="maze_solver",
        output="screen",
        parameters=[
            params_yaml,
            {
                "ronda": ParameterValue(LaunchConfiguration("ronda"), value_type=int),
                "run_id": ParameterValue(LaunchConfiguration("run_id"), value_type=int),
                "side": LaunchConfiguration("side"),
                "meta_enabled": ParameterValue(LaunchConfiguration("meta_enabled"), value_type=bool),
                "meta_x": ParameterValue(LaunchConfiguration("meta_x"), value_type=float),
                "meta_y": ParameterValue(LaunchConfiguration("meta_y"), value_type=float),
                "enable_obstacle_veer": ParameterValue(
                    LaunchConfiguration("enable_karpinchus"), value_type=bool),
            },
        ],
    )

    pare_detector_node = Node(
        package="capytown_granprix_pkg",
        executable="pare_detector",
        name="pare_detector",
        output="screen",
        parameters=[params_yaml],
    )

    box_detector_node = Node(
        package="capytown_granprix_pkg",
        executable="box_detector",
        name="box_detector",
        output="screen",
        parameters=[params_yaml],
        condition=IfCondition(LaunchConfiguration("enable_karpinchus")),
    )

    scan_map_viewer_node = Node(
        package="capytown_granprix_pkg",
        executable="scan_map_viewer",
        name="scan_map_viewer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_map")),
    )

    web_dashboard_node = Node(
        package="capytown_granprix_pkg",
        executable="web_dashboard",
        name="web_dashboard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_dashboard")),
    )

    return LaunchDescription([
        ronda, run_id, side, meta_enabled, meta_x, meta_y, enable_karpinchus,
        show_map, show_dashboard,
        maze_solver_node, pare_detector_node, box_detector_node,
        scan_map_viewer_node, web_dashboard_node,
    ])
