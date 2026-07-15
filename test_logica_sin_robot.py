#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_logica_sin_robot.py - Verifica la LOGICA del proyecto SIN el robot, SIN
ROS2, SIN Docker y SIN pantallas/VNC. Corre en tu propia laptop (Windows,
Mac o Linux), solo necesita Python 3 + numpy + opencv-python.

Prueba las funciones "puras" de cada nodo (las que no dependen de ROS):
maze_solver.py (FSM, ley de control, ajuste de pared por regresion),
pare_detector.py (deteccion HSV del cartel PARE) y box_detector.py
(deteccion de karpinchus/cajas por LiDAR).

Uso (desde la carpeta del proyecto, en tu PC):
    pip install numpy opencv-python
    python test_logica_sin_robot.py

Si todo sale "OK", tu logica esta bien y el problema (si lo hay) esta del
lado del robot/Docker/sensores, no del codigo. Si algo sale "FAIL", te dice
exactamente que funcion y que se esperaba vs. que devolvio.
"""
import math
import sys
import unittest

sys.path.insert(0, ".")

from capytown_granprix_pkg.maze_solver import (  # noqa: E402
    sanitize, decide_state, follow_cmd, fit_wall_line,
    yaw_from_quat, ang_diff, Sectors, DEFAULTS,
)
from capytown_granprix_pkg.box_detector import detect_boxes_in_scan  # noqa: E402

try:
    from capytown_granprix_pkg.pare_detector import detect_red_sign
    _HAVE_CV = True
except Exception as exc:  # opencv/numpy no instalados
    _HAVE_CV = False
    _CV_ERR = exc


class TestSanitize(unittest.TestCase):
    def test_valores_normales_pasan_igual(self):
        self.assertEqual(sanitize(0.5, 0.1, 3.5), 0.5)

    def test_nan_inf_cero_se_reemplazan_por_range_max(self):
        self.assertEqual(sanitize(float("nan"), 0.1, 3.5), 3.5)
        self.assertEqual(sanitize(float("inf"), 0.1, 3.5), 3.5)
        self.assertEqual(sanitize(0.0, 0.1, 3.5), 3.5)
        self.assertEqual(sanitize(-1.0, 0.1, 3.5), 3.5)

    def test_recorta_a_range_min_max(self):
        self.assertEqual(sanitize(0.01, 0.1, 3.5), 0.1)
        self.assertEqual(sanitize(9.0, 0.1, 3.5), 3.5)


class TestDecideState(unittest.TestCase):
    """FSM: encerrado->RECOVER, apertura->TURN_OUT, frente bloqueado->TURN_IN,
    si no FOLLOW_WALL. Con side='right' (regla de la mano derecha)."""

    def setUp(self):
        self.p = dict(DEFAULTS)
        self.p["side"] = "right"

    def test_encerrado_por_3_lados_recover(self):
        # wall_block=0.14 por defecto: hay que estar MAS cerca que eso en
        # ambos lados (no solo cerca) para que cuente como "encerrado".
        s = Sectors(front=0.10, left=0.10, right=0.10)
        self.assertEqual(decide_state("FOLLOW_WALL", s, self.p), "RECOVER")

    def test_pared_derecha_desaparece_turn_out(self):
        s = Sectors(front=1.0, left=0.20, right=1.0)  # pared derecha se abrio
        self.assertEqual(decide_state("FOLLOW_WALL", s, self.p), "TURN_OUT")

    def test_frente_bloqueado_turn_in(self):
        s = Sectors(front=0.10, left=1.0, right=0.20)  # frente cerca, pared ok
        self.assertEqual(decide_state("FOLLOW_WALL", s, self.p), "TURN_IN")

    def test_todo_normal_follow_wall(self):
        s = Sectors(front=1.0, left=1.0, right=0.20)
        self.assertEqual(decide_state("FOLLOW_WALL", s, self.p), "FOLLOW_WALL")

    def test_side_izquierda_usa_pared_izquierda(self):
        p = dict(self.p)
        p["side"] = "left"
        s = Sectors(front=1.0, left=1.0, right=0.20)  # der. cerca no importa
        self.assertEqual(decide_state("FOLLOW_WALL", s, p), "TURN_OUT")


class TestFollowCmd(unittest.TestCase):
    """Disciplina de signo del PD: muy cerca -> se aleja; muy lejos -> se acerca."""

    def setUp(self):
        self.p = dict(DEFAULTS)
        self.p["side"] = "right"

    def test_muy_cerca_de_pared_derecha_gira_izquierda(self):
        s = Sectors(front=1.0, left=1.0, right=0.10)  # mas cerca que wall_target
        _, angular = follow_cmd(s, self.p)
        self.assertGreater(angular, 0.0, "err>0 (muy cerca) debe dar angular>0 (izquierda)")

    def test_muy_lejos_de_pared_derecha_gira_derecha(self):
        s = Sectors(front=1.0, left=1.0, right=0.45)  # mas lejos que wall_target
        _, angular = follow_cmd(s, self.p)
        self.assertLess(angular, 0.0, "err<0 (muy lejos) debe dar angular<0 (derecha)")

    def test_side_izquierda_espeja_el_signo(self):
        p = dict(self.p)
        p["side"] = "left"
        s = Sectors(front=1.0, left=0.10, right=1.0)
        _, angular = follow_cmd(s, p)
        self.assertLess(angular, 0.0, "con pared izquierda muy cerca, debe girar a la derecha (angular<0)")


class TestFitWallLine(unittest.TestCase):
    """Genera puntos LiDAR sinteticos de una pared recta a distancia/angulo
    conocidos y verifica que fit_wall_line los recupere."""

    def _puntos_pared_recta(self, distancia_m, angulo_deg, n=360,
                             lo_deg=70.0, hi_deg=110.0):
        angle_min = math.radians(-180.0)
        angle_inc = math.radians(360.0 / n)
        ranges = [5.0] * n
        ang_pared = math.radians(angulo_deg)
        for i in range(n):
            a = angle_min + i * angle_inc
            aa = math.atan2(math.sin(a), math.cos(a))
            if math.radians(lo_deg) <= aa <= math.radians(hi_deg):
                # distancia perpendicular constante -> recta a `distancia_m`,
                # rotada `angulo_deg` respecto al frente del robot.
                denom = math.cos(aa - ang_pared)
                if abs(denom) > 1e-6:
                    ranges[i] = distancia_m / denom
        return ranges, angle_min, angle_inc

    def test_pared_paralela_da_angulo_cero(self):
        ranges, amin, ainc = self._puntos_pared_recta(distancia_m=0.30, angulo_deg=90.0)
        angulo, distancia, valido = fit_wall_line(
            ranges, amin, ainc, 70.0, 110.0, 0.05, 3.5)
        self.assertTrue(valido)
        self.assertAlmostEqual(distancia, 0.30, delta=0.02)
        self.assertAlmostEqual(math.degrees(angulo), 0.0, delta=3.0)

    def test_pared_inclinada_10_grados_se_detecta(self):
        ranges, amin, ainc = self._puntos_pared_recta(distancia_m=0.30, angulo_deg=100.0)
        angulo, distancia, valido = fit_wall_line(
            ranges, amin, ainc, 70.0, 110.0, 0.05, 3.5)
        self.assertTrue(valido)
        self.assertAlmostEqual(math.degrees(angulo), 10.0, delta=3.0)

    def test_pocos_puntos_no_es_valido(self):
        ranges = [5.0] * 40  # todo fuera de rango -> ningun punto en la ventana
        angulo, distancia, valido = fit_wall_line(
            ranges, math.radians(-180.0), math.radians(9.0), 70.0, 110.0, 0.05, 3.5)
        self.assertFalse(valido)

    def test_outlier_no_arruina_el_ajuste(self):
        ranges, amin, ainc = self._puntos_pared_recta(distancia_m=0.30, angulo_deg=90.0, n=360)
        # mete un outlier bien lejos de la recta, justo en el centro de la
        # ventana (indice del angulo 90 = frente-izquierda)
        idx_90 = round((math.radians(90.0) - amin) / ainc) % len(ranges)
        ranges[idx_90] = 1.5
        angulo, distancia, valido = fit_wall_line(
            ranges, amin, ainc, 70.0, 110.0, 0.05, 3.5,
            outlier_iter=3, outlier_residual_m=0.03)
        self.assertTrue(valido)
        self.assertAlmostEqual(distancia, 0.30, delta=0.03,
                                msg="el filtro de outliers debe ignorar el punto espurio")


class TestYawAngDiff(unittest.TestCase):
    def test_cuaternion_identidad_es_yaw_cero(self):
        self.assertAlmostEqual(yaw_from_quat(0, 0, 0, 1), 0.0, places=5)

    def test_cuaternion_90_grados(self):
        # rotacion +90 en Z: (x,y,z,w) = (0,0,sin(45),cos(45))
        yaw = yaw_from_quat(0, 0, math.sin(math.radians(45)), math.cos(math.radians(45)))
        self.assertAlmostEqual(math.degrees(yaw), 90.0, delta=0.1)

    def test_ang_diff_toma_el_camino_mas_corto(self):
        # de 170 a -170 la diferencia corta es +20, no -340
        d = ang_diff(math.radians(-170), math.radians(170))
        self.assertAlmostEqual(math.degrees(d), 20.0, delta=0.1)


class TestBoxDetector(unittest.TestCase):
    """Genera un /scan sintetico: una pared plana con una protuberancia de
    20 cm (caja) y verifica que detect_boxes_in_scan la encuentre, y que
    una pared plana SIN protuberancia no de falsos positivos."""

    def test_pared_plana_sin_cajas(self):
        n = 200
        angle_min = math.radians(-100)
        angle_inc = math.radians(200.0 / n)
        ranges = [1.0] * n  # pared plana a 1m, sin nada que sobresalga
        boxes = detect_boxes_in_scan(ranges, angle_min, angle_inc, 0.05, 5.0)
        self.assertEqual(len(boxes), 0)

    def test_protuberancia_de_20cm_se_detecta_como_caja(self):
        n = 200
        angle_min = math.radians(-100)
        angle_inc = math.radians(200.0 / n)
        ranges = [1.0] * n
        # una caja de 20cm de ancho a 0.7m (sobresale 0.3m de la pared a 1m),
        # centrada al frente (indice medio)
        centro = n // 2
        ancho_rad = 0.20 / 0.7  # ancho angular aproximado = ancho/distancia
        medio_pasos = max(1, int((ancho_rad / angle_inc) / 2))
        for i in range(centro - medio_pasos, centro + medio_pasos + 1):
            ranges[i] = 0.7
        boxes = detect_boxes_in_scan(ranges, angle_min, angle_inc, 0.05, 5.0)
        self.assertGreaterEqual(len(boxes), 1, "deberia detectar la protuberancia de 20cm como caja")


@unittest.skipUnless(_HAVE_CV, "opencv-python/numpy no instalados - instala con: pip install opencv-python numpy")
class TestPareDetector(unittest.TestCase):
    """Genera imagenes sinteticas (sin camara real) para probar la
    deteccion del cartel PARE por color+forma."""

    def test_cuadrado_rojo_se_detecta_como_pare(self):
        import numpy as np
        img = np.zeros((240, 320, 3), dtype="uint8")  # BGR negro
        # cuadrado rojo (BGR: rojo puro = (0,0,255)) tipo cartel, compacto
        img[80:160, 120:200] = (0, 0, 220)
        found, bbox, area, mask, contour = detect_red_sign(img)
        self.assertTrue(found, "un cuadrado rojo compacto debe detectarse como PARE")

    def test_imagen_sin_rojo_no_da_falso_positivo(self):
        import numpy as np
        img = np.full((240, 320, 3), (200, 200, 200), dtype="uint8")  # gris, sin rojo
        found, bbox, area, mask, contour = detect_red_sign(img)
        self.assertFalse(found, "una imagen sin rojo no debe disparar un PARE")

    def test_franja_roja_delgada_no_es_pare(self):
        import numpy as np
        img = np.zeros((240, 320, 3), dtype="uint8")
        img[100:110, 20:300] = (0, 0, 220)  # franja larga y delgada, no un cartel
        found, bbox, area, mask, contour = detect_red_sign(img)
        self.assertFalse(found, "una franja roja delgada (aspecto no cuadrado) no debe ser PARE")


if __name__ == "__main__":
    if not _HAVE_CV:
        print("[aviso] pare_detector no se pudo importar del todo (" + str(_CV_ERR) + "); "
              "sus tests se saltan. Instala con: pip install opencv-python numpy\n")
    unittest.main(verbosity=2)
