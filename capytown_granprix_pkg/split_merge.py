#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
split_merge.py — Extracción de LÍNEAS de un scan LiDAR 2D por Split & Merge (pieza JARVIS).
=============================================================================================
Reto CapyTown (Henry): el enunciado pide Split & Merge para el LiDAR. Este módulo convierte
un LaserScan en SEGMENTOS DE LÍNEA (las paredes del laberinto/circuito), que alimentan:
  • wall-following (seguir la línea de pared más cercana)
  • detección de cajas (una caja = segmento corto que SOBRESALE de la línea de pared)

Algoritmo:
  SPLIT (recursivo): ajusta una recta entre los extremos del conjunto de puntos, halla el punto
    de MÁXIMA distancia perpendicular a esa recta; si supera `split_thresh` y hay suficientes
    puntos → parte ahí en dos sub-conjuntos y recursa.
  MERGE: une segmentos adyacentes si, fusionados, su máxima desviación a la recta combinada
    sigue bajo `merge_thresh` (casi colineales).

Solo numpy → corre en el Pi/edge. Parametrizado; afinar umbrales con el /scan real del robot.
Autor: JARVIS · 2026-06-25.
"""
from __future__ import annotations
import numpy as np

# umbrales por defecto (metros) — AFINAR con el sample real del MS200 de Henry
SPLIT_THRESH = 0.05      # 5 cm de desviación máx. antes de partir
MERGE_THRESH = 0.05      # tolerancia para fusionar colineales
MIN_POINTS   = 6         # mín. puntos para considerar una línea (filtra ruido)
MIN_SEG_LEN  = 0.10      # longitud mínima de segmento (m)


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float = 0.05, range_max: float = 12.0) -> np.ndarray:
    """LaserScan → puntos (x,y) en el frame del LiDAR. Filtra inf/nan/fuera-de-rango."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(len(r)) * angle_increment
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[valid], ang[valid]
    return np.column_stack((r * np.cos(ang), r * np.sin(ang)))


def _cross2d(ab: np.ndarray, ap: np.ndarray) -> np.ndarray:
    """Producto cruz 2D (escalar) — evita el DeprecationWarning de np.cross con vectores 2D."""
    return ab[0] * ap[..., 1] - ab[1] * ap[..., 0]


def _max_dist_to_line(pts: np.ndarray, a: np.ndarray, b: np.ndarray):
    """Distancia perpendicular máxima de `pts` a la recta a→b. Devuelve (idx, dist)."""
    ab = b - a
    L = np.hypot(*ab)
    if L < 1e-9:
        d = np.hypot(*(pts - a).T)
    else:
        d = np.abs(_cross2d(ab, pts - a)) / L   # distancia perpendicular punto-recta
    i = int(np.argmax(d))
    return i, float(d[i])


def _split(pts: np.ndarray, split_thresh: float, min_points: int) -> list:
    """Devuelve lista de (i0, i1) índices [inclusive] de segmentos tras el split recursivo."""
    n = len(pts)
    if n < min_points:
        return [(0, n - 1)] if n >= 2 else []
    segs = []
    stack = [(0, n - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 - i0 + 1 < 2:
            continue
        sub = pts[i0:i1 + 1]
        idx, dmax = _max_dist_to_line(sub, pts[i0], pts[i1])
        if dmax > split_thresh and (i1 - i0) >= 2:
            mid = i0 + idx
            if mid == i0 or mid == i1:   # no progreso → corta
                segs.append((i0, i1)); continue
            stack.append((i0, mid)); stack.append((mid, i1))
        else:
            segs.append((i0, i1))
    return sorted(segs)


def _merge(pts: np.ndarray, segs: list, merge_thresh: float) -> list:
    """Fusiona segmentos adyacentes casi colineales."""
    if not segs:
        return []
    out = [segs[0]]
    for s in segs[1:]:
        i0, _ = out[-1]
        _, j1 = s
        sub = pts[i0:j1 + 1]
        _, dmax = _max_dist_to_line(sub, pts[i0], pts[j1])
        if dmax <= merge_thresh:
            out[-1] = (i0, j1)            # fusiona
        else:
            out.append(s)
    return out


def split_and_merge(points: np.ndarray, split_thresh: float = SPLIT_THRESH,
                    merge_thresh: float = MERGE_THRESH, min_points: int = MIN_POINTS,
                    min_seg_len: float = MIN_SEG_LEN) -> list:
    """Devuelve lista de líneas como ((x0,y0),(x1,y1)) — las paredes extraídas del scan."""
    if len(points) < 2:
        return []
    segs = _merge(points, _split(points, split_thresh, min_points), merge_thresh)
    lines = []
    for i0, i1 in segs:
        if i1 - i0 + 1 < min_points:
            continue
        a, b = points[i0], points[i1]
        if np.hypot(*(b - a)) >= min_seg_len:
            lines.append((tuple(a), tuple(b)))
    return lines


def detect_boxes(lines: list, box_min: float = 0.05, box_max: float = 0.50,
                 wall_min: float = 0.80) -> list:
    """Caja candidata = SEGMENTO CORTO (longitud en [box_min, box_max]) — Split & Merge ya
    separa la cara de la caja como su propia línea corta, distinta de las paredes largas
    (>= wall_min). Devuelve los centros (midpoints) de esos segmentos cortos = caras de caja.
    Apoyo al censo de las 5 cajas del reto CapyTown."""
    boxes = []
    for (a, b) in lines:
        a, b = np.asarray(a), np.asarray(b)
        L = float(np.hypot(*(b - a)))
        if box_min <= L <= box_max:        # segmento corto = cara de caja, no pared
            boxes.append(tuple((a + b) / 2.0))
    return boxes


# ───────────────────────── self-test POR EFECTO ─────────────────────────
if __name__ == "__main__":
    # escena sintética: un CORNER (2 paredes en L) + una caja sobresaliendo
    np.random.seed(0)
    wall1 = np.column_stack((np.linspace(0, 2, 60), np.full(60, 1.0)))      # pared horizontal y=1
    wall2 = np.column_stack((np.full(60, 2.0), np.linspace(1, 3, 60)))      # pared vertical x=2
    box   = np.column_stack((np.linspace(0.8, 1.0, 12), np.full(12, 0.6))) # caja sobresale (y=0.6)
    pts = np.vstack((wall1, box, wall2)) + np.random.normal(0, 0.01, (132, 2))
    lines = split_and_merge(pts, split_thresh=0.05)
    boxes = detect_boxes(lines)
    print(f"[Split&Merge] puntos={len(pts)} → líneas extraídas={len(lines)} (esperado ~2-3 paredes)")
    for a, b in lines:
        print(f"   línea ({a[0]:.2f},{a[1]:.2f})→({b[0]:.2f},{b[1]:.2f})  len={np.hypot(b[0]-a[0],b[1]-a[1]):.2f}m")
    print(f"[cajas] protrusiones detectadas={len(boxes)} (esperado ~1): {[(round(x,2),round(y,2)) for x,y in boxes]}")
    ok = len(lines) >= 2 and len(boxes) >= 1
    print("✅ Split & Merge FUNCIONA por efecto" if ok else "⚠️ revisar umbrales")
