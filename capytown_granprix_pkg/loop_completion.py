#!/usr/bin/env python3
"""
loop_completion.py — detección de VUELTA COMPLETA por odometría (pieza de FABLE).
================================================================================
El reto "El Censo y el Guardián de las Cajas" es un CIRCUITO EN LAZO (corredor
anular alrededor de una isla, INICIO=META). Un wall-follower puro recorre el lazo
y vuelve a INICIO, pero NO SABE PARAR → patrulla infinito. Esto cierra ese gap.

Lógica (máquina de estados sobre la pose de /odom):
  1. Guarda la pose de INICIO en el primer update.
  2. ARMED → el robot debe ALEJARSE más de `away_dist` de INICIO (evita disparar al
     arrancar) → pasa a AWAY.
  3. AWAY → cuando vuelve a estar dentro de `return_dist` de INICIO **y** ya recorrió
     un camino ≥ `min_path` (evita falsos por jitter) → VUELTA COMPLETA → señal de PARAR.

Puro Python, sin ROS, importable y testeable. ADA/NEXUS lo enchufan al behavior_fsm:
le pasan (x,y) de /odom cada tick; cuando devuelve True, el FSM frena el robot.
Autor: FABLE · 2026-06-24 · verificado por efecto.
"""
import math


class LoopCompletion:
    def __init__(self, away_dist=1.0, return_dist=0.35, min_path=3.0,
                 min_turn=math.radians(225.0)):
        """
        away_dist:   m que hay que alejarse de INICIO para 'armar' el retorno.
        return_dist: m de cercanía a INICIO que cuenta como 'volví'.
        min_path:    m mínimos de camino recorrido para validar el lazo (anti-jitter).
        min_turn:    rad de ROTACIÓN NETA exigida para validar el lazo. Una vuelta REAL
                     acumula ~±270-360° al volver al inicio; un OUT-AND-BACK (sale y un
                     giro de 180° lo manda de vuelta) acumula ~±180° → NO es una vuelta.
                     Umbral en 225° (entre los dos) y favoreciendo NO falso-bloquear una
                     vuelta real (un falso-bloqueo = patrulla infinito = falla la tarea;
                     peor que parar de más). Solo se aplica si se pasa yaw a update().
                     Cura el falso-positivo del retroceso (robot de Henry, 25-jun-2026).
        """
        self.away_dist = away_dist
        self.return_dist = return_dist
        self.min_path = min_path
        self.min_turn = min_turn
        self.start = None          # (x,y) de INICIO
        self.prev = None
        self.path_len = 0.0
        self.cum_turn = 0.0        # rotación NETA acumulada (rad), si se provee yaw
        self.prev_yaw = None
        self.state = "ARMED"       # ARMED -> AWAY -> DONE
        self.went_away = False

    def reset(self):
        self.__init__(self.away_dist, self.return_dist, self.min_path, self.min_turn)

    def update(self, x, y, yaw=None):
        """Llamar cada tick con la pose de /odom (y el yaw si se tiene). Devuelve True si
        la vuelta se completó. Si se pasa yaw, además exige rotación NETA >= min_turn (una
        vuelta real gira ~360°, un retroceso no) → evita el falso-positivo del giro 180°."""
        if self.start is None:
            self.start = (x, y)
            self.prev = (x, y)
            self.prev_yaw = yaw
            return False
        # acumula camino recorrido
        self.path_len += math.hypot(x - self.prev[0], y - self.prev[1])
        self.prev = (x, y)
        # acumula rotación NETA (diferencia angular envuelta) si hay yaw
        if yaw is not None:
            if self.prev_yaw is not None:
                dyaw = math.atan2(math.sin(yaw - self.prev_yaw),
                                  math.cos(yaw - self.prev_yaw))
                self.cum_turn += dyaw
            self.prev_yaw = yaw
        d_start = math.hypot(x - self.start[0], y - self.start[1])

        if self.state == "ARMED":
            if d_start > self.away_dist:
                self.state = "AWAY"
                self.went_away = True
        elif self.state == "AWAY":
            # gate de rotación: si hay yaw, exigir que haya girado como una vuelta real
            turn_ok = (yaw is None) or (abs(self.cum_turn) >= self.min_turn)
            if d_start < self.return_dist and self.path_len >= self.min_path and turn_ok:
                self.state = "DONE"
                return True
        return self.state == "DONE"

    @property
    def done(self):
        return self.state == "DONE"


# ───────────────────────── autotest POR EFECTO ─────────────────────────
if __name__ == "__main__":
    import sys
    ok = True

    def run(traj, **kw):
        lc = LoopCompletion(**kw)
        fired_at = None
        for i, (x, y) in enumerate(traj):
            if lc.update(x, y) and fired_at is None:
                fired_at = i
        return fired_at

    # TEST 1: lazo rectangular (corredor anular ~ perímetro), vuelve a INICIO → debe disparar AL VOLVER
    print("═══ TEST 1: lazo completo (debe disparar al volver, no antes) ═══")
    loop = []
    # rectángulo 3x1.8 (como la pista), pasos de 0.1m, sentido horario desde (0,0)
    def seg(p0, p1, step=0.1):
        n = max(1, int(math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / step))
        return [(p0[0] + (p1[0]-p0[0])*t/n, p0[1] + (p1[1]-p0[1])*t/n) for t in range(n)]
    pts = [(0, 0), (2.4, 0), (2.4, 1.2), (0, 1.2), (0, 0)]
    for a, b in zip(pts, pts[1:]):
        loop += seg(a, b)
    f = run(loop, away_dist=1.0, return_dist=0.35, min_path=4.0)
    # debe disparar cerca del final (al volver a 0,0), NO en los primeros pasos
    good1 = f is not None and f > len(loop)*0.8
    ok = ok and good1
    print(f"  disparó en paso {f}/{len(loop)} (esperado: cerca del final) [{'ok' if good1 else 'FAIL'}]")

    # TEST 2: jitter cerca de INICIO sin dar la vuelta → NO debe disparar (falso positivo)
    print("═══ TEST 2: jitter en INICIO (NO debe disparar) ═══")
    jitter = [(0.05*math.sin(i), 0.05*math.cos(i)) for i in range(200)]
    f2 = run(jitter, away_dist=1.0, return_dist=0.35, min_path=4.0)
    good2 = f2 is None
    ok = ok and good2
    print(f"  disparó: {f2} (esperado: None) [{'ok' if good2 else 'FAIL'}]")

    # TEST 3: media vuelta (se aleja pero no vuelve) → NO debe disparar
    print("═══ TEST 3: media vuelta sin retornar (NO debe disparar) ═══")
    half = seg((0, 0), (2.4, 0)) + seg((2.4, 0), (2.4, 1.2))
    f3 = run(half, away_dist=1.0, return_dist=0.35, min_path=4.0)
    good3 = f3 is None
    ok = ok and good3
    print(f"  disparó: {f3} (esperado: None) [{'ok' if good3 else 'FAIL'}]")

    # --- gate de ROTACIÓN NETA (con yaw): cura el falso-positivo del retroceso ---
    def run_yaw(traj):  # traj = lista de (x, y, yaw)
        lc = LoopCompletion(away_dist=1.0, return_dist=0.35, min_path=4.0)
        fired = None
        for i, (x, y, yaw) in enumerate(traj):
            if lc.update(x, y, yaw) and fired is None:
                fired = i
        return fired, lc.cum_turn

    # TEST 4: OUT-AND-BACK (sale, gira 180° en el lugar, vuelve al INICIO) → NO debe disparar.
    #         Sin el gate, posición+camino lo dispararían (el bug del robot de Henry).
    print("═══ TEST 4: out-and-back con yaw (NO debe disparar; cura el falso 180°) ═══")
    oab = [(i * 0.1, 0.0, 0.0) for i in range(25)]                  # ida, mirando +x
    oab += [(2.4, 0.0, k * math.pi / 18) for k in range(1, 19)]     # giro 180° en el lugar
    oab += [(2.4 - i * 0.1, 0.0, math.pi) for i in range(25)]       # vuelta, mirando -x
    f4, turn4 = run_yaw(oab)
    good4 = f4 is None
    ok = ok and good4
    print(f"  disparó: {f4} (esperado None) · rotación neta={math.degrees(turn4):.0f}° "
          f"(~180° = no es vuelta) [{'ok' if good4 else 'FAIL'}]")

    # TEST 5: VUELTA REAL con yaw (lazo rectangular, gira ~270° al volver) → SÍ debe disparar.
    print("═══ TEST 5: vuelta real con yaw (debe disparar) ═══")
    loop5 = []
    corners = [(0, 0), (2.4, 0), (2.4, 1.2), (0, 1.2), (0, 0)]
    for a, b in zip(corners, corners[1:]):
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        loop5 += [(x, y, h) for (x, y) in seg(a, b)]
    f5, turn5 = run_yaw(loop5)
    good5 = f5 is not None
    ok = ok and good5
    print(f"  disparó en {f5}/{len(loop5)} · rotación neta={math.degrees(turn5):.0f}° "
          f"(~270° = vuelta real) [{'ok' if good5 else 'FAIL'}]")

    print(f"\n{'✅ LOOP-COMPLETION FUNCIONA' if ok else '⚠️ revisar'} — verificado por efecto")
    sys.exit(0 if ok else 1)
