#!/usr/bin/env python3
"""
Proyecto Final – Robótica Móvil
Control Longitudinal (PID) y Lateral (Pure Pursuit) en CARLA 0.8.4

Uso:
    python3 main.py [--host HOST] [--port PORT] [--start START_INDEX]

El servidor CARLA debe estar corriendo antes de ejecutar este script.
"""
from __future__ import print_function
import sys
import os
import time
import math
import argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from carla.client     import make_carla_client, VehicleControl
from carla.settings   import CarlaSettings
from carla.tcp        import TCPConnectionError
from pid_controller   import PIDController
from pure_pursuit     import PurePursuitController


# ── Parámetros configurables (el profesor puede pedirte cambiarlos) ────────
HOST               = 'localhost'
PORT               = 2000
PLAYER_START_INDEX = 1        # Posición de inicio (dada el día de competencia)
MAX_SPEED          = 20.0     # Velocidad máxima (m/s) — ~72 km/h, rectas del circuito
SPEED_MULTIPLIER   = 1.0      # Factor de escala (>1 más rápido, <1 más lento)
SPEED_PREVIEW      = 40       # Waypoints hacia adelante para anticipar frenado
FINISH_X           = -184.0   # Coordenada X de la línea de meta
FINISH_Y           = -12.1    # Coordenada Y de la línea de meta
FINISH_RADIUS      = 8.0      # Radio de detección de meta (m)
# Usa mis_waypoints.txt (grabado manualmente) si existe, si no el de Coursera
_own   = os.path.join(os.path.dirname(__file__), 'mis_waypoints.txt')
_orig  = os.path.join(os.path.dirname(__file__),
                      '../Course1FinalProject/racetrack_waypoints.txt')
WAYPOINTS_FILE = _own if os.path.exists(_own) else _orig
# ──────────────────────────────────────────────────────────────────────────


def load_waypoints(path):
    """Lee el archivo de waypoints CSV con formato: x, y, velocidad."""
    waypoints = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 3:
                waypoints.append([float(p) for p in parts])
    return waypoints


def make_settings():
    """Configura CARLA con los mínimos sensores necesarios."""
    settings = CarlaSettings()
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=False,
        NumberOfVehicles=0,
        NumberOfPedestrians=0,
        WeatherId=1,          # CLEARNOON
        QualityLevel='Low')   # Low para mejor rendimiento
    return settings


def get_pose(measurements):
    """Extrae x, y, yaw (rad) y velocidad del mensaje de medición."""
    pm  = measurements.player_measurements
    x   = pm.transform.location.x
    y   = pm.transform.location.y
    yaw = math.radians(pm.transform.rotation.yaw)
    v   = pm.forward_speed
    return x, y, yaw, v


def find_closest_index(waypoints, x, y):
    """Encuentra el índice del waypoint más cercano a (x, y)."""
    pos    = np.array([x, y])
    wp_arr = np.array([[w[0], w[1]] for w in waypoints])
    dists  = np.linalg.norm(wp_arr - pos, axis=1)
    return int(np.argmin(dists))


def desired_speed(waypoints, progress, start_idx):
    """Velocidad deseada: mínimo en la ventana de preview para frenar antes de curvas."""
    n = len(waypoints)
    speeds = [waypoints[(start_idx + progress + i) % n][2]
              for i in range(SPEED_PREVIEW)]
    raw = min(speeds)
    return float(np.clip(raw * SPEED_MULTIPLIER, 0.0, MAX_SPEED))


def update_progress(wp_xy, x, y, start_idx, progress, n):
    """Avanza el índice circular sin retroceso (máx 50 wp por paso)."""
    pos      = np.array([x, y])
    closest  = int(np.argmin(np.linalg.norm(wp_xy - pos, axis=1)))
    new_prog = (closest - start_idx) % n
    delta    = (new_prog - progress) % n
    return new_prog if delta < 50 else progress


def compute_and_send(client, pid, pursuit, waypoints, x, y, yaw, v, dt,
                     start_idx, progress, n):
    """Calcula throttle, freno y dirección, y los envía al servidor."""
    window          = [waypoints[(start_idx + progress + i) % n] for i in range(50)]
    v_des           = desired_speed(waypoints, progress, start_idx)
    throttle, brake = pid.compute(v_des, v, dt)
    steer           = pursuit.compute(x, y, yaw, v, window)
    control          = VehicleControl()
    control.throttle = float(np.clip(throttle, 0.0, 1.0))
    control.steer    = float(np.clip(steer,    -1.0, 1.0))
    control.brake    = float(np.clip(brake,    0.0, 1.0))
    client.send_control(control)


def run_episode(client, waypoints, pid, pursuit, start_idx):
    """Bucle principal de control hasta completar la vuelta."""
    n            = len(waypoints)
    prev_t       = None
    progress     = 0
    lap_started  = False
    start_real   = None
    last_print_t = 0.0
    wp_xy        = np.array([[w[0], w[1]] for w in waypoints])
    finish       = np.array([FINISH_X, FINISH_Y])

    while True:
        cycle_t0        = time.time()
        measurements, _ = client.read_data()
        x, y, yaw, v    = get_pose(measurements)
        t               = measurements.game_timestamp / 1000.0

        if prev_t is None:
            prev_t = t
            client.send_control(VehicleControl())
            continue

        dt, prev_t = max(t - prev_t, 1e-4), t

        if not lap_started and v > 0.5:
            lap_started, start_real = True, time.time()
            print("Vuelta iniciada. Temporizador en curso...")
            print("-" * 35)

        progress = update_progress(wp_xy, x, y, start_idx, progress, n)
        compute_and_send(client, pid, pursuit, waypoints, x, y, yaw, v, dt,
                         start_idx, progress, n)

        if lap_started:
            elapsed = time.time() - start_real
            if elapsed - last_print_t >= 1.0:
                mm, ss = int(elapsed) // 60, elapsed % 60.0
                print(f"  {mm:02d}:{ss:05.2f}  |  {v*3.6:5.1f} km/h  |  wp {progress}/{n}",
                      end='\r', flush=True)
                last_print_t = elapsed

        pos = np.array([x, y])
        if lap_started and progress > 1000 and np.linalg.norm(pos - finish) < FINISH_RADIUS:
            elapsed = time.time() - start_real
            mm, ss  = int(elapsed) // 60, elapsed % 60.0
            print(f"\n{'='*35}\n  VUELTA COMPLETADA: {mm:02d}:{ss:05.2f}\n{'='*35}")
            break


def main():
    parser = argparse.ArgumentParser(description='Proyecto Final CARLA')
    parser.add_argument('--host',  default=HOST,               help='IP del servidor CARLA')
    parser.add_argument('--port',  default=PORT,  type=int,    help='Puerto del servidor')
    parser.add_argument('--start', default=PLAYER_START_INDEX, type=int,
                        help='Índice de posición de inicio')
    args = parser.parse_args()

    waypoints = load_waypoints(WAYPOINTS_FILE)
    pid       = PIDController()
    pursuit   = PurePursuitController()

    try:
        with make_carla_client(args.host, args.port) as client:
            print("Conectado a CARLA.")
            settings = make_settings()
            client.load_settings(settings)
            client.start_episode(args.start)

            # Esperar un frame para obtener la posición inicial real
            measurements, _ = client.read_data()
            x0, y0, _, _    = get_pose(measurements)
            start_idx       = find_closest_index(waypoints, x0, y0)
            print(f"Posición inicial: ({x0:.1f}, {y0:.1f})  →  waypoint #{start_idx}")
            client.send_control(VehicleControl())

            run_episode(client, waypoints, pid, pursuit, start_idx)

    except TCPConnectionError as e:
        print(f"Error de conexión: {e}")
        print("Asegúrate de que el servidor CARLA esté corriendo.")
        sys.exit(1)


if __name__ == '__main__':
    main()
