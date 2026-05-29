#!/usr/bin/env python3
"""
Grabador de waypoints — conduce el carro manualmente y graba el recorrido completo.

Uso:
  1. Lanza CARLA en modo servidor (Terminal 1):
     cd ~/opt/CarlaSimulator
     ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=20

  2. Ejecuta este grabador (Terminal 2):
     python3 recorder.py

  3. Conduce con W / A / S / D en la ventana pygame.
     Presiona Q cuando termines la vuelta completa → guarda mis_waypoints.txt
"""
from __future__ import print_function
import sys
import os
import math
import time

import pygame

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from carla.client import make_carla_client, VehicleControl
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError

OUTPUT_FILE  = os.path.join(os.path.dirname(__file__), 'mis_waypoints.txt')
RECORD_DIST  = 0.5   # grabar un punto cada 0.5 metros
STEER_MAX      = 0.3   # máximo ángulo de giro normalizado
STEER_RATE     = 0.02  # cuánto aumenta el giro por frame al mantener la tecla
STEER_DECAY    = 0.04  # cuánto vuelve al centro por frame al soltar la tecla
THROTTLE_MAX   = 0.35  # aceleración máxima
THROTTLE_RATE  = 0.02  # cuánto sube el acelerador por frame al mantener W
THROTTLE_DECAY = 0.05  # cuánto baja al soltar W
BRAKE_MAX      = 0.6


def make_settings():
    s = CarlaSettings()
    s.set(SynchronousMode=True,
          SendNonPlayerAgentsInfo=False,
          NumberOfVehicles=0,
          NumberOfPedestrians=0,
          WeatherId=1,
          QualityLevel='Low')
    return s


def keyboard_control(keys, steer_prev, throttle_prev):
    """Acelerador y giro suaves: acumulan/decaen gradualmente."""
    # Acelerador
    if keys[pygame.K_w] or keys[pygame.K_UP]:
        throttle = min(throttle_prev + THROTTLE_RATE, THROTTLE_MAX)
    else:
        throttle = max(throttle_prev - THROTTLE_DECAY, 0.0)

    # Freno
    brake = BRAKE_MAX if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0

    # Dirección
    if keys[pygame.K_a] or keys[pygame.K_LEFT]:
        steer = max(steer_prev - STEER_RATE, -STEER_MAX)
    elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        steer = min(steer_prev + STEER_RATE,  STEER_MAX)
    else:
        if   steer_prev >  STEER_DECAY: steer = steer_prev - STEER_DECAY
        elif steer_prev < -STEER_DECAY: steer = steer_prev + STEER_DECAY
        else: steer = 0.0

    return throttle, steer, brake


def main():
    pygame.init()
    screen = pygame.display.set_mode((420, 160))
    pygame.display.set_caption('Grabador — W/A/S/D para conducir  |  Q = guardar')
    font  = pygame.font.SysFont('monospace', 16)
    clock = pygame.time.Clock()

    waypoints         = []
    last_x, last_y    = None, None
    current_steer     = 0.0
    current_throttle  = 0.0

    try:
        with make_carla_client('localhost', 2000) as client:
            client.load_settings(make_settings())
            client.start_episode(1)
            print("Conectado a CARLA. Conduce con W/A/S/D. Q = guardar y salir.")

            running = True
            while running:
                clock.tick(60)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                        running = False

                keys = pygame.key.get_pressed()
                current_throttle, current_steer, brake = keyboard_control(
                    keys, current_steer, current_throttle)
                throttle = current_throttle
                steer    = current_steer

                measurements, _    = client.read_data()
                pm                 = measurements.player_measurements
                x  = pm.transform.location.x
                y  = pm.transform.location.y
                v  = pm.forward_speed

                # Grabar punto si el carro avanzó al menos RECORD_DIST metros
                if last_x is None or \
                   math.hypot(x - last_x, y - last_y) >= RECORD_DIST:
                    waypoints.append((x, y, max(v, 0.5)))
                    last_x, last_y = x, y

                control          = VehicleControl()
                control.throttle = throttle
                control.steer    = steer
                control.brake    = brake
                client.send_control(control)

                # Pantalla de estado
                screen.fill((20, 20, 20))
                screen.blit(font.render(
                    f'Posicion : ({x:.1f}, {y:.1f})',    True, (200, 200, 200)), (10, 15))
                screen.blit(font.render(
                    f'Velocidad: {v * 3.6:.1f} km/h',   True, (200, 200, 200)), (10, 40))
                screen.blit(font.render(
                    f'Puntos grabados: {len(waypoints)}', True, (100, 255, 100)), (10, 65))
                screen.blit(font.render(
                    'W/A/S/D = conducir   Q = guardar',  True, (255, 200,  50)), (10, 95))
                pygame.display.flip()

    except TCPConnectionError as e:
        print(f"Error de conexion: {e}")
        print("Asegurate de que CARLA este corriendo en modo servidor.")
    finally:
        pygame.quit()

    if waypoints:
        with open(OUTPUT_FILE, 'w') as f:
            for wp in waypoints:
                f.write(f'{wp[0]}, {wp[1]}, {wp[2]}\n')
        print(f"\nGuardados {len(waypoints)} waypoints en:\n  {OUTPUT_FILE}")
    else:
        print("No se grabaron waypoints.")


if __name__ == '__main__':
    main()
