# Proyecto Final – Control Longitudinal y Lateral en CARLA 0.8.4

## Descripción
Controlador autónomo que completa el circuito RaceTrack en CARLA 0.8.4
usando PID (longitudinal) y Pure Pursuit (lateral).

##LA CARPETA SE DEBE INSTALAR EN ~/opt/CarlaSimulator/PythonClient#

## Arquitectura (3 archivos)
| Archivo | Rol |
|---------|-----|
| `main.py` | Punto de entrada, conexión CARLA, bucle de control |
| `pid_controller.py` | Control longitudinal PID (throttle / brake) |
| `pure_pursuit.py` | Control lateral Pure Pursuit (steering) |

## Sensores usados (mínimo para bonificación +3s)
- `forward_speed` → velocidad actual del vehículo
- `transform` (location + rotation) → posición (x, y) y orientación (yaw)

## Dependencias
```
pip3 install numpy
```

## Ejecución

### 1. Lanzar el servidor CARLA (Terminal 1)
```bash
cd ~/opt/CarlaSimulator
./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server 
```

### 2. Ejecutar el controlador (Terminal 2)
```bash
cd ~/opt/CarlaSimulator/PythonClient/ProyectoFinal
python3 main.py
```

### Opciones de línea de comando
```
python3 main.py --host localhost --port 2000 --start 1
```
- `--start N` : índice de posición de inicio (dado por el profesor el día de la competencia)

## Parámetros ajustables (en main.py)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `PLAYER_START_INDEX` | 1 | Punto de partida en el mapa |
| `MAX_SPEED` | 20.0 m/s | Velocidad máxima (~72 km/h) |
| `SPEED_MULTIPLIER` | 1.0 | Factor de escala sobre velocidad del waypoint |
| `SPEED_PREVIEW` | 40 | Waypoints adelante para anticipar frenado en curvas |
| `FINISH_X` | -184.0 | Coordenada X de la línea de meta |
| `FINISH_Y` | -12.1 | Coordenada Y de la línea de meta |
| `FINISH_RADIUS` | 8.0 m | Radio de detección de meta |

## Parámetros ajustables (en pid_controller.py)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `KP` | 1.0 | Ganancia proporcional |
| `KI` | 0.2 | Ganancia integral |
| `KD` | 0.01 | Ganancia derivativa |
| `MAX_THROTTLE_DELTA` | 0.1 | Límite de cambio de acelerador por paso (anti-jerk) |

## Parámetros ajustables (en pure_pursuit.py)
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `LOOKAHEAD_GAIN` | 0.5 | Lookahead adaptativo = GAIN × velocidad + MIN |
| `LOOKAHEAD_MIN` | 5.0 m | Anticipación mínima |
| `LOOKAHEAD_MAX` | 20.0 m | Anticipación máxima |
| `WHEELBASE` | 2.5 m | Batalla del vehículo |
