# Proyecto Final – Control Longitudinal y Lateral en CARLA 0.8.4

Controlador autónomo que completa el circuito **RaceTrack** en CARLA 0.8.4
usando **PID** (longitudinal) y **Pure Pursuit** (lateral).

Curso: Robótica Móvil
Autor: 

- Nicolás Díaz
- Santiago Hernández 
- Daniel Pinilla

---

## Requisitos previos

1. **CARLA Simulator 0.8.4** instalado en el equipo.
   Descarga oficial: https://github.com/carla-simulator/carla/releases/tag/0.8.4
2. **Python 3** con `numpy`:
   ```bash
   pip3 install numpy
   ```
3. (Opcional, solo si se desea volver a grabar la ruta con `recorder.py`):
   ```bash
   pip3 install pygame
   ```

---

## Instalación (IMPORTANTE)

> Los scripts dependen del módulo Python `carla` que viene **dentro** de la
> carpeta `PythonClient/` de CARLA Simulator. Por eso esta carpeta debe
> colocarse **dentro de** `PythonClient/`. No funciona si se descarga en otro
> lado.

### Paso a paso

1. Localiza dónde tienes instalado CARLA Simulator 0.8.4. La carpeta raíz
   contiene `CarlaUE4.sh`, `PythonClient/`, etc.
   Llamémosla `<CARLA_ROOT>`. Ejemplos típicos:
   - Linux: `~/opt/CarlaSimulator/` o `~/CarlaSimulator/`
   - Windows: `C:\CarlaSimulator\`

2. Clona o descarga este repositorio **dentro** de `<CARLA_ROOT>/PythonClient/`:

   **Opción A — con git:**
   ```bash
   cd <CARLA_ROOT>/PythonClient
   git clone https://github.com/shdeza12/Proyecto-final-Robotica-Movil.git ProyectoFinal
   ```

   **Opción B — descargando el ZIP desde GitHub:**
   - Descarga el ZIP desde la página del repo (botón verde "Code" → "Download ZIP").
   - Descomprímelo dentro de `<CARLA_ROOT>/PythonClient/`.
   - Renombra la carpeta resultante a `ProyectoFinal` (sin el sufijo `-main` o `-master`).

3. Verifica que la estructura final sea esta:
   ```
   <CARLA_ROOT>/
   ├── CarlaUE4.sh
   └── PythonClient/
       ├── carla/                  ← módulo del simulador (ya existe)
       ├── Course1FinalProject/    ← otro contenido del simulador
       └── ProyectoFinal/          ← ESTE repo
           ├── main.py
           ├── pid_controller.py
           ├── pure_pursuit.py
           ├── recorder.py
           ├── mis_waypoints.txt
           └── README.md
   ```

---

## Ejecución

### 1. Lanzar el servidor CARLA (Terminal 1)

```bash
cd <CARLA_ROOT>
./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=20
```

> En Windows: `CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=20`

Espera a que el simulador termine de cargar el mapa RaceTrack.

### 2. Ejecutar el controlador (Terminal 2)

```bash
cd <CARLA_ROOT>/PythonClient/ProyectoFinal
python3 main.py
```

### Opciones de línea de comando

```bash
python3 main.py --host localhost --port 2000 --start 1
```

- `--host`  : IP del servidor CARLA (default `localhost`)
- `--port`  : puerto del servidor (default `2000`)
- `--start N` : índice de posición de inicio (dado por el profesor el día de la competencia)

---

## Arquitectura (3 archivos principales)

| Archivo | Rol |
|---------|-----|
| `main.py` | Punto de entrada, conexión a CARLA, bucle de control |
| `pid_controller.py` | Control longitudinal PID (throttle / brake) |
| `pure_pursuit.py` | Control lateral Pure Pursuit (steering) |

Archivos adicionales:
- `mis_waypoints.txt` — ruta grabada del circuito (formato `x, y, velocidad` por línea)
- `recorder.py` — script para volver a grabar la ruta manualmente con W/A/S/D
- `ESTUDIO.md` — guía de estudio detallada para la defensa oral

## Sensores usados (mínimos, para bonificación +3s)

- `forward_speed` → velocidad actual del vehículo
- `transform` (location + rotation) → posición (x, y) y orientación (yaw)

---

## Parámetros ajustables

### En `main.py`
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `PLAYER_START_INDEX` | 1 | Punto de partida en el mapa |
| `MAX_SPEED` | 20.0 m/s | Velocidad máxima (~72 km/h) |
| `SPEED_MULTIPLIER` | 1.0 | Factor de escala sobre velocidad del waypoint |
| `SPEED_PREVIEW` | 40 | Waypoints adelante para anticipar frenado en curvas |
| `FINISH_X` | -184.0 | Coordenada X de la línea de meta |
| `FINISH_Y` | -12.1 | Coordenada Y de la línea de meta |
| `FINISH_RADIUS` | 8.0 m | Radio de detección de meta |

### En `pid_controller.py`
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `KP` | 1.0 | Ganancia proporcional |
| `KI` | 0.2 | Ganancia integral |
| `KD` | 0.01 | Ganancia derivativa |
| `MAX_THROTTLE_DELTA` | 0.1 | Límite de cambio de acelerador por paso (anti-jerk) |

### En `pure_pursuit.py`
| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `LOOKAHEAD_GAIN` | 0.5 | Lookahead adaptativo = GAIN × velocidad + MIN |
| `LOOKAHEAD_MIN` | 5.0 m | Anticipación mínima |
| `LOOKAHEAD_MAX` | 20.0 m | Anticipación máxima |
| `WHEELBASE` | 2.5 m | Batalla del vehículo |

---

## Troubleshooting

### `ImportError: No module named 'carla'`
Significa que la carpeta `ProyectoFinal/` **no está dentro de** `PythonClient/`.
Revisa el paso 2 de la instalación. La estructura debe ser exactamente:
`<CARLA_ROOT>/PythonClient/ProyectoFinal/main.py`.

### `TCPConnectionError: ... Connection refused`
El servidor CARLA no está corriendo o está en otro puerto. Verifica:
- Que ejecutaste `./CarlaUE4.sh ... -carla-server` antes que `main.py`.
- Que el puerto coincide (default `2000`).

### El carro no arranca o va muy lento
- Asegúrate de que `mis_waypoints.txt` está presente en la carpeta.
- Verifica que el `--start` proporcionado por el profesor sea válido (típicamente
  un entero entre 0 y la cantidad de posiciones del mapa).
