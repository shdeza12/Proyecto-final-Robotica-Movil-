"""
Control lateral Pure Pursuit para seguimiento de waypoints.
Sensores usados: get_transform() (posición x, y y orientación yaw)
"""
import numpy as np


# ── Parámetros ajustables ──────────────────────────────────────────────────
LOOKAHEAD_GAIN = 0.5   # lookahead adaptativo = GAIN * velocidad + MIN
LOOKAHEAD_MIN  = 5.0   # distancia mínima de anticipación (m)
LOOKAHEAD_MAX  = 20.0  # distancia máxima de anticipación (m)
WHEELBASE      = 2.5   # batalla del vehículo en metros
# Conversión de radianes a rango [-1, 1] para CARLA (dirección max = 70°)
RAD_TO_STEER   = 180.0 / 70.0 / np.pi
# ──────────────────────────────────────────────────────────────────────────


class PurePursuitController:
    def __init__(self, gain=LOOKAHEAD_GAIN, min_la=LOOKAHEAD_MIN,
                 max_la=LOOKAHEAD_MAX, wheelbase=WHEELBASE):
        self.gain      = gain
        self.min_la    = min_la
        self.max_la    = max_la
        self.wheelbase = wheelbase

    def compute(self, x, y, yaw, speed, waypoints):
        """Calcula el ángulo de dirección normalizado en [-1, 1].

        Args:
            x, y:      posición actual del vehículo (m)
            yaw:       orientación del vehículo (rad)
            speed:     velocidad actual (m/s)
            waypoints: lista de [wx, wy, v] a seguir

        Returns:
            steer normalizado [-1, 1]
        """
        lookahead = self._adaptive_lookahead(speed)
        target    = self._find_lookahead_point(x, y, waypoints, lookahead)

        dx = target[0] - x
        dy = target[1] - y
        dist = np.hypot(dx, dy)

        if dist < 0.01:
            return 0.0

        # Ángulo entre la dirección del vehículo y el punto objetivo
        alpha = np.arctan2(dy, dx) - yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # normalizar a [-π, π]

        # Fórmula Pure Pursuit: δ = arctan(2·L·sin(α) / d)
        steer_rad = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), dist)
        steer_norm = float(np.clip(steer_rad * RAD_TO_STEER, -1.0, 1.0))
        return steer_norm

    def _adaptive_lookahead(self, speed):
        """Lookahead crece con la velocidad para mayor estabilidad."""
        la = self.gain * abs(speed) + self.min_la
        return float(np.clip(la, self.min_la, self.max_la))

    def _find_lookahead_point(self, x, y, waypoints, lookahead):
        """Devuelve el primer waypoint a >= lookahead metros del vehículo."""
        pos = np.array([x, y])
        wp_arr = np.array([[w[0], w[1]] for w in waypoints])
        dists  = np.linalg.norm(wp_arr - pos, axis=1)

        for i, d in enumerate(dists):
            if d >= lookahead:
                return waypoints[i]

        # Si no hay ninguno lo suficientemente lejos, usar el último
        return waypoints[-1]