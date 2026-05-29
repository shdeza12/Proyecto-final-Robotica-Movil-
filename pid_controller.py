"""
Control longitudinal PID para seguimiento de velocidad.
Sensores usados: get_velocity() (forward_speed)
"""
import numpy as np


# ── Ganancias ajustables ───────────────────────────────────────────────────
KP = 1.0    # Proporcional: respuesta al error actual de velocidad
KI = 0.2    # Integral:     corrige error acumulado en el tiempo
KD = 0.01   # Derivativo:   amortigua cambios bruscos de velocidad
MAX_THROTTLE_DELTA = 0.1  # Límite de cambio por paso (anti-jerk)
INTEGRAL_MAX       = 5.0  # Tope del término integral (anti-windup)
BRAKE_GAIN         = 1.5  # Ganancia de frenado: corrige más fuerte al desacelerar
# ──────────────────────────────────────────────────────────────────────────


class PIDController:
    def __init__(self, kp=KP, ki=KI, kd=KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral  = 0.0
        self._prev_error    = 0.0
        self._prev_throttle = 0.0

    def compute(self, v_desired, v_current, dt):
        """Calcula throttle y brake a partir del error de velocidad.

        Args:
            v_desired: velocidad deseada (m/s)
            v_current: velocidad actual   (m/s)
            dt:        paso de tiempo     (s)

        Returns:
            throttle [0, 1], brake [0, 1]
        """
        if dt <= 0.0:
            return self._prev_throttle, 0.0

        error = v_desired - v_current
        self._integral += error * dt
        # Anti-windup: el integral no se dispara en rectas largas (si no, al
        # llegar a la curva pelearía contra el freno y entraría pasado).
        self._integral = float(np.clip(self._integral, -INTEGRAL_MAX, INTEGRAL_MAX))
        derivative = (error - self._prev_error) / dt

        acc = self.kp * error + self.ki * self._integral + self.kd * derivative

        if acc >= 0.0:
            throttle = float(np.tanh(acc))
            # Límite de tasa para evitar jerk (penalización si Δa > 5 m/s³)
            throttle = min(throttle, self._prev_throttle + MAX_THROTTLE_DELTA)
            brake = 0.0
        else:
            throttle = 0.0
            # Al frenar limpiamos el integral acumulado y aplicamos más fuerza,
            # para bajar a la velocidad de curva a tiempo.
            self._integral = 0.0
            brake = float(np.clip(np.tanh(-acc * BRAKE_GAIN), 0.0, 1.0))

        self._prev_error    = error
        self._prev_throttle = throttle
        return throttle, brake

    def reset(self):
        self._integral      = 0.0
        self._prev_error    = 0.0
        self._prev_throttle = 0.0