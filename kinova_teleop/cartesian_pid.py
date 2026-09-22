"""Bounded Cartesian translation PID; gains are not hardware-validated tuning."""

from __future__ import annotations

import math

import numpy as np


def _bounded(vector: np.ndarray, limit: float) -> np.ndarray:
    norm = math.hypot(*vector)
    return vector if norm <= limit else vector * (limit / norm)


class CartesianPID:
    """Position-to-velocity PID in metres and seconds.

    The caller serializes update/reset (KortexBackend uses its state lock).
    Derivative is on measurement, low-pass filtered with a 40 ms time constant.
    Integral contribution is norm-clamped, with conditional anti-windup.
    First samples and gaps over 200 ms discard timing history; a gap does not
    accumulate stale error. Non-increasing/non-finite timestamps are rejected.
    """

    def __init__(self, kp=3.3, ki=.15, kd=.02, max_speed=.05):
        values = np.asarray([kp, ki, kd, max_speed], dtype=float)
        if not np.isfinite(values).all() or np.any(values[:3] < 0):
            raise ValueError('PID gains must be finite and non-negative')
        if not 0 < max_speed <= .05:
            raise ValueError('PID max_speed must be in (0, 0.05] m/s')
        self.kp, self.ki, self.kd, self.max_speed = map(float, values)
        self.reset()

    def reset(self) -> None:
        self._integral = np.zeros(3)
        self._derivative = np.zeros(3)
        self._measured = None
        self._now = None

    def update(self, target, measured, now) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        measured = np.asarray(measured, dtype=float)
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (target, measured)):
            raise ValueError('PID positions must contain three finite values')
        now = float(now)
        if not math.isfinite(now) or (self._now is not None and now <= self._now):
            raise ValueError('PID timestamp must be finite and strictly increasing')
        dt = 0. if self._now is None else now - self._now
        integral = self._integral.copy()
        derivative = np.zeros(3)
        error = target - measured
        if 0 < dt <= .2:
            alpha = dt / (.04 + dt)
            derivative = (1 - alpha) * self._derivative + alpha * (measured - self._measured) / dt
            candidate = _bounded(integral + self.ki * error * dt, self.max_speed)
            proposed = self.kp * error + candidate - self.kd * derivative
            # Integrate only when unsaturated or driving back out of saturation.
            if math.hypot(*proposed) <= self.max_speed or np.dot(proposed, error) < 0:
                integral = candidate
        elif dt > .2:
            integral = np.zeros(3)
        velocity = self.kp * error + integral - self.kd * derivative
        if not np.isfinite(velocity).all():
            raise ValueError('PID arithmetic must remain finite')
        self._integral = integral
        self._derivative = derivative
        self._measured = measured.copy()
        self._now = now
        return _bounded(velocity, self.max_speed)
