"""X-roller mecanum kinematics. Wheel order: FL, FR, RL, RR; SI units."""
from dataclasses import dataclass
from math import isfinite


def _finite(values, count):
    values = tuple(values)
    if len(values) != count or not all(isfinite(v) for v in values):
        raise ValueError(f'Expected {count} finite values')
    return values


@dataclass(frozen=True)
class Geometry:
    radius: float
    half_length: float
    half_width: float

    def __post_init__(self):
        values = _finite((self.radius, self.half_length, self.half_width), 3)
        if min(values) <= 0:
            raise ValueError('Geometry dimensions must be positive')

    @property
    def lever(self):
        return self.half_length + self.half_width


def inverse(vx, vy, wz, geometry):
    vx, vy, wz = _finite((vx, vy, wz), 3)
    k, r = geometry.lever, geometry.radius
    return ((vx-vy-k*wz)/r, (vx+vy+k*wz)/r,
            (vx+vy-k*wz)/r, (vx-vy+k*wz)/r)


def forward(wheels, geometry):
    fl, fr, rl, rr = _finite(wheels, 4)
    r, k = geometry.radius, geometry.lever
    return (r*(fl+fr+rl+rr)/4, r*(-fl+fr+rl-rr)/4,
            r*(-fl+fr-rl+rr)/(4*k))


def limit_wheels(wheels, maximum):
    wheels = _finite(wheels, 4)
    if not isfinite(maximum) or maximum <= 0:
        raise ValueError('Maximum wheel velocity must be finite and positive')
    scale = max(1.0, max(map(abs, wheels))/maximum)
    return tuple(v/scale for v in wheels)
