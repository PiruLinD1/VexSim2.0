import math
from dataclasses import dataclass


def _coerce_xy(value):
    if isinstance(value, Vec2):
        return value.x, value.y
    if isinstance(value, (tuple, list)) and len(value) >= 2:
        return float(value[0]), float(value[1])
    raise TypeError(f"Cannot convert {type(value)!r} to Vec2")


@dataclass(frozen=True)
class Vec2:
    x: float = 0.0
    y: float = 0.0

    def __iter__(self):
        yield self.x
        yield self.y

    def __add__(self, other):
        ox, oy = _coerce_xy(other)
        return Vec2(self.x + ox, self.y + oy)

    def __sub__(self, other):
        ox, oy = _coerce_xy(other)
        return Vec2(self.x - ox, self.y - oy)

    def __mul__(self, scalar: float):
        return Vec2(self.x * float(scalar), self.y * float(scalar))

    def __rmul__(self, scalar: float):
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float):
        s = float(scalar)
        if abs(s) < 1e-12:
            raise ZeroDivisionError("Vec2 division by zero")
        return Vec2(self.x / s, self.y / s)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __iadd__(self, other):
        return self + other

    def __isub__(self, other):
        return self - other

    def __imul__(self, scalar: float):
        return self * scalar

    def __itruediv__(self, scalar: float):
        return self / scalar

    @property
    def length(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def length_sq(self) -> float:
        return (self.x * self.x) + (self.y * self.y)

    @property
    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def dot(self, other) -> float:
        ox, oy = _coerce_xy(other)
        return (self.x * ox) + (self.y * oy)

    def cross(self, other) -> float:
        ox, oy = _coerce_xy(other)
        return (self.x * oy) - (self.y * ox)

    def normalized(self):
        mag = self.length
        if mag < 1e-9:
            return Vec2(0.0, 0.0)
        return self / mag

    def perpendicular(self):
        return Vec2(-self.y, self.x)

    def rotated(self, angle_rad: float):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return Vec2((self.x * c) - (self.y * s), (self.x * s) + (self.y * c))


def as_vec2(value) -> Vec2:
    if isinstance(value, Vec2):
        return value
    x, y = _coerce_xy(value)
    return Vec2(x, y)
