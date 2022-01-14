
OBJECT_NAMES = [
    "BACKGROUND",
    "power_cell"
]


class State:
    def __init__(self):
        self.type = ""
        self.stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.t = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vt = 0.0

    @classmethod
    def from_state(cls, other):
        if not isinstance(other, cls):
            raise ValueError("%s is not of type %s" % (other, cls))
        self = cls()
        self.type = other.type
        self.stamp = other.stamp
        self.x = other.x
        self.y = other.y
        self.z = other.z
        self.t = other.t
        self.vx = other.vx
        self.vy = other.vy
        self.vz = other.vz
        self.vt = other.vt
        return self

    def __str__(self):
        return f"<{self.type}>(" \
               f"x={self.x:0.4f}, y={self.y:0.4f}, z={self.z:0.4f}, t={self.t:0.4f}, " \
               f"vx={self.vx:0.4f}, vy={self.vy:0.4f}, vz={self.vz:0.4f}, vt={self.vt:0.4f}) @ {self.stamp}"

