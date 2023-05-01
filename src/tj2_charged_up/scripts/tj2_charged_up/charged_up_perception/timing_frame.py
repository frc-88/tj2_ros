import copy
import time


class TimingFrame:
    def __init__(self) -> None:
        self.start = 0.0
        self.resize = 0.0
        self.inference = 0.0
        self.detect2d_prep = 0.0
        self.to_3d = 0.0
        self.get_nearest = 0.0
        self.stop = 0.0

    def copy(self):
        return copy.copy(self)

    @classmethod
    def now(cls):
        return time.monotonic()
