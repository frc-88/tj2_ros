import csv
import numpy as np
from typing import List
from dataclasses import dataclass
import dataclasses
from enum import Enum

class ScoreDeviceType(Enum):
    POST = 'post'
    SHELF = 'shelf'

class Alliance(Enum):
    RED = 'red'
    BLUE = 'blue'


class ColumnType(Enum):
    HIGH = 'high'
    MID = 'mid'
    LOW = 'low'


@dataclass
class GridZone:
    x: float
    y: float
    z: float
    alliance: Alliance
    type: ScoreDeviceType
    column: ColumnType
    row: int

    def __post_init__(self):
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, field.type):
                setattr(self, field.name, field.type(value))


class GridZoneManager:
    def __init__(self) -> None:
        self.zones: List[GridZone] = []
        self.coords: np.ndarray = np.array([], dtype=np.float64)
    
    @classmethod
    def from_file(cls, path) -> "GridZoneManager":
        self = cls()
        with open(path) as file:
            reader = csv.DictReader(file)
            for row in reader:
                zone = GridZone(**row)  # type: ignore
                self.add_zone(zone)
        return self

    def add_zone(self, grid_zone: GridZone):
        self.zones.append(grid_zone)
        coords = np.array([[grid_zone.x, grid_zone.y, grid_zone.z]])
        if len(self.coords) == 0:
            self.coords = coords
        else:
            self.coords = np.append(self.coords, coords, axis=0)

    def get_nearest(self, x: float, y: float, z: float) -> GridZone:
        point = np.array([x, y, z], dtype=np.float64)
        delta = np.abs(self.coords - point)
        distance = np.linalg.norm(delta, axis=1)
        closest_index = np.argmin(distance)
        return self.zones[closest_index]
