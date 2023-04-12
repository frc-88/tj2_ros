import csv
from numba import njit
import numpy as np
from typing import Dict, List, Tuple
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


RowType = int


@dataclass
class GridZone:
    x: float
    y: float
    z: float
    alliance: Alliance
    type: ScoreDeviceType
    column: ColumnType
    row: RowType
    filled: bool = False

    def __post_init__(self):
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, field.type):
                setattr(self, field.name, field.type(value))

@njit
def jit_get_nearest(coords: np.ndarray, x: float, y: float, z: float) -> Tuple[int, float]:
    point = np.array([x, y, z], dtype=np.float64)
    delta = np.abs(coords - point)
    # https://github.com/numba/numba/issues/2181
    # distance = np.linalg.norm(delta, axis=1)
    distance = np.sqrt(delta[:, 0] * delta[:, 0] + delta[:, 1] * delta[:, 1] + delta[:, 2] * delta[:, 2])
    closest_index = int(np.argmin(distance))
    return closest_index, float(distance)

def np_get_nearest(coords: np.ndarray, x: float, y: float, z: float) -> Tuple[int, float]:
    point = np.array([x, y, z], dtype=np.float64)
    delta = np.abs(coords - point)
    distance = np.linalg.norm(delta, axis=1)
    closest_index = int(np.argmin(distance))
    return closest_index, float(distance)

class GridZoneManager:
    def __init__(self) -> None:
        self.zones: List[GridZone] = []
        self.coords: np.ndarray = np.array([], dtype=np.float64)
        self.column_indices: Dict[ColumnType, List[int]] = {}
        self.row_indices: Dict[RowType, List[int]] = {}
    
    @classmethod
    def from_file(cls, path) -> "GridZoneManager":
        self = cls()
        with open(path) as file:
            reader = csv.DictReader(file)
            for row in reader:
                zone = GridZone(**row)  # type: ignore
                self.add_zone(zone)
        return self
    
    def get_column(self, column_type: ColumnType) -> List[GridZone]:
        return [self.zones[index] for index in self.column_indices[column_type]]

    def get_row(self, row_type: RowType) -> List[GridZone]:
        return [self.zones[index] for index in self.row_indices[row_type]]

    def add_zone(self, grid_zone: GridZone):
        if grid_zone.row not in self.row_indices:
            self.row_indices[grid_zone.row] = []
        self.row_indices[grid_zone.row].append(len(self.zones))

        if grid_zone.column not in self.column_indices:
            self.column_indices[grid_zone.column] = []
        self.column_indices[grid_zone.column].append(len(self.zones))

        self.zones.append(grid_zone)
        coords = np.array([[grid_zone.x, grid_zone.y, grid_zone.z]])
        if len(self.coords) == 0:
            self.coords = coords
        else:
            self.coords = np.append(self.coords, coords, axis=0)

    def get_nearest(self, x: float, y: float, z: float) -> Tuple[GridZone, float]:
        index, distance = jit_get_nearest(self.coords, x, y, z)
        return self.zones[index], distance
