import csv
import time
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
    filled_time: float = 0.0
    in_view: bool = False
    not_in_view_time: float = 0.0

    def __post_init__(self):
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, field.type):
                setattr(self, field.name, field.type(value))
    
    def set_filled(self, filled: bool) -> None:
        self.filled = filled
        if self.filled:
            self.filled_time = time.monotonic()

    def set_in_view(self, in_view: bool) -> None:
        self.in_view = in_view
        if not self.in_view:
            self.not_in_view_time = time.monotonic()

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
    
    def get_column(self, alliance: Alliance, column_type: ColumnType) -> List[GridZone]:
        return [self.zones[index] for index in self.column_indices[column_type] if self.zones[index].alliance == alliance]

    def get_row(self, alliance: Alliance, row_type: RowType) -> List[GridZone]:
        return [self.zones[index] for index in self.row_indices[row_type] if self.zones[index].alliance == alliance]

    def add_zone(self, grid_zone: GridZone):
        zone_index = len(self.zones)
        if grid_zone.row not in self.row_indices:
            self.row_indices[grid_zone.row] = []
        self.row_indices[grid_zone.row].append(zone_index)

        if grid_zone.column not in self.column_indices:
            self.column_indices[grid_zone.column] = []
        self.column_indices[grid_zone.column].append(zone_index)

        self.zones.append(grid_zone)
        coords = np.array([[grid_zone.x, grid_zone.y, grid_zone.z]])
        if len(self.coords) == 0:
            self.coords = coords
        else:
            self.coords = np.append(self.coords, coords, axis=0)

    def get_nearest(self, x: float, y: float, z: float) -> Tuple[GridZone, float]:
        point = np.array([x, y, z], dtype=np.float64)
        delta = np.abs(self.coords - point)
        distances = np.linalg.norm(delta, axis=1)
        closest_index = int(np.argmin(distances))
        nearest_zone = self.zones[closest_index]
        return nearest_zone, float(distances[closest_index])
