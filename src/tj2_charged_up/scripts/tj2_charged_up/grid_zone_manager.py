import csv
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
    
    @classmethod
    def from_file(cls, path) -> "GridZoneManager":
        self = cls()
        with open(path) as file:
            reader = csv.DictReader(file)
            for row in reader:
                zone = GridZone(**row)  # type: ignore
                self.zones.append(zone)
        return self