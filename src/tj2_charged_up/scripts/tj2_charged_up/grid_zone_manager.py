import csv
import time
import numpy as np
from typing import Dict, List, Tuple
from dataclasses import dataclass
import dataclasses
from enum import Enum

class ScoreDeviceType(Enum):
    """
    Enumeration representing the type of scoring device used in the game.
    - POST: Represents scoring devices for cones.
    - SHELF: Represents scoring devices for cones or cubes.
    """
    POST = 'post'
    SHELF = 'shelf'


class Alliance(Enum):
    """
    Enumeration representing the alliance of a team in the game, which also
    indicates the side the scoring element is on.
    - RED: Represents the negative X-axis side.
    - BLUE: Represents the positive X-axis side.
    """
    RED = 'red'
    BLUE = 'blue'


class ColumnType(Enum):
    """
    Enumeration representing the type of column in the grid zone.
    Each column type implies the height of the scoring element.
    - HIGH: Represents the high height column.
    - MID: Represents the mid height column.
    - LOW: Represents the low height column.
    """
    HIGH = 'high'
    MID = 'mid'
    LOW = 'low'


class ZoneFilledState(Enum):
    """
    Enumeration representing a grid zone state
    - EMPTY: nothing in the grid zone
    - CONE: a cone game object is in the grid zone
    - CUBE: a cube game object is in the grid zone
    """
    EMPTY = 'empty'
    CONE = 'cone'
    CUBE = 'cube'


# Type alias for representing the row number of a grid zone.
RowType = int


@dataclass
class GridZone:
    """
    A data class representing a grid zone, which holds information about the scoring elements
    in the game grid and their properties.

    Attributes:
        x (float): The X-axis position of the grid zone.
        y (float): The Y-axis position of the grid zone.
        z (float): The Z-axis position of the grid zone.
        alliance (Alliance): The alliance the grid zone belongs to.
        type (ScoreDeviceType): The type of scoring device used in the grid zone.
        column (ColumnType): The column type of the grid zone.
        row (RowType): The row number of the grid zone.
        filled (ZoneFilledState): Whether the grid zone is filled with a game object.
        filled_time (float): The monotonic time when the grid zone was last filled.
        in_view (bool): Whether the grid zone is in the camera's view.
        not_in_view_time (float): The monotonic time when the grid zone was last not in view.
    """
    x: float
    y: float
    z: float
    alliance: Alliance
    type: ScoreDeviceType
    column: ColumnType
    row: RowType
    filled: ZoneFilledState = ZoneFilledState.EMPTY
    filled_time: float = 0.0
    in_view: bool = False
    not_in_view_time: float = 0.0

    def __post_init__(self):
        """
        After initializing the GridZone object, this method checks and converts
        the types of the attributes to their correct types if they are supplied
        as strings. This is useful when parsing data from a CSV file.

        This method is automatically called by the dataclass decorator after
        the object is initialized.
        """
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, field.type):
                setattr(self, field.name, field.type(value))

    def set_filled(self, filled: ZoneFilledState) -> None:
        """
        Set the filled state of the grid zone and update the filled_time if the grid zone is filled.

        Args:
            filled (bool): Whether the grid zone is filled with a game object.
        """
        self.filled = filled
        if self.filled:
            self.filled_time = time.monotonic()

    def set_in_view(self, in_view: bool) -> None:
        """
        Set the in_view state of the grid zone and update the not_in_view_time if the grid zone is not in view.

        Args:
            in_view (bool): Whether the grid zone is in the camera's view.
        """
        self.in_view = in_view
        if not self.in_view:
            self.not_in_view_time = time.monotonic()


class GridZoneManager:
    """
    A class to manage and organize grid zones, making it easier to perform
    operations on zones, such as finding the nearest zone or accessing
    zones by their attributes.
    """
    
    def __init__(self) -> None:
        self.zones: List[GridZone] = []

        # A numpy array containing the (x, y, z) coordinates of each zone for faster access and computation.
        self.coords: np.ndarray = np.array([], dtype=np.float64)

        # A dictionary mapping ColumnType to a list of indices in `zones` for faster access to zones by their column type.
        self.column_indices: Dict[ColumnType, List[int]] = {}

        # A dictionary mapping RowType (int) to a list of indices in `zones` for faster access to zones by their row number.
        self.row_indices: Dict[RowType, List[int]] = {}
    
    @classmethod
    def from_file(cls, path) -> "GridZoneManager":
        """
        Initialize a GridZoneManager instance from a CSV file.

        Args:
            path (str): The path to the CSV file containing grid zone data.

        Returns:
            GridZoneManager: A new instance of the GridZoneManager class
            populated with grid zones from the CSV file.
        """
        # Create a new instance of the GridZoneManager class
        self = cls()

        # Open the CSV file
        with open(path) as file:
            # Create a DictReader to read rows from the CSV file as dictionaries
            reader = csv.DictReader(file)

            # Iterate through each row in the CSV file
            for row in reader:
                # Create a GridZone instance from the row data, using the keyword
                # arguments syntax to pass the row dictionary to the GridZone constructor
                zone = GridZone(**row)  # type: ignore

                # Add the new GridZone instance to the GridZoneManager
                self.add_zone(zone)

        # Return the populated GridZoneManager instance
        return self

    def get_column(self, alliance: Alliance, column_type: ColumnType) -> List[GridZone]:
        """
        Get a list of GridZone instances from the specified alliance and column type.

        Args:
            alliance (Alliance): The alliance (red or blue) to filter the grid zones.
            column_type (ColumnType): The column type (high, mid, or low) to filter the grid zones.

        Returns:
            List[GridZone]: A list of GridZone instances that match the specified alliance and column type.
        """
        return [self.zones[index] for index in self.column_indices[column_type] if self.zones[index].alliance == alliance]

    def get_row(self, alliance: Alliance, row_type: RowType) -> List[GridZone]:
        """
        Get a list of GridZone instances from the specified alliance and row type.

        Args:
            alliance (Alliance): The alliance (red or blue) to filter the grid zones.
            row_type (RowType): The row type (integer) to filter the grid zones.

        Returns:
            List[GridZone]: A list of GridZone instances that match the specified alliance and row type.
        """
        return [self.zones[index] for index in self.row_indices[row_type] if self.zones[index].alliance == alliance]

    def add_zone(self, grid_zone: GridZone):
        """
        Add a GridZone instance to the GridZoneManager and update the internal data structures.

        Args:
            grid_zone (GridZone): The GridZone instance to add to the manager.
        """
        # Get the index where the new grid zone will be added
        zone_index = len(self.zones)

        # Update the row indices dictionary
        if grid_zone.row not in self.row_indices:
            self.row_indices[grid_zone.row] = []
        self.row_indices[grid_zone.row].append(zone_index)

        # Update the column indices dictionary
        if grid_zone.column not in self.column_indices:
            self.column_indices[grid_zone.column] = []
        self.column_indices[grid_zone.column].append(zone_index)

        # Append the new grid zone to the list of zones
        self.zones.append(grid_zone)

        # Update the numpy array of coordinates
        coords = np.array([[grid_zone.x, grid_zone.y, grid_zone.z]])
        if len(self.coords) == 0:
            self.coords = coords
        else:
            self.coords = np.append(self.coords, coords, axis=0)

    def get_nearest(self, x: float, y: float, z: float) -> Tuple[GridZone, float]:
        """
        Get the nearest GridZone instance and its distance to the specified point.

        Args:
            x (float): The x-coordinate of the point.
            y (float): The y-coordinate of the point.
            z (float): The z-coordinate of the point.

        Returns:
            Tuple[GridZone, float]: A tuple containing the nearest GridZone instance and its distance to the point.
        """
        # Create a numpy array for the input point
        point = np.array([x, y, z], dtype=np.float64)

        # Calculate the absolute differences between the point and each grid zone's coordinates
        delta = np.abs(self.coords - point)

        # Compute the Euclidean distances between the point and each grid zone
        distances = np.linalg.norm(delta, axis=1)

        # Find the index of the closest grid zone
        closest_index = int(np.argmin(distances))

        # Get the nearest grid zone and its distance
        nearest_zone = self.zones[closest_index]
        return nearest_zone, float(distances[closest_index])
