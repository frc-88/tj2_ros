import os
import math
from typing import List, Optional
import rospy

import shapely.geometry
from shapely.geometry import Polygon
from shapely.ops import nearest_points

import geometry_msgs.msg

from tj2_interfaces.msg import Zone
from tj2_interfaces.msg import ZoneArray
from tj2_interfaces.msg import ZoneInfo
from tj2_interfaces.msg import ZoneInfoArray

from tj2_tools.robot_state import Pose2d


class ZoneManager:
    def __init__(self) -> None:
        self._zones = ZoneArray()
        self._name_mapping = {}

    def load_zones(self, path: str) -> ZoneArray:
        zones = ZoneArray()
        load_dir = os.path.dirname(path)
        if os.path.isdir(load_dir) or len(load_dir) == 0:
            if not os.path.isfile(path):
                rospy.loginfo(f"Zones path doesn't exist: {path}. Creating file.")
                self.save_zones(path, zones)
            else:
                with open(path, 'rb') as file:
                    zones.deserialize(file.read())
        elif len(path) == 0:
            rospy.loginfo("No zone path specified. Not loading zones.")
        else:
            raise FileNotFoundError(f"Zones directory doesn't exist: {load_dir}")
        return zones

    def save_zones(self, path: str, zones: Optional[ZoneArray] = None) -> None:
        if zones is None:
            zones = self._zones
        save_dir = os.path.dirname(path)
        if os.path.isdir(save_dir) or len(save_dir) == 0:
            with open(path, 'wb') as file:
                zones.serialize(file)
        elif len(path) == 0:
            rospy.loginfo("No zone path specified. Not saving zones.")
        else:
            raise FileNotFoundError(f"Zones directory doesn't exist: {save_dir}")

    def add_polygon(self, name: str, priority: float, polygon: Polygon) -> None:
        zone = self.to_zone(name, priority, self._zones.header.frame_id, polygon)
        self.add_zone(zone)

    def add_zone(self, zone: Zone) -> None:
        if len(self._zones.header.frame_id) == 0:
            self._zones.header.frame_id = zone.header.frame_id
        if zone.header.frame_id != self._zones.header.frame_id:
            rospy.logwarn(f"Zone frame id doesn't match this manager's frame. Overwriting add zone's frame id: {zone}")
            zone.header.frame_id = self._zones.header.frame_id
        self._name_mapping[zone.name] = len(self._zones.zones)
        self._zones.zones.append(zone)

    def remove_zone_named(self, name: str) -> None:
        index = self._name_mapping[name]
        del self._name_mapping[name]
        self._zones.zones.pop(index)

    def remove_zone(self, index: int) -> None:
        zone = self._zones.zones.pop(index)
        if zone.name in self._name_mapping:
            del self._name_mapping[zone.name]

    def update_zones(self, zones: ZoneArray) -> None:
        for zone in zones.zones:
            zone.header = zones.header
            self.add_zone(zone)

    def get_zone_named(self, name: str) -> Zone:
        return self.get_zone(self.get_zone_index(name))

    def get_zone_index(self, name: str) -> int:
        return self._name_mapping[name]

    def get_zone(self, index: int) -> None:
        zone = self._zones.zones[index]
        if zone.header.frame_id != self._zones.header.frame_id:
            rospy.logwarn(f"Zone frame id doesn't match this manager's frame. Overwriting zone's frame id: {zone}")
            zone.header.frame_id = self._zones.header.frame_id
        return zone
    
    def __len__(self) -> int:
        return len(self._zones.zones)

    def is_inside(self, index: int, robot_pose: Pose2d) -> bool:
        robot_point = shapely.geometry.Point(robot_pose.x, robot_pose.y)
        polygon = self.to_polygon(self.get_zone(index))
        return polygon.contains(robot_point)

    def get_nearest_point(self, index: int, robot_pose: Pose2d) -> Pose2d:
        robot_point = shapely.geometry.Point(robot_pose.x, robot_pose.y)
        polygon = self.to_polygon(self.get_zone(index))
        pt1, pt2 = nearest_points(polygon, robot_point)
        return Pose2d(
            pt1.coords.xy[0][0],
            pt1.coords.xy[1][0],
            0.0
        )
    
    def get_distance(self, index: int, robot_pose: Pose2d) -> float:
        robot_point = shapely.geometry.Point(robot_pose.x, robot_pose.y)
        polygon = self.to_polygon(self.get_zone(index))
        return polygon.distance(robot_point)

    def to_zone_info(self, robot_pose: Pose2d) -> ZoneInfoArray:
        polygons = self.to_polygons()
        robot_point = shapely.geometry.Point(robot_pose.x, robot_pose.y)
        info_array = ZoneInfoArray()
        info_array.header = self._zones.header
        info_array.is_valid = True
        for index, polygon in enumerate(polygons):
            info = ZoneInfo()
            info.zone = self.get_zone(index)
            info.is_inside = polygon.contains(robot_point)
            pt1, pt2 = nearest_points(polygon, robot_point)
            info.nearest_point.x = pt1.coords.xy[0][0]
            info.nearest_point.y = pt1.coords.xy[1][0]
            delta_x = info.nearest_point.x - robot_pose.x
            delta_y = info.nearest_point.y - robot_pose.y
            info.distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
            info_array.zones.append(info)
        return info_array

    def to_polygons(self) -> List[Polygon]:
        return [self.to_polygon(zone) for zone in self._zones.zones]

    def to_msg(self) -> ZoneArray:
        return self._zones

    @classmethod
    def from_msg(cls, zone_array: ZoneArray) -> "ZoneManager":
        self = cls()
        self.update_zones(zone_array)
        return self
    
    @classmethod
    def from_file(cls, path: str) -> "ZoneManager":
        self = cls()
        self.update_zones(self.load_zones(path))
        return self

    @classmethod
    def to_zone(cls, name: str, priority: float, frame_id: str, polygon: Polygon) -> Zone:
        zone = Zone()
        zone.name = name
        zone.priority = priority
        zone.header.frame_id = frame_id
        
        xy = polygon.exterior.coords.xy
        for index in range(len(xy[0])):
            x = xy[0][index]
            y = xy[1][index]
            pt = geometry_msgs.msg.Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            zone.points.append(pt)
        return zone

    @classmethod
    def to_polygon(cls, zone: Zone) -> Polygon:
        return Polygon([(point.x, point.y) for point in zone.points])
