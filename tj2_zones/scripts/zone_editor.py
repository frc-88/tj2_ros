from geometry_msgs.msg import Point

from tj2_interfaces.msg import Zone
from tj2_interfaces.msg import ZoneArray

from tj2_tools.zone import ZoneManager

frame_id = "map"

zones = ZoneArray()
zones.header.frame_id = frame_id

zone1 = Zone()
zone1.header.frame_id = frame_id
zone1.name = "red"
zone1.priority = 0.0
zone1.points.append(Point(x=2.0, y=2.0))
zone1.points.append(Point(x=1.0, y=2.0))
zone1.points.append(Point(x=1.0, y=1.0))
zone1.points.append(Point(x=2.0, y=1.0))

zone2 = Zone()
zone2.header.frame_id = frame_id
zone2.name = "blue"
zone2.priority = 0.0
zone2.points.append(Point(x=-2.0, y=2.0))
zone2.points.append(Point(x=-1.0, y=2.0))
zone2.points.append(Point(x=-1.0, y=1.0))
zone2.points.append(Point(x=-2.0, y=1.0))

zones.zones.append(zone1)
zones.zones.append(zone2)

manager = ZoneManager.from_msg(zones)

manager.save_zones("rapid-react-2022.bin")
