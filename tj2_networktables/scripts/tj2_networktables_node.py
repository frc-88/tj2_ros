#!/usr/bin/python3
import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo

from networktables import NetworkTables
from tj2_limelight.msg import LimelightTarget
from tj2_limelight.msg import LimelightTargetArray


class TJ2NetworkTables:
    def __init__(self):
        self.node_name = "tj2_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        
        NetworkTables.initialize(server=self.nt_host)
        self.nt = NetworkTables.getTable("")

        self.remote_linear_units_conversion = rospy.get_param("~remote_linear_units_conversion", 1.0)
        self.remote_angular_units_conversion = rospy.get_param("~remote_angular_units_conversion", 1.0)
        self.limelight_target_frame_id = rospy.get_param("~limelight_target_frame_id", "limelight")
        
        self.match_time_pub = rospy.Publisher("match_time", Float64, queue_size=5)
        self.is_autonomous_pub = rospy.Publisher("is_autonomous", Bool, queue_size=5)
        self.limelight_target_pub = rospy.Publisher("/limelight/targets", LimelightTargetArray, queue_size=5)
        self.limelight_led_mode_sub = rospy.Subscriber("/limelight/led_mode", Bool, self.limelight_led_mode_callback, queue_size=5)
        self.limelight_cam_mode_sub = rospy.Subscriber("/limelight/cam_mode", Bool, self.limelight_cam_mode_callback, queue_size=5)
        self.limelight_info_sub = rospy.Subscriber("limelight/camera_info", CameraInfo, self.limelight_info_callback, queue_size=5)

        self.driver_station_table_key = "swerveLibrary/ROS/DriverStation"
        self.limelight_table_key = "limelight"

        self.limelight_led_mode_entry = self.nt.getEntry(self.limelight_table_key + "/ledMode")
        self.limelight_cam_mode_entry = self.nt.getEntry(self.limelight_table_key + "/camMode")

        self.fms_flag_ignored = True

        self.remote_start_time = 0.0
        self.local_start_time = 0.0
        self.prev_timestamp = 0.0

        self.limelight_camera_info = CameraInfo()
        
        self.clock_rate = rospy.Rate(20.0)  # networktable servers update at 10 Hz

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        self.set_limelight_led_mode(True)  # turn limelight off on start up
        
        while not rospy.is_shutdown():
            if self.remote_start_time == 0.0:
                self.init_remote_time()
            self.clock_rate.sleep()
            self.publish_driver_station()
            self.publish_limelight_target()

    def limelight_info_callback(self, msg):
        self.limelight_camera_info = msg

    def publish_limelight_target(self):
        msg = LimelightTargetArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.limelight_target_frame_id
        has_targets = self.nt.getEntry(self.limelight_table_key + "/tv").getDouble(0.0) == 1.0
        if has_targets:
            for index in range(self.num_limelight_targets):  # TODO: find num_limelight_targets
                target_msg = LimelightTarget()
                tx = self.nt.getEntry(self.limelight_table_key + "/tx%s" % index).getDouble(0.0)
                ty = self.nt.getEntry(self.limelight_table_key + "/ty%s" % index).getDouble(0.0)
                target_msg.thor = int(self.nt.getEntry(self.limelight_table_key + "/thor%s" % index).getDouble(0.0))
                target_msg.tvert = int(self.nt.getEntry(self.limelight_table_key + "/tvert%s" % index).getDouble(0.0))

                width = self.limelight_camera_info.width
                height = self.limelight_camera_info.height

                target_msg.tx = int((tx + 1.0) / 2.0 * width)
                target_msg.ty = int((-ty + 1.0) / 2.0 * height)

        self.limelight_target_pub.publish(msg)

    def publish_driver_station(self):
        is_fms_attached = self.nt.getEntry(self.driver_station_table_key + "/isFMSAttached").getBoolean(False)
        is_autonomous = self.nt.getEntry(self.driver_station_table_key + "/isAutonomous").getBoolean(True)

        if self.fms_flag_ignored or is_fms_attached:
            match_time = self.nt.getEntry(self.driver_station_table_key + "/getMatchTime").getDouble(-1.0)
            self.match_time_pub.publish(match_time)
            self.is_autonomous_pub.publish(is_autonomous)
        else:
            self.match_time_pub.publish(-1.0)


    def limelight_led_mode_callback(self, msg):
        self.set_limelight_led_mode(msg.data)
    
    def get_limelight_led_mode(self):
        mode = self.limelight_led_mode_entry.getDouble(1.0)
        return mode == 1.0

    def set_limelight_led_mode(self, flag):
        mode = 1.0 if flag else 0.0
        rospy.loginfo("Setting limelight led mode to %s" % mode)
        self.limelight_led_mode_entry.setDouble(mode)


    def limelight_cam_mode_callback(self, msg):
        self.set_limelight_cam_mode(msg.data)

    def get_limelight_cam_mode(self):
        mode = self.limelight_cam_mode_entry.getDouble(1.0)
        return mode == 1.0

    def set_limelight_cam_mode(self, flag):
        mode = 1.0 if flag else 0.0
        rospy.loginfo("Setting limelight cam mode to %s" % mode)
        self.limelight_cam_mode_entry.setDouble(mode)


    def get_remote_time(self):
        """
        Gets RoboRIO's timestamp based on the networktables entry in seconds
        """
        return self.nt.getEntry("timestamp").getDouble(0.0) * 1E-6
    
    def get_local_time(self):
        return rospy.Time.now().to_sec()

    def check_time_offsets(self, remote_timestamp):
        if remote_timestamp < self.prev_timestamp:  # remote has restarted
            self.remote_start_time = remote_timestamp
            self.local_start_time = self.get_local_time()
            rospy.loginfo("Remote clock jumped back. Resetting offset")
        self.prev_timestamp = remote_timestamp

    def get_local_time_as_remote(self):
        local_timestamp = self.get_local_time()
        remote_timestamp = self.get_remote_time()
        self.check_time_offsets(remote_timestamp)
        remote_time = local_timestamp - self.local_start_time + self.remote_start_time
        remote_time *= 1E6
        return remote_time

    def wait_for_time(self):
        rospy.loginfo("Waiting for remote time")
        while self.remote_start_time == 0.0:
            self.init_remote_time()
            self.clock_rate.sleep()
            if rospy.is_shutdown():
                return
        rospy.loginfo("Remote time found")
    
    def init_remote_time(self):
        self.remote_start_time = self.get_remote_time()
        self.local_start_time = self.get_local_time()
    
    def get_remote_time_as_local(self):
        """
        Gets the RoboRIO's time relative to ROS time epoch
        """
        remote_timestamp = self.get_remote_time()
        self.check_time_offsets(remote_timestamp)
        return remote_timestamp - self.remote_start_time + self.local_start_time


if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
