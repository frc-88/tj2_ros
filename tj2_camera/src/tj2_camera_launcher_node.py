#!/usr/bin/env python3
import rospy
import rospkg
import rostopic

from std_srvs.srv import Trigger, TriggerResponse

from sensor_msgs.msg import Image

from tj2_tools.launch_manager import LaunchManager


class TJ2CameraLauncher:
    def __init__(self):
        self.package_name = "tj2_camera"
        self.node_name = "tj2_camera_launcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.package_name)
        self.default_launches_dir = self.package_dir + "/launch"

        self.on_start = rospy.get_param("~on_start", False)
        self.camera_ns = rospy.get_param("~camera_ns", "/camera")
        self.service_ns_name = rospy.get_param("~service_ns_name", "/tj2")
        self.camera_launch_path = rospy.get_param("~camera_launch", self.default_launches_dir + "/tj2_camera.launch")
        self.record_launch_path = rospy.get_param("~record_launch", self.default_launches_dir + "/record_camera.launch")
        self.set_params_launch_path = rospy.get_param("~set_params_launch", self.default_launches_dir + "/set_parameters.launch")
        self.expected_camera_rate = rospy.get_param(self.camera_ns + "/realsense2_camera/color_fps", 30.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.min_rate_threshold = max(0.0, self.expected_camera_rate - self.min_rate_offset)
        self.max_rate_threshold = max(0.0, self.expected_camera_rate + self.min_rate_offset)

        self.camera_launcher = LaunchManager(self.camera_launch_path)
        self.record_launcher = LaunchManager(self.record_launch_path)
        self.set_params_launcher = LaunchManager(self.set_params_launch_path)

        self.launchers = [
            self.camera_launcher,
            self.record_launcher,
            self.set_params_launcher
        ]

        self.start_camera_srv = rospy.Service(self.service_ns_name + "/start_camera", Trigger, self.start_camera_callback)
        self.stop_camera_srv = rospy.Service(self.service_ns_name + "/stop_camera", Trigger, self.stop_camera_callback)
        self.start_record_srv = rospy.Service(self.service_ns_name + "/start_record", Trigger, self.start_record_callback)
        self.stop_record_srv = rospy.Service(self.service_ns_name + "/stop_record", Trigger, self.stop_record_callback)
        self.is_camera_running_srv = rospy.Service(self.service_ns_name + "/is_camera_running", Trigger, self.is_camera_running_callback)

        self.camera_rate = rostopic.ROSTopicHz(15)
        self.camera_topic = self.camera_ns + "/color/image_raw"
        rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_rate.callback_hz, callback_args=self.camera_topic, queue_size=1)

        rospy.loginfo("%s init complete" % self.node_name)

    def get_publish_rate(self):
        result = self.camera_rate.get_hz(self.camera_topic)
        if result is None:
            return 0.0
        else:
            return result[0]

    def start_camera_callback(self, req):
        started = self.start_camera()
        return TriggerResponse(True, "Camera started" if started else "Camera is already running!")
    
    def is_camera_running_callback(self, req):
        if self.camera_launcher.is_running():
            rospy.sleep(1.0)
            rate = self.get_publish_rate()
            if self.min_rate_threshold <= rate <= self.max_rate_threshold:
                return TriggerResponse(True, "Camera is running and publishing at %0.2f Hz" % rate)
            else:
                return TriggerResponse(False, "Camera is running but not publishing within the threshold (%0.1f..%0.1f): %0.2f" % (self.min_rate_threshold, self.max_rate_threshold, rate))
        else:
            return TriggerResponse(False, "Camera is not started")

    def stop_camera_callback(self, req):
        stopped = self.camera_launcher.stop()
        self.record_launcher.stop()
        return TriggerResponse(True, "Camera stopped" if stopped else "Camera is already stopped!")
    
    def start_record_callback(self, req):
        started = self.record_launcher.start()
        return TriggerResponse(True, "Recording started" if started else "Recording is already running!")
    
    def is_camera_recording_callback(self, req):
        if self.record_launcher.is_running():
            return TriggerResponse(True, "Camera is recording")
        else:
            return TriggerResponse(False, "Camera is not recording")

    def stop_record_callback(self, req):
        stopped = self.record_launcher.stop()
        return TriggerResponse(True, "Recording stopped" if stopped else "Recording is already stopped!")
    
    def start_camera(self):
        started = self.camera_launcher.start()
        rospy.Timer(rospy.Duration(5.0), self.start_set_params, oneshot=True)
        return started

    def start_set_params(self, event):
        rospy.loginfo("Setting camera dynamic reconfigure parameters")
        self.set_params_launcher.start()

    def run(self):
        if self.on_start:
            started = self.start_camera()
            if not started:
                rospy.logerr("Camera failed to start!")
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            if not self.camera_launcher.is_running():
                continue
            rate = self.get_publish_rate()
            if not (self.min_rate_threshold <= rate <= self.max_rate_threshold):
                rospy.logwarn_throttle(2.0, "Camera isn't publishing at the expected rate (%0.1f..%0.1f): %0.1f" % (self.min_rate_threshold, self.max_rate_threshold, rate))

    def stop_all(self):
        for launcher in self.launchers:
            launcher.stop()

    def shutdown_hook(self):
        self.stop_all()


if __name__ == "__main__":
    node = TJ2CameraLauncher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
