#!/usr/bin/env python3
import rospy
import rospkg
import rostopic

from std_srvs.srv import Trigger, TriggerResponse

from sensor_msgs.msg import Image

from dynamic_reconfigure.client import Client as DynamicClient

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
        self.expected_camera_rate = rospy.get_param("~expected_camera_rate", 15.0)
        self.expected_depth_rate = rospy.get_param("~expected_depth_rate", 15.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.color_min_rate_threshold = max(0.0, self.expected_camera_rate - self.min_rate_offset)
        self.color_max_rate_threshold = max(0.0, self.expected_camera_rate + self.min_rate_offset)
        self.depth_min_rate_threshold = max(0.0, self.expected_depth_rate - self.min_rate_offset)
        self.depth_max_rate_threshold = max(0.0, self.expected_depth_rate + self.min_rate_offset)
        self.l500_depth_config = rospy.get_param("~l500_depth_config", None)
        self.motion_module_config = rospy.get_param("~motion_module_config", None)
        self.rgb_camera_config = rospy.get_param("~rgb_camera_config", None)

        self.camera_launcher = LaunchManager(self.camera_launch_path)
        self.record_launcher = LaunchManager(self.record_launch_path)
        self.launchers = [
            self.camera_launcher,
            self.record_launcher,
        ]

        self.start_camera_srv = rospy.Service(self.service_ns_name + "/start_camera", Trigger, self.start_camera_callback)
        self.stop_camera_srv = rospy.Service(self.service_ns_name + "/stop_camera", Trigger, self.stop_camera_callback)
        self.start_record_srv = rospy.Service(self.service_ns_name + "/start_record", Trigger, self.start_record_callback)
        self.stop_record_srv = rospy.Service(self.service_ns_name + "/stop_record", Trigger, self.stop_record_callback)
        self.is_camera_running_srv = rospy.Service(self.service_ns_name + "/is_camera_running", Trigger, self.is_camera_running_callback)

        self.camera_rate = rostopic.ROSTopicHz(15)
        self.camera_topic = self.camera_ns + "/color/image_raw"
        rospy.Subscriber(self.camera_topic, rospy.AnyMsg, self.camera_rate.callback_hz, callback_args=self.camera_topic, queue_size=1)

        self.depth_rate = rostopic.ROSTopicHz(15)
        self.depth_topic = self.camera_ns + "/depth/image_raw"
        rospy.Subscriber(self.depth_topic, rospy.AnyMsg, self.depth_rate.callback_hz, callback_args=self.depth_topic, queue_size=1)

        self.l500_depth_client = None
        self.motion_module_client = None
        self.rgb_camera_client = None
        self.l500_depth_dyn_topic = self.camera_ns + "/l500_depth_sensor"
        self.motion_module_dyn_topic = self.camera_ns + "/motion_module"
        self.rgb_camera_dyn_topic = self.camera_ns + "/rgb_camera"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.l500_depth_client is None:
            self.l500_depth_client = DynamicClient(self.l500_depth_dyn_topic)
        if self.motion_module_client is None:
            self.motion_module_client = DynamicClient(self.motion_module_dyn_topic)
        if self.rgb_camera_client is None:
            self.rgb_camera_client = DynamicClient(self.rgb_camera_dyn_topic)

    def get_publish_rate(self):
        color_result = self.camera_rate.get_hz(self.camera_topic)
        depth_result = self.depth_rate.get_hz(self.depth_topic)
        color_rate = 0.0 if color_result is None else color_result[0]
        depth_rate = 0.0 if depth_result is None else depth_result[0]
        return color_rate, depth_rate

    def start_camera_callback(self, req):
        started = self.start_camera()
        return TriggerResponse(True, "Camera started" if started else "Camera is already running!")
    
    def is_camera_running_callback(self, req):
        if self.camera_launcher.is_running():
            rospy.sleep(1.0)
            color_rate, depth_rate = self.get_publish_rate()
            if (self.color_min_rate_threshold <= color_rate <= self.color_max_rate_threshold and
                    self.depth_min_rate_threshold <= depth_rate <= self.depth_max_rate_threshold):
                return TriggerResponse(True, "Color and depth are running and publishing at %0.2f Hz and %0.2f" % (color_rate, depth_rate))
            else:
                return TriggerResponse(False,
                    "Camera is running but not publishing within the threshold. "
                    "Color (%0.1f..%0.1f): %0.2f. Depth (%0.1f..%0.1f): %0.2f" % (
                        self.color_min_rate_threshold, self.color_max_rate_threshold, color_rate,
                        self.depth_min_rate_threshold, self.depth_max_rate_threshold, depth_rate)
                )
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
        rospy.Timer(rospy.Duration(1.0), self.set_camera_parameters, oneshot=True)
        return started

    def set_camera_parameters(self, event):
        self.init_dynamic_clients()
        if self.l500_depth_config is not None:
            rospy.wait_for_service(self.l500_depth_dyn_topic + "/set_parameters", 30.0)
            self.l500_depth_client.update_configuration(self.l500_depth_config)
            rospy.loginfo("Updating l500_depth parameters")
        else:
            rospy.loginfo("l500_depth parameters are not set. Skipping dynamic reconfigure")
        if self.motion_module_config is not None:
            rospy.wait_for_service(self.motion_module_dyn_topic + "/set_parameters", 30.0)
            self.motion_module_client.update_configuration(self.motion_module_config)
            rospy.loginfo("Updating motion_module parameters")
        else:
            rospy.loginfo("motion_module parameters are not set. Skipping dynamic reconfigure")
        if self.rgb_camera_config is not None:
            rospy.wait_for_service(self.rgb_camera_dyn_topic + "/set_parameters", 30.0)
            self.rgb_camera_client.update_configuration(self.rgb_camera_config)
            rospy.loginfo("Updating rgb_camera parameters")
        else:
            rospy.loginfo("rgb_camera parameters are not set. Skipping dynamic reconfigure")

    def run(self):
        if self.on_start:
            started = self.start_camera()
            if not started:
                rospy.logerr("Camera failed to start!")
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            if not self.camera_launcher.is_running():
                continue
            color_rate, depth_rate = self.get_publish_rate()
            if not (self.color_min_rate_threshold <= color_rate <= self.color_max_rate_threshold and
                    self.depth_min_rate_threshold <= depth_rate <= self.depth_max_rate_threshold):
                rospy.logwarn(2.0, 
                    "Camera is running but not publishing within the threshold. "
                    "Color (%0.1f..%0.1f): %0.2f. Depth (%0.1f..%0.1f): %0.2f" % (
                        self.color_min_rate_threshold, self.color_max_rate_threshold, color_rate,
                        self.depth_min_rate_threshold, self.depth_max_rate_threshold, depth_rate)
                )

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
