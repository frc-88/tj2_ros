#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from tj2_interfaces.msg import RequestFrames


class FrameRequestManager:
    def __init__(self) -> None:
        self.enable_uvc_pub = rospy.Publisher("enable", Bool, queue_size=1)
        self.frame_request_sub = rospy.Subscriber(
            "request_frames", RequestFrames, self.frame_request_callback, queue_size=1
        )
        self.request_time = rospy.Time(0)
        self.request_duration = rospy.Duration(0)

    def frame_request_callback(self, request: RequestFrames) -> None:
        self.request_duration = request.request_duration
        self.request_time = rospy.Time.now()
        self.enable_uvc_pub.publish(Bool(data=False))

    def run(self) -> None:
        while not rospy.is_shutdown():
            if rospy.Time.now() - self.request_time > self.request_duration:
                self.enable_uvc_pub.publish(Bool(data=True))
            rospy.sleep(0.1)


def main() -> None:
    rospy.init_node("frame_request_manager")
    FrameRequestManager().run()


if __name__ == "__main__":
    main()
