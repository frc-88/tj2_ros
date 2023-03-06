#!/usr/bin/env python3
import os
import cv2
import datetime

import rospy

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image


class RecordToImages:
    def __init__(self):
        self.name = "record_to_video_file"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.images_dir = rospy.get_param("~images_dir", "./%Y-%m-%dT%H-%M-%S")
        self.image_format = rospy.get_param("~images_path", "image-{index}-%Y-%m-%dT%H-%M-%S--%f.jpg")
        self.rotate = rospy.get_param("~rotate", 0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=10)
        self.images_dir = os.path.realpath(datetime.datetime.now().strftime(self.images_dir))
        if not os.path.isdir(self.images_dir):
            os.makedirs(self.images_dir)

        rospy.loginfo("%s init complete" % self.name)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.rotate == 1:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 2:
            image = cv2.rotate(image, cv2.ROTATE_180)
        elif self.rotate == 3:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        frame_date = datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())
        image_path = frame_date.strftime(self.image_format)
        image_path = image_path.format(index=msg.header.seq)
        image_path = os.path.join(self.images_dir, image_path)
        print(f"Writing to {image_path}")
        cv2.imwrite(image_path, image)
        
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = RecordToImages()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
