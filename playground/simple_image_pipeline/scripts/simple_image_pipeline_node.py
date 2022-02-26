#!/usr/bin/env python3
import os
import cv2
import json
import datetime
import numpy as np

import rospy

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from tj2_tools.rosbag_to_file import utils
from tj2_tools.rosbag_to_file.rosbag_to_json import StreamArray


class SimpleImagePipeline:
    def __init__(self):
        self.name = "simple_image_pipeline"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)
        self.video_path = rospy.get_param("~video_path", "./video.mp4")
        self.video_fps = rospy.get_param("~video_fps", 30)
        self.bridge = CvBridge()
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=10)
        #self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.image_callback, queue_size=10)

        self.video_path = datetime.datetime.now().strftime(self.video_path)

        self.json_objects = []

        self.video_writer = None
        self.debug_image_pub = rospy.Publisher("black_block_raw", Image, queue_size=10)

        rospy.loginfo("%s init complete" % self.name)

    def add_object(self, timestamp, name, obj):
        row = [
            timestamp,
            name,
            obj
        ]
        rospy.loginfo(str(row))
        self.json_objects.append(row)
    
    def get_timestamp(self, ros_time=None):
        if ros_time is None:
            ros_time = rospy.Time.now()
        return float(ros_time.to_sec())

    def init_video_writer(self, shape):
        height, width = shape[0:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        return cv2.VideoWriter(self.video_path, fourcc, self.video_fps, (width, height))

    def info_callback(self, msg):
        dict_msg = utils.msg_to_dict(msg)
        self.add_object(self.get_timestamp(), "camera_info", dict_msg)
        self.info_sub.unregister()  # only use the first message
        rospy.loginfo("Camera model loaded")
    
    def image_callback(self, msg):
        try:
            #image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            image = np.copy(image)
            gray_img = (image/256).astype(np.uint8)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        ret, th = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)
        ret, labels, stats, centroid = cv2.connectedComponentsWithStats(th)
        mask = (np.ones(gray_img.shape)*255).astype(np.uint8)
        connected_component_size_thresh = 500
        stats = [stat for stat in stats if stat[-1] < connected_component_size_thresh]
        for i, stat in enumerate(stats):
            l, r, t, b = stat[0], stat[0]+stat[2]+1, stat[1], stat[1]+stat[3]+1
            mask[t:b,l:r] = 0
        image[mask == 0] = 0

        erosion_size = 3
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                           (erosion_size, erosion_size))
        image = cv2.erode(image, element)

        dilatation_size = erosion_size
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                           (dilatation_size, dilatation_size))
        image = cv2.dilate(image, element)
        # value, image = cv2.threshold(image, 1, 0xffff, cv2.THRESH_BINARY)

        #print("1:", image.dtype, image.shape)
        #image = cv2.medianBlur(image, 5)
        #print("2:", image.dtype, image.shape)
        #image = mask
        '''
        num_labels, labels_im = cv2.connectedComponents(img_8bit)
        print(num_labels)
        print(labels_im.shape)
        print(len(labels_im))
        image = img_8bit
        '''
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
        else:
            self.debug_image_pub.publish(image_msg)
        '''
        if self.video_writer is None:
            self.video_writer = self.init_video_writer(image.shape)

        self.add_object(self.get_timestamp(msg.header.stamp), "frame", {"seq": msg.header.seq})
        self.video_writer.write(image)
        '''
    def objs_generator(self):
        for row in self.json_objects:
            yield row

    def objs_to_json(self):
        stream_array = StreamArray(self.objs_generator())
        name = os.path.splitext(os.path.basename(self.video_path))[0]
        directory = os.path.dirname(self.video_path)
        path = os.path.join(directory, name + ".json")
        rospy.loginfo("Writing frame info to %s" % path)
        with open(path, 'w') as outfile:
            for chunk in json.JSONEncoder(indent=4).iterencode(stream_array):
                outfile.write(chunk)

    def run(self):
        rospy.spin()

    def shutdown_hook(self):
        self.objs_to_json()
        if self.video_writer is not None:
            rospy.loginfo("Writing video to %s" % self.video_path)
            self.video_writer.release()


if __name__ == "__main__":
    node = SimpleImagePipeline()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
