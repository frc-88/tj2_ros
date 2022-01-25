#!/usr/bin/env python3
import rospy

import numpy as np

import torch
from torch.backends import cudnn
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.torch_utils import select_device
from yolov5.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.augmentations import letterbox
from yolov5.utils.general import LOGGER

from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


class Tj2Yolo:
    def __init__(self):
        self.name = "tj2_yolo"
        rospy.init_node(
            self.name
        )

        self.model_device = rospy.get_param("~model_device", "0")
        self.model_path = rospy.get_param("~model_path", "./yolov5s.pt")
        self.image_width = rospy.get_param("~image_width", 960)
        self.image_height = rospy.get_param("~image_height", 540)
        self.publish_overlay = rospy.get_param("~publish_overlay", True)
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.25)
        self.nms_iou_threshold = rospy.get_param("~nms_iou_threshold", 0.45)
        self.max_detections = rospy.get_param("~max_detections", 100)  # maximum detections per image

        self.classes_filter = None
        self.half = False  # flag for whether to use half or full precision floats
        self.augment = False  # augmented inference
        self.agnostic_nms = False  # class-agnostic NMS
        self.overlay_line_thickness = 3  # bounding box thickness (pixels)

        self.selected_model_device = select_device(self.model_device)
        self.model = DetectMultiBackend(self.model_path, device=self.selected_model_device, dnn=False)

        self.stride = self.model.stride
        self.class_names = self.model.names
        pt = self.model.pt
        jit = self.model.jit
        onnx = self.model.onnx
        engine = self.model.engine

        # Half
        self.half &= (pt or jit or onnx or engine) and self.selected_model_device.type != 'cpu'  # FP16 supported on limited backends with CUDA
        if pt or jit:
            self.model.model.half() if self.half else self.model.model.float()
        
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        self.image_size = (self.image_width, self.image_height)
        self.model.warmup(imgsz=(1, 3, *self.image_size), half=self.half)  # warmup
        
        self.bridge = CvBridge()
        self.color_image_sub = rospy.Subscriber("image_raw", Image, self.image_callback, queue_size=1)
        self.overlay_pub = rospy.Publisher("overlay", Image, queue_size=1)

        rospy.loginfo("%s is ready" % self.name)

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        detection_arr_msg, overlay_image = self.detect(cv2_img)
        if self.publish_overlay and overlay_image is not None:
            try:
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, encoding="bgr8")
                self.overlay_pub.publish(overlay_msg)
            except TypeError as e:
                rospy.logerr("Exception occurred while converting frame: %s. %s" % (e, overlay_image.shape))

    def detect(self, image):
        # Padded resize
        trans_image = letterbox(image, self.image_size, stride=self.stride, auto=True)[0]

        # Convert
        trans_image = trans_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        trans_image = np.ascontiguousarray(trans_image)

        torch_image = torch.from_numpy(trans_image).to(self.selected_model_device)
        torch_image = torch_image.half() if self.half else torch_image.float()  # uint8 to fp16/32
        torch_image /= 255  # 0 - 255 to 0.0 - 1.0
        if len(torch_image.shape) == 3:
            torch_image = torch_image[None]  # expand for batch dim
        
        # Inference
        prediction = self.model(torch_image, augment=self.augment, visualize=False)

        # NMS
        prediction = non_max_suppression(
            prediction,
            self.confidence_threshold,
            self.nms_iou_threshold,
            self.classes_filter,
            self.agnostic_nms,
            max_det=self.max_detections
        )

        detection_arr_msg = Detection2DArray()
        overlay_image = None

        assert len(prediction) <= 1
        if len(prediction) == 0:
            return detection_arr_msg, overlay_image

        detection = prediction[0]

        # Rescale boxes from torch_image size to image size
        detection[:, :4] = scale_coords(torch_image.shape[2:], detection[:, :4], image.shape).round()

        if self.publish_overlay:
            annotator = Annotator(image, line_width=self.overlay_line_thickness, example=str(self.class_names))
            for *xyxy, confidence, class_index in reversed(detection):
                class_index = int(class_index)
                label = f"{self.class_names[class_index]} {confidence:.2f}"
                annotator.box_label(xyxy, label, color=colors(class_index, True))
            overlay_image = annotator.result()
        else:
            overlay_image = None
        
        gain = torch.tensor(image.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        for *xyxy, confidence, class_index in reversed(detection):
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gain).view(-1).tolist()  # normalized xywh
            detection_msg = Detection2D()
            detection_msg.bbox.center.x = xywh[0]
            detection_msg.bbox.center.y = xywh[1]
            detection_msg.bbox.size_x = xywh[2]
            detection_msg.bbox.size_y = xywh[3]
            obj_with_pose = ObjectHypothesisWithPose()
            obj_with_pose.id = int(class_index)
            obj_with_pose.score = confidence
            detection_msg.results.append(obj_with_pose)

            detection_arr_msg.detections.append(detection_msg)
        
        return detection_arr_msg, overlay_image


    def run(self):
        rospy.spin()


def main():
    node = Tj2Yolo()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)


if __name__ == "__main__":
    main()
