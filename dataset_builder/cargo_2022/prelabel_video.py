import os
import sys
import cv2
import time
from pathlib import Path

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.yolo.detector import YoloDetector
from tj2_tools.training.pascal_voc import PascalVOCFrame
from tj2_tools.training.pascal_voc import PascalVOCObject


def main(video_path, output_dir, frame_skip=50):
    if not output_dir.is_dir():
        print("Making directory:", output_dir)
        os.makedirs(str(output_dir))

    capture = cv2.VideoCapture(str(video_path))
    success, image = capture.read()
    if success:
        height, width = image.shape[0:2]
        yolo = YoloDetector(0, "/home/ben/tj2_ros/tj2_yolo/models/cargo_2022.pt", width, height,
                            confidence_threshold=0.65, nms_iou_threshold=0.45,
                            publish_overlay=True)
    else:
        raise RuntimeError("Failed to read from video: %s" % video_path)

    with open(output_dir / "labels.txt", 'w') as file:
        file.write("\n".join(yolo.class_names))

    paused = False
    image_count = 0
    try:
        while True:
            key = cv2.waitKey(1)
            key &= 0xff
            key = chr(key)
            if key == 'q':
                break
            elif key == ' ':
                paused = not paused
                print("paused:", paused)
            if paused:
                time.sleep(0.05)
                continue
            success, image = capture.read()
            if not success:
                print("Failed to get frame")
                return
            image_path = output_dir / ("image-%05d.jpg" % image_count)
            anno_path = output_dir / ("image-%05d.xml" % image_count)

            image_count += 1
            if image_count % frame_skip != 0:
                continue
            detections, overlay = yolo.detect(image)

            frame = PascalVOCFrame()
            frame.width = image.shape[1]
            frame.height = image.shape[0]
            frame.depth = image.shape[2]
            frame.set_path(str(image_path))
            for obj_id, (xywh, confidence) in detections.items():
                obj = PascalVOCObject()
                label, index = yolo.get_label(obj_id)
                obj.name = label
                obj.bndbox[0] = int(xywh[0])
                obj.bndbox[1] = int(xywh[1])
                obj.bndbox[2] = int(xywh[2])
                obj.bndbox[3] = int(xywh[3])

                frame.objects.append(obj)
            print(anno_path)
            print(image_path)
            frame.write(str(anno_path))
            cv2.imwrite(str(image_path), image)

            cv2.imshow("video", overlay)
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(Path("resources/br-build-room-2022-01-26/Image from iOS.mov"),
         Path("resources/br-build-room-2022-01-26-images"), frame_skip=10)
    # main(Path("resources/br-build-room-2022-01-26/Image from iOS (1).mov"),
    #      Path("resources/br-build-room-2022-01-26-1-images"), frame_skip=10)
