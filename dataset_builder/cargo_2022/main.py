import os
import cv2
import time
import json
import numpy as np
from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject
from tj2_tools.training.helpers import 


def load_annotation(path):
    with open(path) as file:
        annotations = json.load(file)

    keys_to_labels = {}
    for object_data in annotations["objects"]:
        keys_to_labels[object_data["key"]] = object_data["classTitle"]

    frames = []
    labels = set()
    for frame_data in annotations["frames"]:
        index = int(frame_data["index"])
        frame = PascalVOCFrame()
        if index >= len(frames):
            frames.extend([None for _ in range(index - len(frames) + 1)])

        frames[index] = frame

        frame.width = annotations["size"]["width"]
        frame.height = annotations["size"]["height"]
        frame.depth = 3

        for object_data in frame_data["figures"]:
            key = object_data["objectKey"]
            label = keys_to_labels[key]
            labels.add(label)
            ext_points = object_data["geometry"]["points"]["exterior"]
            xmin = ext_points[0][0]
            ymin = ext_points[0][1]
            xmax = ext_points[1][0]
            ymax = ext_points[1][1]

            obj = PascalVOCObject()
            obj.name = label
            obj.bndbox = [xmin, ymin, xmax, ymax]
            frame.add_object(obj)
    labels = list(labels)
    labels.sort()
    return frames, labels


def draw_bndbox(image, frame):
    draw_image = np.copy(image)
    for obj in frame.objects:
        # print(obj.bndbox)
        pt1 = obj.bndbox[0], obj.bndbox[1]
        pt2 = obj.bndbox[2], obj.bndbox[3]
        cv2.rectangle(draw_image, pt1, pt2, (0, 0, 255), 3)
    return draw_image


def write_image(image, path):
    directory = os.path.dirname(path)
    if not os.path.isdir(directory):
        os.makedirs(directory)
    cv2.imwrite(path, image)


def load_annotated_video(video_path, annotation_path, out_directory):
    capture = cv2.VideoCapture(video_path)
    frames, labels = load_annotation(annotation_path)
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

            if image_count >= len(frames):
                frame = None
            else:
                frame = frames[image_count]

            if frame is None:
                draw_img = image
            else:
                name = "image-%04d" % image_count
                img_path = os.path.join(out_directory, name + ".jpg")
                write_image(image, img_path)

                frame.set_path(img_path)
                anno_path = os.path.join(out_directory, name + ".xml")
                frame.write(anno_path)

                draw_img = draw_bndbox(image, frame)
            cv2.imshow("video", draw_img)
            image_count += 1
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main():
    load_annotated_video(
        "/home/ben/Downloads/Filming Videos/ds0/video/2021-11-19 09-33-03.mp4",
        "/home/ben/Downloads/Filming Videos/ds0/ann/2021-11-19 09-33-03.mp4.json",
        "./2021-11-19 09-33-03"
    )
    # load_annotated_video(
    #     "/home/ben/Downloads/Filming Videos/ds0/video/2021-11-19 09-32-48.mp4",
    #     "/home/ben/Downloads/Filming Videos/ds0/ann/2021-11-19 09-32-48.mp4.json",
    # )


if __name__ == "__main__":
    main()
