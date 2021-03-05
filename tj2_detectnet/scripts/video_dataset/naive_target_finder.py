import os
import re
import cv2
import math
import numpy as np
from pascal_voc import PascalVOCFrame, PascalVOCObject

IMAGE = None
PATHS = []
FILE_INDEX = 0
LABELS = {}

threshold = [
    # 0, 41, 24,
    # 21, 255, 252

    20, 174, 14,
    33, 255, 255
]

draw_width = 500
window_name = "image"
trackbar_window_name = "thresholds"


def load_image(path):
    global IMAGE
    print("loading %s" % path)
    IMAGE = cv2.imread(path)


def naive_target_pipeline(image, lower, upper):
    contour_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    contour_image = cv2.GaussianBlur(contour_image, (15, 15), 0)
    contour_image = cv2.inRange(contour_image, lower, upper)

    contour_image = cv2.dilate(contour_image, None, iterations=4)
    # contour_image = cv2.erode(contour_image, None, iterations=4)

    contours, hierarchy = cv2.findContours(contour_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours_area = []
    # calculate area and filter into new array
    for con in contours:
        area = cv2.contourArea(con)
        if 300 < area < 15000:
            contours_area.append(con)

    contours_circles = []
    bounding_boxes = []

    # check if contour is of circular shape
    for con in contours_area:
        perimeter = cv2.arcLength(con, True)
        area = cv2.contourArea(con)
        if perimeter == 0:
            break
        circularity = 4 * math.pi * (area / (perimeter * perimeter))
        # print(circularity)
        if 0.6 < circularity < 1.4:
            contours_circles.append(con)

            bndbox = cv2.boundingRect(con)
            bounding_boxes.append(bndbox)

    contour_image = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)
    image = np.copy(IMAGE)

    cv2.drawContours(contour_image, contours_circles, -1, (0, 255, 0), 3)
    for bndbox in bounding_boxes:
        x, y, w, h = bndbox
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return bounding_boxes, contour_image, image


def on_trackbar(index, value):
    threshold[index] = value
    lower = tuple(threshold[0:3])
    upper = tuple(threshold[3:6])
    print(str(lower)[1:-1] + ", ")
    print(str(upper)[1:-1])
    print()
    height, width, depth = IMAGE.shape

    bounding_boxes, contour_image, image = naive_target_pipeline(IMAGE, lower, upper)

    image_path = PATHS[FILE_INDEX]
    frame = PascalVOCFrame()
    frame.set_path(image_path)
    frame.width = width
    frame.height = height
    frame.depth = depth

    for bndbox in bounding_boxes:
        x, y, w, h = bndbox
        obj = PascalVOCObject()
        obj.name = "powercell"
        obj.bndbox = [x, y, x + w, y + h]
        obj.truncated = obj.is_truncated(width, height)

        frame.add_object(obj)
    LABELS[FILE_INDEX] = frame

    draw_height = int(height * draw_width / width)
    contour_image = cv2.resize(contour_image, (draw_width, draw_height))
    small_image = cv2.resize(image, (draw_width, draw_height))

    # small_image = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)[..., 1]
    # small_image = cv2.cvtColor(small_image, cv2.COLOR_GRAY2BGR)

    contour_image = np.concatenate((contour_image, small_image))

    cv2.imshow(window_name, contour_image)


def create_window():
    trackbar_r_lower = "H lower"
    trackbar_g_lower = "S lower"
    trackbar_b_lower = "V lower"
    trackbar_r_upper = "H upper"
    trackbar_g_upper = "S upper"
    trackbar_b_upper = "V upper"
    trackbars = [
        trackbar_r_lower,
        trackbar_g_lower,
        trackbar_b_lower,
        trackbar_r_upper,
        trackbar_g_upper,
        trackbar_b_upper
    ]
    threshold_index_mapping = [
        2, 1, 0, 5, 4, 3
        # 3, 4, 5, 0, 1, 2
    ]

    draw_order = [1, 2, 3, 5, 4, 0]

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(trackbar_window_name, cv2.WINDOW_AUTOSIZE)
    for index in draw_order:
        name = trackbars[index]
        cv2.createTrackbar(name, trackbar_window_name, 0, 255, lambda value, index=index: on_trackbar(index, value))
        cv2.setTrackbarPos(name, trackbar_window_name, threshold[index])


def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(data, key=alphanum_key)


def get_filenames(dirname):
    paths = []
    for dirpath, dirnames, filenames in os.walk(dirname):
        for filename in sorted_alphanumeric(filenames):
            if filename.endswith(".png"):
                paths.append(os.path.join(dirname, filename))
    return paths


def find_paths():
    global PATHS, FILE_INDEX
    dirname = "tj2_02-27-2021_2"
    basedir = "/Users/Woz4tetra/Documents/Code/2021-Robot-ROS/tj2_detectnet/scripts/video_dataset/output"

    PATHS = get_filenames(os.path.join(basedir, dirname))

    load_image(PATHS[FILE_INDEX])


def cycle_through_images():
    global PATHS, FILE_INDEX
    find_paths()
    create_window()

    while True:
        on_trackbar(0, threshold[0])
        key = cv2.waitKey(-1)
        if key > 255:
            continue
        key = chr(key)
        if key == 'q':
            break
        elif key == 'd':
            FILE_INDEX += 1
            if FILE_INDEX >= len(PATHS):
                FILE_INDEX = 0
            print("Image index: %s" % FILE_INDEX)
            load_image(PATHS[FILE_INDEX])

        elif key == 'a':
            FILE_INDEX -= 1
            if FILE_INDEX < 0:
                FILE_INDEX = len(PATHS) - 1
            print("Image index: %s" % FILE_INDEX)
            load_image(PATHS[FILE_INDEX])

        elif key == 's':
            root, path = LABELS[FILE_INDEX].write()
            print("Saved label path: %s" % path)

    cv2.destroyAllWindows()


def process_all_images():
    global PATHS, FILE_INDEX
    find_paths()
    create_window()
    for FILE_INDEX in range(46, len(PATHS)):
        print("Image index: %s" % FILE_INDEX)
        load_image(PATHS[FILE_INDEX])
        on_trackbar(0, threshold[0])
        root, path = LABELS[FILE_INDEX].write()
        print("Saved label path: %s" % path)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


def main():
    process_all_images()
    # cycle_through_images()


if __name__ == '__main__':
    main()
