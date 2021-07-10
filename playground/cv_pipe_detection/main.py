import cv2
import math
import numpy as np


def dilate(image):
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.dilate(image, element)


def skeletonize(image, max_num_steps=150):
    skel = np.zeros_like(image)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (7, 7))
    step = 0
    for step in range(max_num_steps):
        # Step 2: Open the image
        open = cv2.morphologyEx(image, cv2.MORPH_OPEN, element)
        # Step 3: Substract open from the original image
        temp = cv2.subtract(image, open)
        # Step 4: Erode the original image and refine the skeleton
        eroded = cv2.erode(image, element)
        skel = cv2.bitwise_or(skel, temp)
        image = eroded.copy()
        # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
        if cv2.countNonZero(image) == 0:
            break
    print("Eroded in %s steps" % step)
    return skel


def houghlines(image, draw_image):
    edges = cv2.Canny(image, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1.0, np.pi / 360, 100, minLineLength=200, maxLineGap=50)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = math.atan2(y2 - y1, x2 - x1)
        if not (-math.pi/4 < angle < math.pi/4):
            continue
        cv2.line(draw_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return draw_image


def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
                         "otherwise OpenCV changed their cv2.findContours return "
                         "signature yet again. Refer to OpenCV's documentation "
                         "in that case"))

    # return the actual contours array
    return cnts


def contours(image):
    edges = cv2.Canny(image, 30, 200)
    contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    draw_image = np.zeros_like(image)
    for contour in grab_contours(contours):
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 200:
            continue
        cv2.drawContours(draw_image, [contour], -1, (255, 255, 255), 1)
    return draw_image


def main():
    # path = "images/realsense_2021-06-19-15-09-57/1624129825542816639.png"
    path = "images/realsense_2021-06-19-15-09-57/1624129807297926664.png"

    # path = "images/realsense_2021-06-19-15-14-30/1624130076387370586.png"
    # path = "images/realsense_2021-06-19-15-14-30/1624130079406070709.png"

    image = cv2.imread(path, cv2.IMREAD_ANYDEPTH)
    height, width = image.shape[-2:]
    image = image[:, 0: width - 50]

    if "realsense_2021-06-19-15-14-30" in path:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    draw_img = np.copy(image)
    draw_img = cv2.normalize(draw_img, draw_img, 0, 255, cv2.NORM_MINMAX)
    draw_img = np.uint8(draw_img)
    draw_img = cv2.cvtColor(draw_img, cv2.COLOR_GRAY2BGR)

    image = dilate(image)
    # threshold, image = cv2.threshold(image, 5000, 65535, cv2.THRESH_TOZERO_INV)
    # threshold, image = cv2.threshold(image, 1500, 65535, cv2.THRESH_TOZERO)
    # threshold, image = cv2.threshold(image, 2500, 65535, cv2.THRESH_TOZERO_INV)
    # threshold, image = cv2.threshold(image, 2400, 65535, cv2.THRESH_TOZERO)
    # image = cv2.blur(image, (3, 3))
    # image = skeletonize(image)

    image = cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
    image = np.uint8(image)
    image = cv2.medianBlur(image, 3)
    image = contours(image)
    output_image = houghlines(image, draw_img)

    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = np.concatenate((output_image, image))

    cv2.imshow("image", image)
    key = ''
    while key != 'q':
        value = cv2.waitKey(-1)
        if 0 <= value < 256:
            key = chr(value)

            if key == 's':
                cv2.imwrite("image.jpg", image)


if __name__ == '__main__':
    main()
