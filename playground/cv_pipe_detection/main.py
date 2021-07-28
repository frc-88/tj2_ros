import cv2
import math
import datetime
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def dilate(image):
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.dilate(image, element)


def skeletonize(image, max_num_steps=150):
    skel = np.zeros_like(image)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
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


def houghlines(edges, depth_image, draw_image):
    lines = cv2.HoughLinesP(edges, 1.1, np.pi / 360, 150, minLineLength=100, maxLineGap=200)
    result = []
    raw_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = math.atan2(y2 - y1, x2 - x1)
        if not (-math.pi / 4 < angle < math.pi / 4):
            continue
        cv2.line(draw_image, (x1, y1), (x2, y2), (255, 255, 255), 1)

        mask_image = np.zeros_like(draw_image).astype(np.uint8)
        cv2.line(mask_image, (x1, y1), (x2, y2), (255, 255, 255), 1)
        
        masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=mask_image)

        column_sums = masked_depth.sum(axis=0)
        nonzero_count = ((masked_depth > 1) & (masked_depth < 100)).sum(axis=0)
        zs = np.true_divide(column_sums, nonzero_count, where=nonzero_count != 0)

        cloud = []
        for xn in range(x1, x2):
            zn = zs[xn]
            if zn < 1:
                continue
            yn = (y2 - y1) / (x2 - x1) * (xn - x1) + y1
            cloud.append((xn, yn, zn))
        if len(cloud) == 0:
            continue
        cloud = np.array(cloud)

        z_cloud = cloud[:, 2]
        z_std = z_cloud.std()
        z_dist = np.abs(z_cloud - z_cloud.mean())
        not_outlier = z_dist < 1.5 * z_std
        # cloud_dist = cloud_dist[not_outlier]

        cloud = cloud[not_outlier]

        xy_fit = np.polyfit(cloud[:, 0], cloud[:, 1], 1)
        xz_fit = np.polyfit(cloud[:, 0], cloud[:, 2], 1)
        
        y1_fit = xy_fit[0] * x1 + xy_fit[1]
        y2_fit = xy_fit[0] * x2 + xy_fit[1]
        z1_fit = xz_fit[0] * x1 + xz_fit[1]
        z2_fit = xz_fit[0] * x2 + xz_fit[1]

        line_points = np.array([[x1, y1_fit, z1_fit], [x2, y2_fit, z2_fit]])

        # cloud_mean = cloud.mean(axis=0)
        # cloud_dist = cloud - cloud_mean

        
        raw_lines.append(cloud)

        # uu, dd, vv = np.linalg.svd(cloud_dist)
        
        # # x_window = max(x1, x2) // 2
        # # y_window = max(y1, y2) // 2
        # # window = max(x_window, y_window)
        # window = 500
        # line_points = vv[0] * np.array([[window], [-window]]) + cloud_mean

        result.append(line_points)
        
        # result.append((x1, y1, x2, y2))
    # raw_lines = np.array(raw_lines).T
    return result, raw_lines, draw_image

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
    # image = cv2.Canny(image, 50, 150, apertureSize=3)
    contours = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    draw_image = np.zeros_like(image)
    for contour in grab_contours(contours):
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 200:
            continue
        cv2.drawContours(draw_image, [contour], -1, (255, 255, 255), 1)
    return draw_image


def sobel(image):
    ksize = 3
    # gX = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=ksize)
    # return gX
    gY = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=ksize)
    # gX = cv2.convertScaleAbs(gX)
    # gY = cv2.convertScaleAbs(gY)
    return gY
    # combined = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
    # return combined

def mouse_event(image, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(image[y][x])


def sobel_pipeline(image, draw_img):
    image = dilate(image)

    image = cv2.medianBlur(image, 3)

    image = sobel(image)
    image = cv2.normalize(image, image, 0, 65535, cv2.NORM_MINMAX)
    image = np.uint16(image)

    image = cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
    image = np.uint8(image)

    return image


def contour_pipeline_1(depth_image, draw_img):
    image = dilate(depth_image)
    # threshold, image = cv2.threshold(image, 5000, 65535, cv2.THRESH_TOZERO_INV)
    # threshold, image = cv2.threshold(image, 1500, 65535, cv2.THRESH_TOZERO)
    # threshold, image = cv2.threshold(image, 2500, 65535, cv2.THRESH_TOZERO_INV)
    # threshold, image = cv2.threshold(image, 2400, 65535, cv2.THRESH_TOZERO)
    # image = cv2.blur(image, (3, 3))
    # image = skeletonize(image)

    image = cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
    image = np.uint8(image)
    blurred = cv2.medianBlur(image, 3)
    

    # image = cv2.Canny(image, 30, 200)
    image = cv2.Canny(blurred, 50, 150, apertureSize=3)
    image = contours(image)
    lines, raw_lines, lines_image = houghlines(image, blurred, np.copy(blurred))

    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    output_image = lines_image
    # output_image = np.concatenate((lines_image, image))

    return lines, raw_lines, output_image


def main():
    # path = "images/realsense_2021-06-19-15-09-57/1624129825542816639.png"
    # path = "images/realsense_2021-06-19-15-09-57/1624129807297926664.png"

    # path = "images/realsense_2021-06-19-15-14-30/1624130076387370586.png"
    path = "images/realsense_2021-06-19-15-14-30/1624130079406070709.png"

    image = cv2.imread(path, cv2.IMREAD_ANYDEPTH)
    height, width = image.shape[-2:]
    image = image[:, 0: width - 50]

    if "realsense_2021-06-19-15-14-30" in path:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    draw_img = np.copy(image)
    draw_img = cv2.normalize(draw_img, draw_img, 0, 255, cv2.NORM_MINMAX)
    draw_img = np.uint8(draw_img)
    draw_img = cv2.cvtColor(draw_img, cv2.COLOR_GRAY2BGR)

    # image = sobel_pipeline(image, draw_img)
    lines, raw_lines, image = contour_pipeline_1(image, draw_img)

    # cv2.namedWindow("image")
    # cv2.setMouseCallback("image", lambda *args: mouse_event(image, *args))
    # cv2.imshow("image", image)
    # key = ''
    # while key != 'q':
    #     value = cv2.waitKey(-1)
    #     if 0 <= value < 256:
    #         key = chr(value)

    #         if key == 's':
    #             date_str = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S--%f")
    #             cv2.imwrite("image-%s.jpg" % date_str, image)

    # ax = fig.add_subplot(projection='3d')
    # ax.scatter(lines[0], lines[1], lines[2])

    fig, (ax1, ax2) = plt.subplots(1, 2)
    for line in lines:
        ax1.plot(line[:, 0], line[:, 2])
    plt.gca().set_prop_cycle(None)
    for cloud in raw_lines:
        ax1.scatter(cloud[:, 0], cloud[:, 2])
    # ax1.scatter(raw_lines[0], raw_lines[2])

    plt.imshow(image)
    for line in lines:
        ax2.plot(line[:, 0], line[:, 1])
    plt.gca().set_prop_cycle(None)
    for cloud in raw_lines:
        ax2.scatter(cloud[:, 0], cloud[:, 1])
    # ax2.scatter(lines[0], lines[1])


    # X = np.arange(0, lines_image.shape[1])
    # Y = np.zeros((lines_image.shape[0],))
    # X, Y = np.meshgrid(X, Y)
    # Z = np.arange(0, lines_image.shape[0])
    # Z = np.tile(Z, (lines_image.shape[1], 1))
    # Z = Z.T
    # print(Z)

    # X = np.arange(0, lines_image.shape[1])
    # Y = np.arange(0, lines_image.shape[0])
    # X, Y = np.meshgrid(X, Y)
    # Z = np.ones((lines_image.shape[0], lines_image.shape[1]))

    # print(lines_image.shape, X.shape, Y.shape, Z.shape)
    
    # face_colors_image = lines_image.astype(np.float64) / 255.0
    # ax.plot_surface(X, Y, Z, rstride=3, cstride=3, facecolors=face_colors_image)
    
    plt.show()


if __name__ == '__main__':
    main()
