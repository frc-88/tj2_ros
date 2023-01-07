import os
import cv2
import random
import numpy as np


def random_warp(offset):
    return offset + random.randint(0, 50)


def warp_test():
    image_dir = "/home/ben/object-recognition/detectnet_training/data/tj2_2020_voc_image_database/JPEGImages"
    # image_filename = "5172_POV-Great_Northern_2020_Quals_22.mp4_00060.jpg"
    image_filename = "powercell_05f3becf-ccf5-4376-9bbc-e71b4b68d107.jpg"
    image_path = os.path.join(image_dir, image_filename)

    src = cv2.imread(image_path)
    width = src.shape[1]
    height = src.shape[0]

    center_x = width // 2
    center_y = height // 2
    shape_offset = 50
    shape_pts = np.array([[center_x - shape_offset, center_y - shape_offset],
                  [center_x + shape_offset, center_y - shape_offset],
                  [center_x + shape_offset, center_y + shape_offset],
                  [center_x - shape_offset, center_y + shape_offset]]).astype(np.int32)
    draw_shape_pts = shape_pts.reshape((-1, 1, 2))
    src = cv2.polylines(
        src, [draw_shape_pts], True, (255, 0, 0), 6
    )

    random_center = random.randint(500, 600)
    random_offset_x = random.randint(-100, 100)
    random_offset_y = random.randint(-100, 100)
    src_triangle = np.array([[0, 0], [width - 1, 0], [0, height - 1]]).astype(np.float32)
    dst_triangle = np.array([
        [-random_warp(random_center), -random_warp(random_center)],
        [width + random_warp(random_center), -random_warp(random_center)],
        [-random_warp(random_center), height + random_warp(random_center)]]
    ).astype(np.float32)
    dst_triangle[:, 0] += random_offset_x
    dst_triangle[:, 1] += random_offset_y

    warp_mat = cv2.getAffineTransform(src_triangle, dst_triangle)
    warp_dst = cv2.warpAffine(src, warp_mat, (src.shape[1], src.shape[0]))

    ones_column = np.ones(len(shape_pts))
    ones_column = ones_column.reshape((-1, 1))
    shape_pts = np.concatenate((shape_pts, ones_column), axis=1)
    warp_pts = np.dot(shape_pts, warp_mat.T)
    warp_pts = warp_pts[..., 0:2]
    warp_pts = warp_pts.reshape((-1, 1, 2))
    warp_pts = warp_pts.astype(np.int32)
    warp_dst = cv2.polylines(
        warp_dst, [warp_pts], True, (0, 0, 255), 4
    )

    display_img = np.concatenate((src, warp_dst))
    display_img = cv2.resize(display_img, (display_img.shape[1] // 3, display_img.shape[0] // 3))

    cv2.imshow("warped", display_img)
    key = cv2.waitKey()
    if key <= 255:
        return chr(key) == 'q'
    else:
        return False


def main():
    while True:
        if warp_test():
            break


if __name__ == '__main__':
    main()
