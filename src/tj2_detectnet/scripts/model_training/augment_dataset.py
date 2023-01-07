# take existing dataset and artificially transform the data in order to generate more training data
import os
import cv2
import random
import numpy as np
from tj2_tools.training.pascal_voc import PascalVOCFrame, PascalVOCObject
from utils import read_dataset, build_image_sets



def random_warp(offset):
    return offset + random.randint(0, 30)


def get_random_warp(width, height):
    random_center = random.randint(300, 400)
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
    return warp_mat


def warp_bndbox(warp_mat, bndbox):
    pt1 = np.array([bndbox[0], bndbox[1], 1], dtype=np.int32)
    pt2 = np.array([bndbox[2], bndbox[3], 1], dtype=np.int32)
    pts = np.array([pt1, pt2])

    warp_pts = np.dot(pts, warp_mat.T)
    warp_pts = warp_pts[..., 0:2]
    warp_pts = warp_pts.reshape((-1, 1, 2))
    warp_pts = warp_pts.astype(np.int32)

    warp_pts = warp_pts[:, 0]
    warp_bndbox = [
        warp_pts[0][0], warp_pts[0][1],
        warp_pts[1][0], warp_pts[1][1]
    ]
    return warp_bndbox


def disect_path(path):
    dirname = os.path.dirname(path)
    filename = os.path.basename(path)
    name, ext = os.path.splitext(filename)
    return dirname, filename, name, ext


def apply_random_warp(annotation, postfix, dry_run=True):
    image_path = annotation.path
    image_dir, image_filename, image_name, image_ext = disect_path(image_path)
    anno_dir, anno_filename, anno_name, anno_ext = disect_path(annotation.frame_path)

    assert anno_name == image_name

    warp_image_filename = image_name + postfix + image_ext
    warp_image_path = os.path.join(image_dir, warp_image_filename)

    warp_anno_filename = anno_name + postfix + anno_ext
    warp_anno_path = os.path.join(anno_dir, warp_anno_filename)

    warp_annotation = PascalVOCFrame.from_frame(annotation)
    warp_annotation.set_path(warp_image_path)

    src = cv2.imread(image_path)
    width = src.shape[1]
    height = src.shape[0]
    size = width, height

    warp_mat = get_random_warp(width, height)
    warp_dst = cv2.warpAffine(src, warp_mat, size)

    warp_annotation.objects = []
    for obj in annotation.objects:
        warped_bndbox = warp_bndbox(warp_mat, obj.bndbox)
        warp_obj = PascalVOCObject.from_obj(obj)
        warp_obj.bndbox = warped_bndbox
        if warp_obj.is_out_of_bounds(width, height):
            continue
        warp_obj.truncated = warp_obj.is_truncated(width, height)
        warp_obj.constrain_bndbox(width, height)
        warp_annotation.add_object(warp_obj)

    if len(warp_annotation.objects) == 0:
        print("Warped image has no objects in view. Skipping: %s" % image_path)
        return

    print("Warped image: %s" % warp_image_path)
    if not dry_run:
        cv2.imwrite(warp_image_path, warp_dst)
    print("Warped annotation: %s" % warp_anno_path)
    if not dry_run:
        warp_annotation.write(warp_anno_path)


def generate_warps(dataset, num_warps, dry_run=True):
    postfix_format = "_warp"
    for annotation in dataset.values():
        if postfix_format in annotation.frame_id:
            continue

        for count in range(0, num_warps):
            count += 1
            postfix = postfix_format + str(count)
            apply_random_warp(annotation, postfix, dry_run)


def main():
    source_dir = "/home/ben/jetson-inference/python/training/detection/ssd/data/tj2_2020_voc_image_database"
    # dataset = read_dataset(source_dir)
    # generate_warps(dataset, 2, dry_run=False)

    dataset = read_dataset(source_dir)
    build_image_sets(dataset, source_dir, dry_run=False)


if __name__ == '__main__':
    main()
