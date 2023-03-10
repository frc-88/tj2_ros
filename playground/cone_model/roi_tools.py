
import numpy as np

def get_iou(box, target):
    l, r, t, b = box
    ll, rr, tt, bb = target
    o_l, o_r, o_t, o_b = max(l, ll), min(r, rr), max(t, tt), min(b, bb)  # overlap lrtb
    ow, oh = max(0, o_r - o_l), max(0, o_b - o_t)
    area_of_overlap = ow * oh
    area_box, area_target = (r - l) * (b - t), (rr - ll) * (bb - tt)
    area_of_union = area_box + area_target - area_of_overlap
    iou = area_of_overlap / area_of_union
    return iou


def get_ious(box, features_boxes):
    l, r, t, b = box
    features_ious = []
    for feature_boxes in features_boxes:
        ols = np.maximum(l, feature_boxes[::, ::, ::, 0])
        ors = np.minimum(r, feature_boxes[::, ::, ::, 1])
        ots = np.maximum(t, feature_boxes[::, ::, ::, 2])
        obs = np.minimum(b, feature_boxes[::, ::, ::, 3])
        ows, ohs = np.maximum(0, ors - ols), np.maximum(0, obs - ots)
        areas_of_overlaps = ows * ohs
        area_of_box = (r - l) * (b - t)
        areas_of_feature_boxes = (feature_boxes[::, ::, ::, 1] - feature_boxes[::, ::, ::, 0]) * (
                feature_boxes[::, ::, ::, 3] - feature_boxes[::, ::, ::, 2])
        areas_of_unions = areas_of_feature_boxes + area_of_box - areas_of_overlaps
        ious = areas_of_overlaps / areas_of_unions
        features_ious.append(ious)
    return features_ious
