import numpy as np
from .roi_params import *


def get_anchors(
    skips=anchor_skips, base_sizes=anchor_base_sizes, aspect_ratios=anchor_aspect_ratios
):
    def base_boxes(l, rs):
        bbs = np.array([[l * r[0], l * r[1]] for r in rs], dtype=np.int32)
        return bbs

    def cal_feature_boxs(skip, bbs):
        h_skip_n, w_skip_n = IMG_H // skip, IMG_W // skip
        cxs = np.linspace(0, IMG_W - skip, w_skip_n) + skip / 2
        cys = np.linspace(0, IMG_H - skip, h_skip_n) + skip / 2
        ts, bs, ls, rs = [], [], [], []
        for i in range(anchor_base_n):
            ts.append(cys - bbs[i, 1])
            bs.append(cys + bbs[i, 1])
            ls.append(cxs - bbs[i, 0])
            rs.append(cxs + bbs[i, 0])
        ts, bs, ls, rs = np.array(ts), np.array(bs), np.array(ls), np.array(rs)
        LS = np.zeros(ts.shape + (ls.shape[1],))
        RS, TS, BS = LS.copy(), LS.copy(), LS.copy()
        for i in range(anchor_base_n):
            (LS[i], TS[i]), (RS[i], BS[i]) = np.meshgrid(ls[i], ts[i]), np.meshgrid(
                rs[i], bs[i]
            )
        CXS, CYS = np.meshgrid(cxs, cys)
        LS, RS, TS, BS = (
            LS.transpose(1, 2, 0),
            RS.transpose(1, 2, 0),
            TS.transpose(1, 2, 0),
            BS.transpose(1, 2, 0),
        )
        CXS, CYS = np.stack(([CXS] * anchor_base_n), axis=-1), np.stack(
            [CYS] * anchor_base_n, axis=-1
        )
        feature_boxes = (
            np.array([LS, RS, TS, BS, CXS, CYS]).transpose(1, 2, 3, 0).astype(np.int32)
        )
        return feature_boxes

    features_boxes = []  # in the sequence of ls, rs, ts and bs
    skip = skips[0]
    bbs = base_boxes(base_sizes[0], aspect_ratios[0])  # widths and heights
    features_boxes.append(cal_feature_boxs(skip, bbs))
    skip = skips[1]
    bbs = base_boxes(base_sizes[1], aspect_ratios[1])
    features_boxes.append(cal_feature_boxs(skip, bbs))
    skip = skips[2]
    bbs = base_boxes(base_sizes[2], aspect_ratios[2])
    features_boxes.append(cal_feature_boxs(skip, bbs))
    return features_boxes


anchors = get_anchors(
    skips=anchor_skips, base_sizes=anchor_base_sizes, aspect_ratios=anchor_aspect_ratios
)
