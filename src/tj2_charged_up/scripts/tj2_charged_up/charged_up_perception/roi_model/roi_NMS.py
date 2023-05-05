import numpy as np
from .roi_params import *
from .roi_tools import *
from .roi_anchors import *


# box_cred_th, for any potential object with cred less than it will not be counted as obj
# but for training mode we have to find any cls so the value will be 0 for training.
def NMS(features, box_cred_th=0.0, cls_crd_th=0.0, maximumn_obj_num=1):
    def get_iou(box, boxes):
        l, r, t, b = list(box[2:])
        ll, rr, tt, bb = list(boxes[::, 2:].transpose())
        o_l, o_r, o_t, o_b = (
            np.maximum(l, ll),
            np.minimum(r, rr),
            np.maximum(t, tt),
            np.minimum(b, bb),
        )  # overlap lrtb
        ow, oh = np.maximum(0, o_r - o_l), np.maximum(0, o_b - o_t)
        area_of_overlap = ow * oh
        area_box, area_target = (r - l) * (b - t), (rr - ll) * (bb - tt)
        area_target = np.maximum(area_target, 1)
        area_of_union = area_box + area_target - area_of_overlap
        ious = area_of_overlap / area_of_union
        return ious

    batch_boxes = []
    for img_idx in range(features[0].shape[0]):
        boxes = np.zeros((6, 0))
        for anchor_layer_idx in range(len(features)):
            out = features[anchor_layer_idx][img_idx]
            out = out.reshape(out.shape[:2] + (anchor_base_n, -1))
            cls_plane = np.argmax(out[::, ::, ::, 1 : classes_num + 1], axis=-1)
            cls_confidence_plan = np.max(out[::, ::, ::, 1 : classes_num + 1], axis=-1)
            pos_points = np.where(
                (out[::, ::, ::, 0] > box_cred_th) * (cls_confidence_plan > cls_crd_th)
            )
            layer_boxes_params = out[pos_points]
            layer_base_boxes = anchors[anchor_layer_idx][pos_points]
            ls, rs, ts, bs, cxs, cys = list(layer_base_boxes.transpose())
            frms, tos = (
                1 + classes_num + cls_plane[pos_points] * 4,
                1 + classes_num + cls_plane[pos_points] * 4 + 4,
            )
            if len(frms) > 0:
                rls, rrs, rts, rbs = list(
                    np.array(
                        [
                            layer_boxes_param[frm:to]
                            for layer_boxes_param, frm, to in zip(
                                layer_boxes_params, frms, tos
                            )
                        ]
                    ).transpose()
                )
            else:
                rls, rrs, rts, rbs = [], [], [], []
            s, e = cxs + reg_range_d * (ls - cxs), cxs + reg_range_u * (ls - cxs)
            ls = np.minimum(
                np.maximum(0, (s + (e - s) * rls + reg_range_d)).astype(np.int0),
                IMG_W - 1,
            )
            s, e = cxs + reg_range_d * (rs - cxs), cxs + reg_range_u * (rs - cxs)
            rs = np.minimum(
                np.maximum(0, (s + (e - s) * rrs + reg_range_d)).astype(np.int0),
                IMG_W - 1,
            )
            s, e = cys + reg_range_d * (ts - cys), cys + reg_range_u * (ts - cys)
            ts = np.minimum(
                np.maximum(0, (s + (e - s) * rts + reg_range_d)).astype(np.int0),
                IMG_H - 1,
            )
            s, e = cys + reg_range_d * (bs - cys), cys + reg_range_u * (bs - cys)
            bs = np.minimum(
                np.maximum(0, (s + (e - s) * rbs + reg_range_d)).astype(np.int0),
                IMG_H - 1,
            )
            cls = cls_plane[pos_points]
            crds = layer_boxes_params[::, 0]
            layer_boxes = np.vstack((crds, cls, ls, rs, ts, bs))
            boxes = np.hstack((boxes, layer_boxes))
        boxes = boxes.transpose()

        for cls_idx in range(classes_num):
            cur_cls_boxes = boxes[np.where(boxes[::, 1] == cls_idx)]
            for i in range(maximumn_obj_num):
                if (
                    cur_cls_boxes.shape[0] == 0
                    or cur_cls_boxes[::, 0].max() <= box_cred_th
                ):
                    break
                max_crd_idx = np.argmax(cur_cls_boxes[::, 0])
                max_box = cur_cls_boxes[max_crd_idx].copy()
                max_box[0] = img_idx
                batch_boxes.append(max_box)
                ious = get_iou(max_box, cur_cls_boxes)
                cur_cls_boxes[np.where(ious > nms_iou_thresh), 0] = 0
    batch_boxes = np.array(batch_boxes, np.int32)
    return batch_boxes
