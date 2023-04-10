
import os
import cv2
import json
from roi_tools import *
from roi_anchors import *
from roi_params import *
import random

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

def get_anchors_pool():
    # for no overlapped neg samples
    anchors_pool = np.zeros((4, 0))
    zero_ious, _ = get_ious_withins([0, 0, 0, 0], anchors)
    for i in range(len(zero_ious)):
        anchors_points = np.array(np.where(zero_ious[i] > -1))  # all points
        head = np.ones((1, anchors_points.shape[1])) * i
        anchors_points = np.concatenate((head, anchors_points), axis=0)
        anchors_pool = np.concatenate((anchors_pool, anchors_points), axis=-1)
    anchors_pool = anchors_pool.T.astype(np.int32)
    return anchors_pool
anchors_pool = get_anchors_pool()

def augment(img, json_data):
    aug_percent = 25 # the probability that img is not modified
    rand = np.random.randint(0, 100)
    if rand < aug_percent:
        return img, json_data

    org_h, org_w = img.shape[:2]
    shift_h = np.random.randint(-30, 31)
    shift_v = np.random.randint(-10, 11)
    enlarge = np.random.uniform(0.8, 1.2)
    rotate_angle = np.random.randint(-10, 11)

    # some images are too big, resize them to relatively small sizes
    # to save processing time
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        json_data['shapes'][i]['points'] = [[p[0]*IMG_W/org_w, p[1]*IMG_H/org_h] for p in points]
    img = cv2.resize(img, (IMG_W, IMG_H))

    pad = img.copy()
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        cv2.fillPoly(pad, [np.array(points, dtype=int)], (0, 0, 0))
    #pad_img = np.zeros((IMG_H*3, IMG_W*3, 3), dtype=np.uint8)
    pad = np.concatenate((pad, pad, pad), axis=1)
    pad_img = np.concatenate((pad, pad, pad), axis=0)
    pad_img[IMG_H:IMG_H*2,IMG_W:IMG_W*2] = img
    img = pad_img
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        json_data['shapes'][i]['points'] = [[p[0]+IMG_W, p[1]+IMG_H] for p in points]

    img_h, img_w = img.shape[:2]
    base = np.zeros(img.shape, dtype=np.uint8)
    if shift_h == 0:
        base = img
    elif shift_h > 0:
        base[::,shift_h:] = img[::,:-shift_h]
    else:
        base[::,:shift_h] = img[::,-shift_h:]
    img = base
    base = np.zeros(img.shape, dtype=np.uint8)
    if shift_v == 0:
        base = img
    elif shift_v > 0:
        base[shift_v:,::] = img[:-shift_v,::]
    else:
        base[:shift_v,::] = img[-shift_v:,::]
    img = base
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        json_data['shapes'][i]['points'] = [[p[0]+shift_h, p[1]+shift_v] for p in points]

    M = cv2.getRotationMatrix2D((img_w/2, img_h/2), rotate_angle, enlarge)
    img = cv2.warpAffine(img, M, (img_w, img_h))
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        json_data['shapes'][i]['points'] = [np.dot(M, [[p[0]], [p[1]], [1]]) for p in points]

    img = img[IMG_H:IMG_H*2,IMG_W:IMG_W*2]
    img_h, img_w = img.shape[:2]
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        json_data['shapes'][i]['points'] = [[p[0]-IMG_W, p[1]-IMG_H] for p in points]

    # hide the polygons that are out of the boundaries.
    for i in range(len(json_data['shapes'])):
        points = json_data['shapes'][i]['points']
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        lx, rx = min(xs), max(xs)
        ty, by = min(ys), max(ys)
        if lx < 0 or rx >= img_w or ty < 0 or by >= img_h:
            json_data['shapes'][i]['label'] = 'hide'

    new_img, new_json_data = img, json_data
    return new_img, new_json_data

def generator(batch_pairs):
    objs = []
    roi_samples = []
    imgs_targets = []
    imgs = []
    mask_qs = np.zeros([batch_size, classes_num, IMG_H//4, IMG_W//4], dtype=np.uint8)
    for img_idx, (img_path, json_path) in enumerate(batch_pairs):
        flip = random.randint(0, 1)
        img = cv2.imread(img_path)
        json_data = json.load(open(json_path, encoding='gbk'))
        img, json_data = augment(img, json_data)
        org_h, org_w = img.shape[:2]
        img = cv2.resize(img, (IMG_W, IMG_H))
        img_targets = [np.zeros(size+(anchor_base_n, classes_num+4+1+1)) for size in targets_sizes]
        overall_pos_samples, overall_neg_ol_samples = [], []
        obj_boxes = []
        if flip:
            img = img[::,::-1].copy() # if there's no .copy() here, the following fillPoly will give an error.
        imgs.append(img)
        for i in range(len(json_data['shapes'])):
            label = json_data['shapes'][i]['label']
            points = np.array(json_data['shapes'][i]['points'])
            point_qs = [[int(p[0]*IMG_W/(org_w*4)+0.5), int(p[1]*IMG_H/(org_h*4)+0.5)] for p in points]
            points = [[int(p[0]*IMG_W/org_w+0.5), int(p[1]*IMG_H/org_h+0.5)] for p in points]
            if flip:
                points = [[IMG_W-1-p[0], p[1]] for p in points]
                label = label.replace('_l_', '_r_') if '_l_' in label else label.replace('_r_', '_l_')
            if label == 'hide':
                cv2.fillPoly(img, [np.array(points, dtype=int)], (0, 0, 0))
                continue
            if 'cone' in label:
                for_i = 1
                intact = 'intactY' in label
                for_s = 1 if intact else 0
                stand = ('_s' in label) and for_s
                for_lr = intact and not stand
                l_r = 1 if ('_r_' in label) and for_lr else 0
                for_angle = for_lr
                angle = ((90-int(label.split('_')[-1])+360)%360 if l_r else 90+int(label.split('_')[-1]))//20 if for_angle else 0
            else:
                intact = 0
                for_i = 0
                stand, l_r, angle = 0, 0, 0
                for_s, for_lr, for_angle = 0, 0, 0
            cls_idx = classes.index(label.split('_')[0])
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(points, dtype=int)], (255,255,255))
            mask = mask[::,::,0]
            mask_q = np.zeros(mask_qs[0,0].shape+(3,), dtype=np.uint8)
            cv2.fillPoly(mask_q, [np.array(point_qs, dtype=int)], (255, 255, 255))
            mask_q = mask_q[::,::,0]
            mask_qs[img_idx,cls_idx] = np.maximum(mask_qs[img_idx,cls_idx], mask_q)
            obj_t, obj_b = min([p[1] for p in points]), max([p[1] for p in points])
            obj_l, obj_r = min([p[0] for p in points]), max([p[0] for p in points])
            obj_boxes.append([cls_idx, obj_t, obj_b, obj_l, obj_r])
            objs.append({'img_idx':img_idx, 'cls_idx':cls_idx, 't':obj_t, 'b':obj_b, 'l':obj_l, 'r':obj_r, \
                         'stand':stand, 'l_r':l_r, 'angle':angle, 'for_s':for_s, 'for_lr':for_lr, \
                         'for_angle':for_angle, 'for_i':for_i, 'intact':intact, 'mask':mask})
            obj_idx = len(objs) - 1

            anchors_ious, anchor_withins = get_ious_withins([obj_l, obj_r, obj_t, obj_b], anchors)
            anchor_layer_idx = np.argmax([ious.max() for ious in anchors_ious])
            anchor_coor = anchors_ious[anchor_layer_idx].argmax()
            anchor_coor_y, anchor_coor_x = divmod(anchor_coor, anchors_ious[anchor_layer_idx].shape[1] *
                                                    anchors_ious[anchor_layer_idx].shape[2])
            anchor_coor_x, anchor_coor_idx = divmod(anchor_coor_x, anchors_ious[anchor_layer_idx].shape[2])
            #print('best: ', anchors_ious[anchor_layer_idx][anchor_coor_y, anchor_coor_x, anchor_coor_idx])
            anchors_ious[anchor_layer_idx][anchor_coor_y, anchor_coor_x, anchor_coor_idx] = 1  # the best one
            pos_pool, reg_pool, neg_no_overlap_pool, neg_overlap_pool, roi_neg_overlap_pool = [np.zeros((4, 0))]*5
            neg_within_pool = np.zeros((4,0))
            for i in range(len(anchors_ious)):
                pos_points = np.array(np.where(anchors_ious[i] > iou_pos_sample_thresh))
                head = np.ones((1, pos_points.shape[1]))*i
                pos_points = np.concatenate((head, pos_points), axis=0)
                pos_pool = np.concatenate((pos_pool, pos_points), axis=-1)
                reg_points = np.array(np.where((anchors_ious[i] > iou_reg_sample_thresh)*(anchors_ious[i] <= iou_pos_sample_thresh)))
                head = np.ones((1, reg_points.shape[1]))*i
                reg_points = np.concatenate((head, reg_points), axis=0)
                reg_pool = np.concatenate((reg_pool, reg_points), axis=-1)

                neg_no_overlap_points = np.array(np.where(anchors_ious[i]==0))
                head = np.ones((1, neg_no_overlap_points.shape[1]))*i
                neg_no_overlap_points = np.concatenate((head, neg_no_overlap_points), axis=0)
                neg_no_overlap_pool = np.concatenate((neg_no_overlap_pool, neg_no_overlap_points), axis=1)

                neg_overlap_points = np.array(np.where((anchors_ious[i]>0)*(anchors_ious[i]<iou_neg_sample_thresh)))
                head = np.ones((1, neg_overlap_points.shape[1]))*i
                neg_overlap_points = np.concatenate((head, neg_overlap_points), axis=0)
                neg_overlap_pool = np.concatenate((neg_overlap_pool, neg_overlap_points), axis=1)

                neg_within_points = np.array(np.where((anchors_ious[i]<iou_neg_within_sample_thresh)*\
                (anchor_withins[i]>iou_within_sample_thresh)*(anchor_withins[i]<1))) # less than 1 to ensure anchor not totally inside the obj
                head = np.ones((1, neg_within_points.shape[1]))*i
                neg_within_points = np.concatenate((head, neg_within_points), axis=0)
                neg_within_pool = np.concatenate((neg_within_pool, neg_within_points), axis=1)

                roi_neg_overlap_points = np.array(np.where((anchors_ious[i] > 0) * (anchors_ious[i] < roi_iou_neg_thresh)))
                head = np.ones((1, roi_neg_overlap_points.shape[1])) * i
                roi_neg_overlap_points = np.concatenate((head, roi_neg_overlap_points), axis=0)
                roi_neg_overlap_pool = np.concatenate((roi_neg_overlap_pool, roi_neg_overlap_points), axis=1)
            pos_pool = pos_pool.T.astype(np.int32)
            neg_overlap_pool = neg_overlap_pool.T.astype(np.int32)
            neg_within_pool = neg_within_pool.T.astype(np.int32)
            reg_pool = reg_pool.T.astype(np.int32)
            roi_neg_overlap_pool = roi_neg_overlap_pool.T.astype(np.uint32)
            pos_samples = pos_pool

            for sample in pos_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx]\
                    [feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                s, e = fx + (fl - fx) * reg_range_d, fx + (fl - fx) * reg_range_u
                r_l = min(max((obj_l - s) / (e - s), 0), 1)
                s, e = fx + (fr - fx) * reg_range_d, fx + (fr - fx) * reg_range_u
                r_r = min(max((obj_r - s) / (e - s), 0), 1)
                s, e = fy + (ft - fy) * reg_range_d, fy + (ft - fy) * reg_range_u
                r_t = min(max((obj_t - s) / (e - s), 0), 1)
                s, e = fy + (fb - fy) * reg_range_d, fy + (fb - fy) * reg_range_u
                r_b = min(max((obj_b - s) / (e - s), 0), 1)
                # the first number is for knowing if it is an effective sample
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1] = 1 # the second is for objectiveness
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 2] = 1 # 2:2+classes_num are for classification
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx,
                2 + classes_num:2 + classes_num + 4] = np.array([r_l, r_r, r_t, r_b])
            reg_sample_num = min(reg_sample_quota_each_obj, len(reg_pool))
            reg_samples = random.sample(list(reg_pool), reg_sample_num)
            #reg_samples = reg_pool
            for sample in reg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                s, e = fx + (fl - fx) * reg_range_d, fx + (fl - fx) * reg_range_u
                r_l = min(max((obj_l - s) / (e - s), 0), 1)
                s, e = fx + (fr - fx) * reg_range_d, fx + (fr - fx) * reg_range_u
                r_r = min(max((obj_r - s) / (e - s), 0), 1)
                s, e = fy + (ft - fy) * reg_range_d, fy + (ft - fy) * reg_range_u
                r_t = min(max((obj_t - s) / (e - s), 0), 1)
                s, e = fy + (fb - fy) * reg_range_d, fy + (fb - fy) * reg_range_u
                r_b = min(max((obj_b - s) / (e - s), 0), 1)
                # the first number is for knowing if it is an effective sample
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 2  # one means it's positive or negative, 2 means reg samples
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 2] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx,
                2 + classes_num:2 + classes_num + 4] = np.array([r_l, r_r, r_t, r_b])
            neg_overlap_sample_num = min(neg_ol_sample_quota_each_obj, len(neg_overlap_pool))
            neg_overlap_samples = random.sample(list(neg_overlap_pool), neg_overlap_sample_num)
            neg_within_sample_num = min(neg_within_sample_quota_each_ojb, len(neg_within_pool))
            neg_within_samples = random.sample(list(neg_within_pool), neg_within_sample_num)
            overall_pos_samples += list(pos_samples)
            overall_neg_ol_samples += (neg_overlap_samples + neg_within_samples)
            # this sample is negative for one class, but probably neg for another classes,
            # that will be processed when all shapes are processed.
            # for the first stage, most negatives are eliminated, so for the second step, not so
            # many neg samples are needed
            for sample in np.concatenate((pos_pool, reg_pool)):
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                iou = anchors_ious[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx]
                if iou < roi_iou_reg_thresh:
                    continue
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                pos_neg, for_regression, for_pos_neg =1, 1, iou > roi_iou_pos_thresh
                if ft > 0 and fb < IMG_H and fb > ft and fl > 0 and fr < IMG_W and fr > fl:
                    roi_samples.append({'img_idx':img_idx, 'cls_idx':cls_idx, 'obj_idx':obj_idx, 'ft':ft, 'fb':fb, 'fl':fl, 'fr':fr,
                                        'pos_neg':pos_neg, 'stand':stand, 'l_r':l_r, 'angle':angle, 'for_reg':for_regression,
                                        'for_pos_neg':for_pos_neg, 'for_s':for_s, 'for_lr':for_lr, 'for_angle':for_angle,
                                        'for_i':for_i, 'intact':intact})
            roi_neg_samples = random.sample(list(roi_neg_overlap_pool), min(roi_neg_ol_sample_quota_each_obj, roi_neg_overlap_pool.shape[0]))
            for sample in roi_neg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                pos_neg, for_regression, for_pos_neg = 0, 0, 1
                stand, l_r, angle, for_s, for_l_r, for_angle = 0, 0, 0, 0, 0, 0
                if ft > 0 and fb < IMG_H and fb > ft and fl > 0 and fr < IMG_W and fr > fl:
                    roi_samples.append({'img_idx':img_idx, 'cls_idx':cls_idx, 'obj_idx':obj_idx, 'ft':ft, 'fb':fb, 'fl':fl, 'fr':fr,
                                        'pos_neg':pos_neg, 'stand':stand, 'l_r':l_r, 'angle':angle, 'for_reg':for_regression,
                                        'for_pos_neg':for_pos_neg, 'for_s':for_s, 'for_lr':for_lr, 'for_angle':for_angle,
                                        'for_i':for_i, 'intact':intact})

        # In fact, some of the neg_bg_samples are not neccessarily neg samples for some classes,
        # but they are neg samples for most classes with a high probability, the ones that are
        # not neg samples will be ignored. Don't be confused by the name.
        for sample in overall_neg_ol_samples:
            feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
            fl, fr, ft, fb, fx, fy = anchors[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx]
            max_iou = 0
            for obj_cls_idx, obj_t, obj_b, obj_l, obj_r in obj_boxes:
                iou = get_iou([obj_t, obj_b, obj_l, obj_r], [ft, fb, fl, fr])
                max_iou = max(max_iou, iou)
            if max_iou < roi_iou_neg_thresh:
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1] = 0
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 2:2+classes_num] = 0

        neg_bg_samples = random.sample(list(anchors_pool), min(neg_bg_sample_quota, len(anchors_pool)))
        for sample in neg_bg_samples:
            feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
            fl, fr, ft, fb, fx, fy = anchors[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx]
            max_iou = 0
            for obj_cls_idx, obj_t, obj_b, obj_l, obj_r in obj_boxes:
                iou = get_iou([obj_t, obj_b, obj_l, obj_r], [ft, fb, fl, fr])
                max_iou = max(max_iou, iou)
            if max_iou < roi_iou_neg_thresh:
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1] = 0
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 2:2+classes_num] = 0

        imgs_targets.append(img_targets)

    img_shape = imgs[0].shape
    imgs = np.concatenate(imgs, axis=0)
    imgs = imgs.reshape((-1,)+img_shape)
    return imgs, imgs_targets, objs, roi_samples, mask_qs
