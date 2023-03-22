
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
    zero_ious = get_ious([0, 0, 0, 0], anchors)
    for i in range(len(zero_ious)):
        anchors_points = np.array(np.where(zero_ious[i] > -1))  # all points
        head = np.ones((1, anchors_points.shape[1])) * i
        anchors_points = np.concatenate((head, anchors_points), axis=0)
        anchors_pool = np.concatenate((anchors_pool, anchors_points), axis=-1)
    anchors_pool = anchors_pool.T.astype(np.int32)
    return anchors_pool

anchors_pool = get_anchors_pool()

def generate_fake_samples(img, json_data):
    bound_overd = 40
    imh, imw = img.shape[:2]
    for i in range(len(json_data['shapes'])):
        label = json_data['shapes'][i]['label']
        if 'hide' in label:
            continue
        points = np.array(json_data['shapes'][i]['points'])
        shape_l, shape_t = [int(v) for v in points.min(axis=0)]
        shape_r, shape_b = [int(v) for v in points.max(axis=0)]
        shape_w, shape_h = shape_r - shape_l + 1, shape_b - shape_t + 1
        # don't process the objects that are touching boundaries of the images
        if shape_l < bound_overd and shape_r > imw - bound_overd and shape_t < bound_overd and shape_b > imh - bound_overd:
            continue
        org_x, org_y = (shape_l+shape_r)//2, (shape_t+shape_b)//2
        #new_x, new_y = random.randint(shape_w//2+10, imw-shape_w//2-10), random.randint(shape_h//2+10, imh-shape_h//2-10)
        new_x, new_y = imw//2, imh//2
        off_x, off_y = new_x-org_x, new_y-org_y
        org_mask = np.zeros(img.shape, dtype=np.uint8)
        cv2.fillPoly(org_mask, [np.array(points, dtype=int)], (255,255,255))
        new_points = [[p[0]+off_x, p[1]+off_y] for p in points]
        new_mask = np.zeros(img.shape, dtype=np.uint8)
        cv2.fillPoly(new_mask, [np.array(new_points, dtype=int)], (255,255,255))
        org_mask, new_mask = org_mask[::,::,0], new_mask[::,::,0]
        try:
            img[np.where(new_mask > 0)] = img[np.where(org_mask > 0)]
        except:
            print('error')
        else:
            None


    cv2.imshow('img', img)
    cv2.waitKey(500)
    print('here. ')


def generator(batch_pairs):
    objs = []
    roi_samples = []
    imgs_targets = []
    imgs = []
    mask_qs = np.zeros([batch_size, classes_num, IMG_H//4, IMG_W//4], dtype=np.uint8)
    for img_idx, (img_path, json_path) in enumerate(batch_pairs):
        flip = random.randint(0, 1)
        img = cv2.imread(img_path)
        org_h, org_w = img.shape[:2]
        json_data = json.load(open(json_path, encoding='gbk'))
        #generate_fake_samples(img, json_data)
        img = cv2.resize(img, (IMG_W, IMG_H))
        img_targets = [np.zeros(size+(anchor_base_n, classes_num+4+1)) for size in targets_sizes]
        overall_pos_samples, overall_neg_ol_samples = [], []
        obj_boxes = []
        if flip:
            img = img[::,::-1].copy() # if there's no .copy() here, the following fillPoly will give an error.
        imgs.append(img)
        for i in range(len(json_data['shapes'])):
            label = json_data['shapes'][i]['label']
            #color = colors[0] if label.find('cone') != -1 else colors[1]
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
                stand = '_s' in label
                l_r = 0 if stand else '_r_' in label
                angle = 0 if stand else int(label.split('_')[-1])/180
                for_s = 1
                for_angle = for_l_r = 1 if stand == 0 else 0
            else:
                stand, l_r, angle = 0, 0, 0
                for_s, for_l_r, for_angle = 0, 0, 0
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
            objs.append([img_idx, cls_idx, obj_t, obj_b, obj_l, obj_r, stand, l_r, angle, for_s, for_l_r, for_angle, mask])
            obj_idx = len(objs) - 1

            anchors_ious = get_ious([obj_l, obj_r, obj_t, obj_b], anchors)
            anchor_layer_idx = np.argmax([ious.max() for ious in anchors_ious])
            anchor_coor = anchors_ious[anchor_layer_idx].argmax()
            anchor_coor_y, anchor_coor_x = divmod(anchor_coor, anchors_ious[anchor_layer_idx].shape[1] *
                                                    anchors_ious[anchor_layer_idx].shape[2])
            anchor_coor_x, anchor_coor_idx = divmod(anchor_coor_x, anchors_ious[anchor_layer_idx].shape[2])
            #print('best: ', anchors_ious[anchor_layer_idx][anchor_coor_y, anchor_coor_x, anchor_coor_idx])
            anchors_ious[anchor_layer_idx][anchor_coor_y, anchor_coor_x, anchor_coor_idx] = 1  # the best one
            pos_pool, reg_pool, neg_no_overlap_pool, neg_overlap_pool = np.zeros((4, 0)), np.zeros((4, 0)), np.zeros((4, 0)), np.zeros((4, 0))
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
            pos_pool = pos_pool.T.astype(np.int32)
            neg_no_overlap_pool = neg_no_overlap_pool.T.astype(np.int32)
            neg_overlap_pool = neg_overlap_pool.T.astype(np.int32)
            reg_pool = reg_pool.T.astype(np.int32)
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
                # this sample is positive for one class, and very likely negative for other classes, but ignored
                # for that case, don't worry, that will be processed when all shapes are processed.
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1:classes_num + 1] = -1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx,
                1 + classes_num:1 + classes_num + 4] = np.array([r_l, r_r, r_t, r_b])
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
                img_targets[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 2  # one means it's positive or negative, 2 means reg samples
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1:classes_num + 1] = -1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx,
                1 + classes_num:1 + classes_num + 4] = np.array([r_l, r_r, r_t, r_b])
            neg_overlap_sample_num = min(neg_ol_sample_quota_each_obj, len(neg_overlap_pool))
            #neg_no_overlap_sample_num = min(int(neg_sample_quota_each_obj*(1-neg_overlap_ratio)), len(neg_no_overlap_pool))
            neg_overlap_samples = random.sample(list(neg_overlap_pool), neg_overlap_sample_num)
            #neg_no_overlap_samples = random.sample(list(neg_no_overlap_pool), neg_no_overlap_sample_num)
            #neg_samples = neg_overlap_samples + neg_no_overlap_samples
            overall_pos_samples += list(pos_samples)
            overall_neg_ol_samples += neg_overlap_samples
            # this sample is negative for one class, but probably neg for another classes,
            # that will be processed when all shapes are processed.
            for sample in neg_overlap_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1:classes_num + 1] = -1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] = 0

            roi_pos_samples = pos_samples
            # for the first stage, most negatives are eliminated, so for the second step, not so
            # many neg samples are needed
            roi_neg_samples = random.sample(neg_overlap_samples, min(roi_neg_ol_sample_quota_each_obj, len(neg_overlap_samples)))
            for sample in roi_pos_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                pos_neg, for_regression, for_pos_neg =1, 1, 1
                if ft > 0 and fb < IMG_H and fb > ft and fl > 0 and fr < IMG_W and fr > fl:
                    roi_samples.append([img_idx, cls_idx, obj_idx, ft, fb, fl, fr, pos_neg, stand, l_r, angle, for_regression, for_pos_neg, for_s, for_l_r, for_angle])
            if len(pos_samples) > 0:  # it means there are positive samples
                pos_neg, for_regression, for_pos_neg = 1, 0, 1
                if obj_t > 0 and obj_b < IMG_H and obj_b > obj_t and obj_l > 0 and obj_r < IMG_W and obj_r > obj_l:
                    roi_samples.append([img_idx, cls_idx, obj_idx, obj_t, obj_b, obj_l, obj_r, pos_neg, stand, l_r, angle, for_regression,
                                        for_pos_neg, for_s, for_l_r, for_angle])  # this is the best sample, must be used
            for sample in roi_neg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]  # feature lrtbxy
                pos_neg, for_regression, for_pos_neg = 0, 0, 1
                stand, l_r, angle, for_s, for_l_r, for_angle = 0, 0, 0, 0, 0, 0
                if ft > 0 and fb < IMG_H and fb > ft and fl > 0 and fr < IMG_W and fr > fl:
                    roi_samples.append([img_idx, cls_idx, obj_idx, ft, fb, fl, fr, pos_neg, stand, l_r, angle, for_regression, for_pos_neg, for_s, for_l_r, for_angle])


        # In fact, some of the neg_bg_samples are not neccessarily neg samples for some classes,
        # but they are neg samples for most classes with a high probability, the ones that are
        # not negsamples will be ignored. Don't be confused by the name.
        neg_bg_samples = random.sample(list(anchors_pool), neg_bg_sample_quota)
        for sample in neg_bg_samples:
            feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
            img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
            # all classes are ignored, but don't worry, later the code will process the negative samples
            img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1:classes_num + 1] = -1

        # Here we can add the code that collect some negative roi samples that have no overlap with the corresponding
        # samples, but the code will be a little complicated, and I think that for the first stage the model can
        # get rid of almost all the negative samples that has no overlap with the classes, so I don't do that now.
        #roi_neg_bg_samples = random.sample(neg_bg_samples, min(len(neg_bg_samples), roi_neg_bg_sample_quota))

        # This is for making sure that all the selected samples are correctly labeled if they are negative samples of a certain classes.
        for cls_idx in range(classes_num):
            max_ious = get_ious([0,0,0,0], anchors)
            for box_cls_idx, box_t, box_b, box_l, box_r in obj_boxes:
                if cls_idx == box_cls_idx:
                    ious = get_ious([box_l, box_r, box_t, box_b], anchors)
                    for layer_idx in range(len(ious)):
                        max_ious[layer_idx] = np.maximum(max_ious[layer_idx], ious[layer_idx])
            for layer_idx in range(len(max_ious)):
                img_targets[layer_idx][::,::,::,cls_idx+1][np.where(max_ious[layer_idx]<iou_neg_sample_thresh)] = 0

        imgs_targets.append(img_targets)

    img_shape = imgs[0].shape
    imgs = np.concatenate(imgs, axis=0)
    imgs = imgs.reshape((-1,)+img_shape)
    return imgs, imgs_targets, objs, roi_samples, mask_qs
