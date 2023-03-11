
import os
import cv2
import json
from roi_tools import *
from roi_anchors import *
from roi_params import *
import random

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

def generator(batch_pairs):
    objs = []
    roi_samples = []
    imgs_targets = []
    imgs = []
    for img_idx, (img_path, json_path) in enumerate(batch_pairs):
        flip = random.randint(0, 1)
        img = cv2.imread(img_path)
        org_h, org_w = img.shape[:2]
        img = cv2.resize(img, (IMG_W, IMG_H))
        #cv2.imshow('img', img)
        #cv2.waitKey(0)
        json_data = json.load(open(json_path, encoding='gbk'))
        img_targets = [np.zeros(size+(anchor_base_n, classes_num+4+1)) for size in targets_sizes]
        overall_pos_samples, overall_neg_samples = [], []
        obj_boxes = []
        if flip:
            img = img[::,::-1].copy() # if there's no .copy() here, the following fillPoly will give an error.
        imgs.append(img)
        for i in range(len(json_data['shapes'])):
            label = json_data['shapes'][i]['label']
            #color = colors[0] if label.find('cone') != -1 else colors[1]
            points = np.array(json_data['shapes'][i]['points'])
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
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(points, dtype=int)], (255,255,255))
            mask = mask[::,::,0]
            obj_t, obj_b = min([p[1] for p in points]), max([p[1] for p in points])
            obj_l, obj_r = min([p[0] for p in points]), max([p[0] for p in points])
            cls_idx = classes.index(label.split('_')[0])
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
                pos_points = np.array(np.where(anchors_ious[i] > iou_pos_thresh))
                head = np.ones((1, pos_points.shape[1]))*i
                pos_points = np.concatenate((head, pos_points), axis=0)
                pos_pool = np.concatenate((pos_pool, pos_points), axis=-1)
                reg_points = np.array(np.where((anchors_ious[i] > iou_reg_thresh)*(anchors_ious[i] <= iou_pos_thresh)))
                head = np.ones((1, reg_points.shape[1]))*i
                reg_points = np.concatenate((head, reg_points), axis=0)
                reg_pool = np.concatenate((reg_pool, reg_points), axis=-1)
                neg_no_overlap_points = np.array(np.where(anchors_ious[i]==0))
                head = np.ones((1, neg_no_overlap_points.shape[1]))*i
                neg_no_overlap_points = np.concatenate((head, neg_no_overlap_points), axis=0)
                neg_no_overlap_pool = np.concatenate((neg_no_overlap_pool, neg_no_overlap_points), axis=1)
                neg_overlap_points = np.array(np.where((anchors_ious[i]>0)*(anchors_ious[i]<iou_neg_thresh)))
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
            neg_overlap_sample_num = min(int(neg_sample_quota_each_obj*neg_overlap_ratio), len(neg_overlap_pool))
            neg_no_overlap_sample_num = min(int(neg_sample_quota_each_obj*(1-neg_overlap_ratio)), len(neg_no_overlap_pool))
            neg_overlap_samples = random.sample(list(neg_overlap_pool), neg_overlap_sample_num)
            neg_no_overlap_samples = random.sample(list(neg_no_overlap_pool), neg_no_overlap_sample_num)
            neg_samples = neg_overlap_samples + neg_no_overlap_samples
            overall_pos_samples += list(pos_samples)
            overall_neg_samples += neg_samples
            for sample in neg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 0] = 1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, 1:classes_num + 1] = -1
                img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] = 0
            overall_samples = overall_pos_samples + overall_neg_samples

            '''
            print('%2d %2d %2d --- %3d %3d'%(len(pos_samples), len(reg_samples), len(neg_samples), obj_r-obj_l, obj_b-obj_t))
            show = img.copy()
            for sample in pos_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]
                fl, fr, ft, fb = max(fl, 0), min(fr, IMG_W - 1), max(ft, 0), min(fb, IMG_H - 1)
                show[ft, fl:fr] = np.array([0, 0, 255], dtype=np.uint8)
                show[fb, fl:fr] = np.array([0, 0, 255], dtype=np.uint8)
                show[ft:fb, fl] = np.array([0, 0, 255], dtype=np.uint8)
                show[ft:fb, fr] = np.array([0, 0, 255], dtype=np.uint8)
            for sample in reg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]
                fl, fr, ft, fb = max(fl, 0), min(fr, IMG_W - 1), max(ft, 0), min(fb, IMG_H - 1)
                show[ft, fl:fr] = np.array([0, 255, 0], dtype=np.uint8)
                show[fb, fl:fr] = np.array([0, 255, 0], dtype=np.uint8)
                show[ft:fb, fl] = np.array([0, 255, 0], dtype=np.uint8)
                show[ft:fb, fr] = np.array([0, 255, 0], dtype=np.uint8)
            for sample in neg_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                fl, fr, ft, fb, fx, fy = anchors[feature_idx][
                    feature_coor_y, feature_coor_x, feature_coor_idx]
                fl, fr, ft, fb = max(fl, 0), min(fr, IMG_W - 1), max(ft, 0), min(fb, IMG_H - 1)
                show[ft, fl:fr] = np.array([0, 0, 0], dtype=np.uint8)
                show[fb, fl:fr] = np.array([0, 0, 0], dtype=np.uint8)
                show[ft:fb, fl] = np.array([0, 0, 0], dtype=np.uint8)
                show[ft:fb, fr] = np.array([0, 0, 0], dtype=np.uint8)
            cv2.imshow('show', show)
            cv2.waitKey(200)
            print('here. ')
            for anchor in anchors:
                show = img.copy()
                for yy in range(anchor.shape[0]):
                    for xx in range(anchor.shape[1]):
                        for aa in range(anchor.shape[2]):
                            fl, fr, ft, fb, fx, fy = anchor[yy, xx, aa]
                            show[fy,fx] = np.array([255, 255, 255], dtype=np.uint8)
                yy, xx = anchor.shape[:2]
                yy, xx = yy//2, xx//2
                for aa in range(anchor[yy,xx].shape[0]):
                    fl, fr, ft, fb = anchor[yy, xx, aa,:4]
                    if fl > 0 and fr < IMG_W and ft > 0 and fb < IMG_H:
                        show[ft, fl:fr] = np.array([0, 0, 0], dtype=np.uint8)
                        show[fb, fl:fr] = np.array([0, 0, 0], dtype=np.uint8)
                        show[ft:fb, fl] = np.array([0, 0, 0], dtype=np.uint8)
                        show[ft:fb, fr] = np.array([0, 0, 0], dtype=np.uint8)
                cv2.imshow('show', show)
                cv2.waitKey(200)
                print('here. ')

            '''

            roi_pos_samples = pos_samples
            # for the first stage, most negatives are eliminated, so for the second step, not so
            # many neg samples are needed
            roi_neg_samples = random.sample(neg_samples, min(roi_neg_sample_quota_each_obj, len(neg_samples)))
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

        # This is for making sure that all the selected samples are correctly labeled if they are negative samples of a certain classes.
        for cls_idx, t, b, l, r in obj_boxes:
            for sample in overall_samples:
                feature_idx, feature_coor_y, feature_coor_x, feature_coor_idx = sample
                anchor = anchors[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx][:4]
                iou = get_iou([l, r, t, b], anchor)
                # sample pos samples' ious are smaller than threshold
                if iou < iou_neg_thresh and \
                        img_targets[feature_idx][
                            feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] == -1:
                    img_targets[feature_idx][feature_coor_y, feature_coor_x, feature_coor_idx, cls_idx + 1] = 0
        imgs_targets.append(img_targets)

    img_shape = imgs[0].shape
    imgs = np.concatenate(imgs, axis=0)
    imgs = imgs.reshape((-1,)+img_shape)
    '''
    print(flip)
    for img_idx, img in enumerate(imgs):
        cv2.imshow('img', img)
        for obj in objs:
            if obj[0] != img_idx:
                continue
            cls_idx = obj[1]
            colors = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
            ct, cb, cl, cr = obj[2:6]
            img[ct, cl:cr + 1] = colors[cls_idx]
            img[cb, cl:cr + 1] = colors[cls_idx]
            img[ct:cb + 1, cl] = colors[cls_idx]
            img[ct:cb + 1, cr] = colors[cls_idx]
            cv2.imshow('img', img)
            mask = obj[-1]
            cv2.imshow('mask', mask)
            cv2.waitKey(0)
    '''
    return imgs, imgs_targets, objs, roi_samples
