import torch

from roi_params import *
import numpy as np
import torch.nn.functional as F
from roi_tools import *
from roi_anchors import *
import cv2

def dice_loss_function(predict, target, smooth=1):
    predict, target = predict.view(-1), target.view(-1)
    intersection = (predict * target).sum()
    dice = (2 * intersection + smooth) / (predict.sum() + target.sum() + smooth)
    return 1 - dice

def weighted_BCE(outs, tgts, weight_up=1.6, weight_down=0.4):
    outs, tgts = outs.reshape(-1), tgts.reshape(-1)
    outs_, tgts_ = outs.cpu().detach().numpy(), tgts.cpu().detach().numpy()
    diffs = abs(outs_-tgts_)
    idcs = [idx for idx in range(len(diffs))]
    Z = sorted(zip(diffs, idcs), reverse=True)
    diffs, idcs = zip(*Z)
    lenn = len(diffs)
    weights = [idx*(weight_down-weight_up)/lenn+weight_up for idx in range(lenn)]
    Z = sorted(zip(idcs, diffs, weights))
    idcs, diffs, weights = zip(*Z)
    weights = torch.tensor(weights).float().to(device)
    #loss = torch.nn.BCEWithLogitsLoss(weights)(outs, tgts)
    loss = torch.nn.BCELoss(weight=weights)(outs, tgts)
    return loss

# it is like focal loss, but not focal loss,
# in focal, alpha is a constant, but here, alpha depends on the number ratio of samples
# cannot change the sequence of outs and tgts for this function
# But need to know why it doesn't work, doesn't converge !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111
def balance_focal_BCE(outs, tgts, pos_neg_balance = False, gama = 2):
    assert(abs(tgts.sum().item() - len(torch.where(tgts>0.99)[0])) < 0.001) # helps confirm that the second arg is tgts, not outs
    outs, tgts = outs.reshape(-1), tgts.reshape(-1)
    outs_, tgts_ = outs.cpu().detach().numpy(), tgts.cpu().detach().numpy()
    diffs = abs(outs_-tgts_)
    r2s = diffs**gama  # This is for resolving the imbalance of difficulties
    #r_ave = r2s.sum()/len(r2s)
    #r2s = [r2/r_ave for r2 in r2s] if r_ave>0 else [1]*len(r2s)
    total_num = len(tgts)
    pos_num = int(tgts.sum())
    neg_num = total_num - pos_num
    if pos_num == total_num or neg_num == total_num or not pos_neg_balance:
        r1s = [1]*total_num
    else:
        deno = 2*pos_num*neg_num/(pos_num+neg_num)
        pos_r, neg_r = neg_num/deno, pos_num/deno
        r1s = [pos_r if tgt > 0.5 else neg_r for tgt in tgts]
    weights = [r1*r2 for r1, r2 in zip(r1s, r2s)]
    weights = torch.tensor(weights).float().to(device)
    loss = torch.nn.BCELoss(weight=weights)(outs, tgts)
    return loss

def focal_BCE(outs, tgts, pos_neg_balance = False, gama = 2):
    assert(abs(tgts.sum().item() - len(torch.where(tgts>0.99)[0])) < 0.001) # helps confirm that the second arg is tgts, not outs
    outs, tgts = outs.reshape(-1), tgts.reshape(-1)
    outs_, tgts_ = outs.cpu().detach().numpy(), tgts.cpu().detach().numpy()
    diffs = abs(outs_-tgts_)
    rs = diffs**gama  # This is for resolving the imbalance of difficulties
    total_num = len(tgts)
    pos_num = int(tgts.sum())
    neg_num = total_num - pos_num
    if pos_num == total_num or neg_num == total_num or not pos_neg_balance:
        weights = torch.tensor(rs).float().to(device)
        loss = torch.nn.BCELoss(weight=weights)(outs, tgts)
    else:
        weights = torch.tensor(rs).to(device).float()
        pos_points = torch.where(tgts>0.5)
        neg_points = torch.where(tgts<=0.5)
        pos_weights, neg_weights = weights[pos_points], weights[neg_points]
        pos_outs, neg_outs = outs[pos_points], outs[neg_points]
        pos_tgts, neg_tgts = tgts[pos_points], tgts[neg_points]
        # the case that both are 0 was excluded before entering the function
        if len(pos_tgts) == 0:
            loss = torch.nn.BCELoss(weight=neg_weights)(neg_outs, neg_tgts)
        elif len(neg_tgts) == 0:
            loss = torch.nn.BCELoss(weight=pos_weights)(pos_outs, pos_tgts)
        else:
            pos_loss = torch.nn.BCELoss(weight=pos_weights)(pos_outs, pos_tgts)
            neg_loss = torch.nn.BCELoss(weight=neg_weights)(neg_outs, neg_tgts)
            loss = (pos_loss+neg_loss)/2

    return loss

def pull_loss(outs, tgts):
    assert(abs(tgts.sum().item() - len(torch.where(tgts>0.99)[0])) < 0.001) # helps confirm that the second arg is tgts, not outs
    outs, tgts = outs.reshape(-1), tgts.reshape(-1)
    pos_points = torch.where(tgts > 0.5)
    loss1 = F.binary_cross_entropy(outs, tgts)
    if len(pos_points[0]) > 0:
        loss2 = F.binary_cross_entropy(outs[pos_points], tgts[pos_points])
        loss = (loss1+loss2)/2
    else:
        loss = loss1
    return loss

def get_loss(imgs, mode, outputs, targets, objs):
    if mode == 'training_stage1' or mode == 'training':
        pos_outputs, pos_targets = torch.zeros((0, 1+classes_num + 4)).to(device), torch.zeros((0, 1+classes_num + 4)).to(
            device)
        reg_outputs, reg_targets = torch.zeros((0, 1+classes_num + 4)).to(device), torch.zeros((0, 1+classes_num + 4)).to(
            device)
        neg_outputs, neg_targets = torch.zeros((0, 1)).to(device), torch.zeros((0, 1)).to(device)

        all_detect_loss_sum = torch.tensor(0).to(device).float()
        for img_idx in range(len(targets)):
            for anchor_layer_idx in range(len(targets[0])):
                output, target = outputs[anchor_layer_idx][img_idx], targets[img_idx][anchor_layer_idx]
                output = output.reshape(target.shape[:3] + (-1,))
                all_detect_loss = F.binary_cross_entropy(output[::,::,::,1], torch.tensor(target[::,::,::,1]).to(device).float())
                all_detect_loss_sum += all_detect_loss
                reg_points = np.array(np.where((np.maximum(target[::,::,::,0] == 1,target[::,::,::,0] == 2)))).T
                cats = np.argmax(target[::, ::, ::, 2:classes_num + 2], axis=-1)
                frms, tos = 1+classes_num + cats * 4, 1+classes_num + (cats + 1) * 4
                output_ = torch.zeros(target.shape).to(device)
                for i, j, k in reg_points:
                    frm, to = frms[i, j, k], tos[i, j, k]
                    output_[i, j, k, -4:] = output[i, j, k, frm:to]
                output_[::, ::, ::, 1:classes_num + 1 + 1] = output[::, ::, ::, :classes_num + 1]
                output = output_
                pos_points = np.where((target[::, ::, ::, 1] == 1) * (target[::, ::, ::, 0] == 1))
                reg_points = np.where((target[::, ::, ::, 1] == 1) * (target[::, ::, ::, 0] == 2))
                neg_points = np.where((target[::, ::, ::, 1] != 1) * (target[::, ::, ::, 0] == 1))
                pos_output, pos_target = output[pos_points][::, 1:], target[pos_points][::, 1:]
                reg_output, reg_target = output[reg_points][::, 1:], target[reg_points][::, 1:]
                neg_output, neg_target = output[neg_points][::, 1:2], target[neg_points][::,1:2]
                pos_target = torch.from_numpy(pos_target).to(device).float()
                reg_target = torch.from_numpy(reg_target).to(device).float()
                neg_target = torch.from_numpy(neg_target).to(device).float()
                pos_outputs = torch.cat((pos_outputs, pos_output))
                pos_targets = torch.cat((pos_targets, pos_target))
                reg_outputs = torch.cat((reg_outputs, reg_output))
                reg_targets = torch.cat((reg_targets, reg_target))
                neg_outputs = torch.cat((neg_outputs, neg_output))
                neg_targets = torch.cat((neg_targets, neg_target))
        all_detect_loss_sum /= (len(targets)*len(targets[0]))
        pos_detect_targets = pos_targets[::, 0]
        pos_detect_outputs = pos_outputs[::, 0]
        neg_detect_targets = neg_targets[::, 0]
        neg_detect_outputs = neg_outputs[::, 0]
        pos_regression_targets = pos_targets[::, 1+classes_num:].reshape(-1)
        pos_regression_outputs = pos_outputs[::, 1+classes_num:].reshape(-1)
        reg_regression_targets = reg_targets[::, 1+classes_num:].reshape(-1)
        reg_regression_outputs = reg_outputs[::, 1+classes_num:].reshape(-1)
        regression_targets = torch.cat((pos_regression_targets, reg_regression_targets), axis=0)
        regression_outputs = torch.cat((pos_regression_outputs, reg_regression_outputs), axis=0)
        # pos_detect_loss = F.binary_cross_entropy(pos_detect_outputs, pos_detect_targets) if len(pos_detect_outputs) > 0 else torch.tensor(0)
        # neg_detect_loss = F.binary_cross_entropy(neg_detect_outputs, neg_detect_targets)
        classification_targets = pos_targets[::,1:1+classes_num]
        classification_outputs = pos_outputs[::,1:1+classes_num]
        classification_loss = F.binary_cross_entropy(classification_outputs, classification_targets) if len(classification_targets) > 0 else torch.tensor(0)
        pos_detect_loss = focal_BCE(pos_detect_outputs, pos_detect_targets, pos_neg_balance=False) if len(pos_detect_outputs) > 0 else torch.tensor(0)
        neg_detect_loss = focal_BCE(neg_detect_outputs, neg_detect_targets, pos_neg_balance=False) if len(neg_detect_outputs) > 0 else torch.tensor(0)
        #pos_detect_loss, neg_detect_loss = masked_pos_loss_sum, masked_neg_loss_sum
        regression_loss = F.smooth_l1_loss(regression_outputs, regression_targets) if len(
            regression_outputs) > 0 else torch.tensor(0)

        masks_loss, dice_loss = torch.tensor(0).to(device).float(), torch.tensor(0).to(device).float()
        segs_o = outputs[3]
        for seg_o, obj in zip(segs_o, objs):
            img_idx, cls_idx, ct, cb, cl, cr, mask_o = seg_o
            mask_t = obj['mask'][ct:cb + 1, cl:cr + 1]
            mask_t = torch.tensor(mask_t / 255).to(device).float()
            masks_loss += F.binary_cross_entropy(mask_o, mask_t)
            dice_loss += dice_loss_function(mask_o, mask_t)
        masks_loss = masks_loss / len(segs_o) if len(segs_o) > 0 else masks_loss
        dice_loss = dice_loss / len(segs_o) if len(segs_o) > 0 else dice_loss

        if mode == 'training_stage1':
            roi_reg_loss, roi_pos_neg_loss, roi_intact_loss, roi_stand_loss, roi_angle_loss = [torch.tensor(0)] * 5
            return pos_detect_loss, neg_detect_loss, all_detect_loss_sum, classification_loss, regression_loss, roi_reg_loss, roi_pos_neg_loss, \
                       roi_intact_loss, roi_stand_loss, roi_angle_loss, masks_loss, dice_loss

    roi_reg_infos = outputs[4]
    roi_reg_outs, roi_reg_tgts = [], []
    for roi_reg_info in roi_reg_infos:
        # froms and tos
        if roi_reg_info['for_reg'] > 0.5:
            from_box, to_box = roi_reg_info['from_box'], roi_reg_info['to_box']
            ft, fb, fl, fr = from_box['t'], from_box['b'], from_box['l'], from_box['r']
            tt, tb, tl, tr = to_box['t'], to_box['b'], to_box['l'], to_box['r']
            fcx, fcy = (fl + fr) / 2, (ft + fb) / 2
            s, e = (ft - fcy) * roi_reg_range_d + fcy, (ft - fcy) * roi_reg_range_u + fcy
            rt = min(max((tt - s) / (e - s), 0), 1)
            s, e = (fb - fcy) * roi_reg_range_d + fcy, (fb - fcy) * roi_reg_range_u + fcy
            rb = min(max((tb - s) / (e - s), 0), 1)
            s, e = (fl - fcx) * roi_reg_range_d + fcx, (fl - fcx) * roi_reg_range_u + fcx
            rl = min(max((tl - s) / (e - s), 0), 1)
            s, e = (fr - fcx) * roi_reg_range_d + fcx, (fr - fcx) * roi_reg_range_u + fcx
            rr = min(max((tr - s) / (e - s), 0), 1)
            roi_reg_outs.append(roi_reg_info['crop_reg'][:4])
            roi_reg_tgts.append([rt, rb, rl, rr])
    if len(roi_reg_tgts) > 0:
        roi_reg_tgts = torch.tensor(roi_reg_tgts).reshape(-1).float().to(device)
        roi_reg_outs = torch.cat(roi_reg_outs)
        roi_reg_loss = F.smooth_l1_loss(roi_reg_outs, roi_reg_tgts)
    else:
        roi_reg_loss = torch.tensor(0)

    if len(roi_reg_infos) > 0:
        roi_pos_neg_outs = [info['crop_reg'][4:5] for info in roi_reg_infos if info['for_pos_neg'] > 0]  # info[-2] is for_pos_neg
        roi_pos_neg_tgts = [info['pos_neg'] for info in roi_reg_infos if info['for_pos_neg'] > 0]
        roi_pos_neg_tgts = torch.FloatTensor(roi_pos_neg_tgts).float().to(device)
        roi_pos_neg_outs = torch.cat(roi_pos_neg_outs)
        #roi_pos_neg_loss = focal_BCE(roi_pos_neg_outs, roi_pos_neg_tgts, pos_neg_balance=False)
        roi_pos_neg_loss = F.binary_cross_entropy(roi_pos_neg_outs, roi_pos_neg_tgts)
        #roi_pos_neg_loss = pull_loss(roi_pos_neg_outs, roi_pos_neg_tgts)
    else:
        roi_pos_neg_loss = torch.tensor(0)

    roi_intact_outs = [info['crop_reg'][5:6] for info in roi_reg_infos if info['for_i']==1]
    roi_intact_tgts = [info['intact'] for info in roi_reg_infos if info['for_i']==1]
    if len(roi_intact_outs) == 0:
        roi_intact_loss = torch.tensor(0)
    else:
        roi_stand_outs = torch.cat(roi_intact_outs)
        roi_stand_tgts = torch.FloatTensor(roi_intact_tgts).float().to(device)
        roi_intact_loss = focal_BCE(roi_stand_outs, roi_stand_tgts, pos_neg_balance=False)

    roi_stand_outs = [info['crop_reg'][6:7] for info in roi_reg_infos if info['for_s']==1]
    roi_stand_tgts = [info['stand'] for info in roi_reg_infos if info['for_s']==1]
    if len(roi_stand_outs) == 0:
        roi_stand_loss = torch.tensor(0)
    else:
        roi_stand_outs = torch.cat(roi_stand_outs)
        roi_stand_tgts = torch.FloatTensor(roi_stand_tgts).float().to(device)
        roi_stand_loss = focal_BCE(roi_stand_outs, roi_stand_tgts, pos_neg_balance=False)

    roi_angle_outs = [info['crop_reg'][7:] for info in roi_reg_infos if info['for_angle']==1]
    roi_angle_tgts = []
    for info in roi_reg_infos:
        if info['for_angle'] != 1:
            continue
        tgt = np.zeros(18)
        tgt[info['angle']] = 1
        roi_angle_tgts.append(tgt)
    if len(roi_angle_outs) == 0:
        roi_angle_loss = torch.tensor(0)
    else:
        roi_angle_outs = [out.reshape((-1,)+out.shape) for out in roi_angle_outs]
        roi_angle_outs = torch.cat(roi_angle_outs)
        roi_angle_tgts = torch.FloatTensor(np.array(roi_angle_tgts)).float().to(device)
        #roi_angle_loss = focal_BCE(roi_angle_outs, roi_angle_tgts, pos_neg_balance=False)
        roi_angle_loss = F.binary_cross_entropy(roi_angle_outs, roi_angle_tgts)
        #roi_angle_loss = pull_loss(roi_angle_outs, roi_angle_tgts)
    '''
    for img_idx, img in enumerate(imgs):
        for anchor_layer_idx in range(len(targets[0])):
            target = targets[img_idx][anchor_layer_idx]
            pos_prj = target[::,::,::,1:classes_num+1].max(axis=-1)
            pos_points = np.array(np.where((pos_prj == 1) * (target[::, ::, ::, 0] == 1))).T
            reg_points = np.array(np.where((pos_prj == 1) * (target[::, ::, ::, 0] == 2))).T
            neg_points = np.array(np.where((pos_prj != 1) * (target[::, ::, ::, 0] == 1))).T
            color = np.array([255, 0, 0], dtype=np.uint8)
            for i, j, k in pos_points:
                l, r, t, b = anchors[anchor_layer_idx][i, j, k, :4]
                t, b, l, r = max(t, 0), min(b, IMG_H-1), max(l, 0), min(r, IMG_W-1)
                img[t, l:r] = color
                img[b, l:r] = color
                img[t:b, l] = color
                img[t:b, r] = color
            color = np.array([0, 255, 0], dtype=np.uint8)
            for i, j, k in reg_points:
                l, r, t, b = anchors[anchor_layer_idx][i, j, k, :4]
                t, b, l, r = max(t, 0), min(b, IMG_H - 1), max(l, 0), min(r, IMG_W - 1)
                img[t, l:r] = color
                img[b, l:r] = color
                img[t:b, l] = color
                img[t:b, r] = color

            color = np.array([0, 0, 255], dtype=np.uint8)
            for i, j, k in neg_points:
                l, r, t, b = anchors[anchor_layer_idx][i, j, k, :4]
                t, b, l, r = max(t, 0), min(b, IMG_H - 1), max(l, 0), min(r, IMG_W - 1)
                img[t, l:r] = color
                img[b, l:r] = color
                img[t:b, l] = color
                img[t:b, r] = color

    roi_reg_infos = outputs[4]
    roi_reg_tgts = [], []
    for roi_reg_info in roi_reg_infos:
        # froms and tos
        img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, \
        for_pos_neg, for_stand, for_lr, for_angle, reg = roi_reg_info
        if for_pos_neg and pos_neg:
            color = np.array([255, 0, 0], dtype=np.uint8)
            t, b, l, r = from_box
            imgs[img_idx][t, l:r] = color
            imgs[img_idx][b, l:r] = color
            imgs[img_idx][t:b, l] = color
            imgs[img_idx][t:b, r] = color

        if for_reg and not for_pos_neg:
            np.array([0, 255, 0], dtype=np.uint8)
            t, b, l, r = from_box
            imgs[img_idx][t, l:r] = color
            imgs[img_idx][b, l:r] = color
            imgs[img_idx][t:b, l] = color
            imgs[img_idx][t:b, r] = color
        if for_pos_neg and not pos_neg:
            color = np.array([0, 0, 255], dtype=np.uint8)
            t, b, l, r = from_box
            imgs[img_idx][t, l:r] = color
            imgs[img_idx][b, l:r] = color
            imgs[img_idx][t:b, l] = color
            imgs[img_idx][t:b, r] = color

    for img in imgs:
        cv2.imshow('img', img)
        cv2.waitKey(500)
        print(img_idx)
    '''

    '''
    for img_idx, img in enumerate(imgs):
        for anchor_layer_idx in range(len(targets[0])):
            target = targets[img_idx][anchor_layer_idx]
            pos_prj = target[::,::,::,1]
            colors = [np.array([255, 0, 0], dtype=np.uint8), np.array([0, 0, 255], dtype=np.uint8)]
            for cls_idx in range(classes_num):
                points = np.array(np.where((target[::,::,::,0]>0)*(target[::,::,::,1+cls_idx]>0))).T
                for i, j, k in points:
                    l, r, t, b = anchors[anchor_layer_idx][i, j, k, :4]
                    t, b, l, r = max(t, 0), min(b, IMG_H-1), max(l, 0), min(r, IMG_W-1)
                    img[t, l:r] = colors[cls_idx]
                    img[b, l:r] = colors[cls_idx]
                    img[t:b, l] = colors[cls_idx]
                    img[t:b, r] = colors[cls_idx]
                points = np.array(np.where((target[::,::,::,0]>0)*(target[::,::,::,1+cls_idx]==0))).T
                for i, j, k in points:
                    l, r, t, b = anchors[anchor_layer_idx][i, j, k, :4]
                    t, b, l, r = max(t, 0), min(b, IMG_H-1), max(l, 0), min(r, IMG_W-1)
                    img[t, l:r] = np.array([0, 0, 0], dtype=np.uint8)
                    img[b, l:r] = np.array([0, 0, 0], dtype=np.uint8)
                    img[t:b, l] = np.array([0, 0, 0], dtype=np.uint8)
                    img[t:b, r] = np.array([0, 0, 0], dtype=np.uint8)

        cv2.imshow('img', img)
        cv2.waitKey(500)
        print(img_idx)
        '''

    if mode == 'training_stage2':
        pos_detect_loss, neg_detect_loss, all_detect_loss_sum, classification_loss, regression_loss, masks_loss, dice_loss = [torch.tensor(0)] * 7

    return pos_detect_loss, neg_detect_loss, all_detect_loss_sum, classification_loss, regression_loss, roi_reg_loss, roi_pos_neg_loss, roi_intact_loss,\
           roi_stand_loss, roi_angle_loss, masks_loss, dice_loss,
