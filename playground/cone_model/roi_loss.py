import torch

from roi_params import *
import numpy as np
import torch.nn.functional as F
from roi_tools import *

def dice_loss_function(predict, target, smooth=1):
    predict, target = predict.view(-1), target.view(-1)
    intersection = (predict * target).sum()
    dice = (2 * intersection + smooth) / (predict.sum() + target.sum() + smooth)
    return 1 - dice

def get_loss(outputs, targets, objs, tgt_mask_qs):
    pos_outputs, pos_targets = torch.zeros((0, classes_num + 4)).to(device), torch.zeros((0, classes_num + 4)).to(
        device)
    reg_outputs, reg_targets = torch.zeros((0, classes_num + 4)).to(device), torch.zeros((0, classes_num + 4)).to(
        device)
    neg_outputs, neg_targets = torch.zeros((0, classes_num)).to(device), torch.zeros((0, classes_num)).to(device)

    for img_idx in range(len(targets)):
        for anchor_layer_idx in range(len(targets[0])):
            output, target = outputs[anchor_layer_idx][img_idx], targets[img_idx][anchor_layer_idx]
            output = output.reshape(target.shape[:3]+(-1,))
            pos_prj = target[::,::,::,1:classes_num+1].max(axis=-1)
            pos_points = np.array(np.where(pos_prj>0)).T
            cats = np.argmax(target[::,::,::,1:classes_num+1], axis=-1)
            frms, tos = classes_num + cats * 4, classes_num + (cats + 1) * 4
            output_ = torch.zeros(target.shape).to(device)
            for i, j, k in pos_points:
                frm, to = frms[i, j, k], tos[i, j, k]
                output_[i, j, k, -4:] = output[i, j, k, frm:to]
            eff_points = np.array(np.where(target[::, ::, ::, 0] == 1)).T
            for i, j, k in eff_points:
                output_[i, j, k, 1:classes_num + 1] = output[i, j, k, :classes_num]
            # target[ign_points] = output_[ign_points].cpu().detach().numpy() # test which one is better
            output = output_
            pos_points = np.where((pos_prj == 1) * (target[::, ::, ::, 0] == 1))
            reg_points = np.where((pos_prj == 1) * (target[::, ::, ::, 0] == 2))
            neg_points = np.where((pos_prj != 1) * (target[::, ::, ::, 0] == 1))
            pos_output, pos_target = output[pos_points][::, 1:], target[pos_points][::, 1:]
            reg_output, reg_target = output[reg_points][::, 1:], target[reg_points][::, 1:]
            neg_output, neg_target = output[neg_points][::, 1:1 + classes_num], target[neg_points][::,
                                                                                1:1 + classes_num]
            pos_target = torch.from_numpy(pos_target).to(device).float()
            reg_target = torch.from_numpy(reg_target).to(device).float()
            neg_target = torch.from_numpy(neg_target).to(device).float()
            pos_outputs = torch.cat((pos_outputs, pos_output))
            pos_targets = torch.cat((pos_targets, pos_target))
            reg_outputs = torch.cat((reg_outputs, reg_output))
            reg_targets = torch.cat((reg_targets, reg_target))
            neg_outputs = torch.cat((neg_outputs, neg_output))
            neg_targets = torch.cat((neg_targets, neg_target))

    pos_detect_eff_points = np.where(pos_targets[::, :classes_num].detach().cpu().numpy() > -1)
    pos_detect_targets = pos_targets[::, :classes_num][pos_detect_eff_points]
    pos_detect_outputs = pos_outputs[::, :classes_num][pos_detect_eff_points]
    neg_detect_eff_points = np.where(neg_targets[::, :classes_num].detach().cpu().numpy() > -1)
    neg_detect_targets = neg_targets[::, :classes_num][neg_detect_eff_points]
    neg_detect_outputs = neg_outputs[::, :classes_num][neg_detect_eff_points]
    pos_regression_targets = pos_targets[::, classes_num:].reshape(-1)
    pos_regression_outputs = pos_outputs[::, classes_num:].reshape(-1)
    reg_regression_targets = reg_targets[::, classes_num:].reshape(-1)
    reg_regression_outputs = reg_outputs[::, classes_num:].reshape(-1)
    regression_targets = torch.cat((pos_regression_targets, reg_regression_targets), axis=0)
    regression_outputs = torch.cat((pos_regression_outputs, reg_regression_outputs), axis=0)
    pos_detect_loss = F.binary_cross_entropy(pos_detect_outputs, pos_detect_targets) if len(pos_detect_outputs) > 0 else torch.tensor(0)
    neg_detect_loss = F.binary_cross_entropy(neg_detect_outputs, neg_detect_targets)
    regression_loss = F.smooth_l1_loss(regression_outputs, regression_targets) if len(regression_outputs) > 0 else torch.tensor(0)

    roi_reg_infos = outputs[4]
    roi_reg_outs, roi_reg_tgts = [], []
    for roi_reg_info in roi_reg_infos:
        # froms and tos
        img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, \
        for_pos_neg, for_stand, for_lr, for_angle, reg = roi_reg_info
        if for_reg > 0.5:
            (ft, fb, fl, fr), (tt, tb, tl, tr) = from_box, to_box
            fcx, fcy = (fl + fr) / 2, (ft + fb) / 2
            s, e = (ft - fcy) * roi_reg_range_d + fcy, (ft - fcy) * roi_reg_range_u + fcy
            rt = min(max((tt - s) / (e - s), 0), 1)
            s, e = (fb - fcy) * roi_reg_range_d + fcy, (fb - fcy) * roi_reg_range_u + fcy
            rb = min(max((tb - s) / (e - s), 0), 1)
            s, e = (fl - fcx) * roi_reg_range_d + fcx, (fl - fcx) * roi_reg_range_u + fcx
            rl = min(max((tl - s) / (e - s), 0), 1)
            s, e = (fr - fcx) * roi_reg_range_d + fcx, (fr - fcx) * roi_reg_range_u + fcx
            rr = min(max((tr - s) / (e - s), 0), 1)
            '''
            iou = get_iou(from_box, to_box)
            print(iou)
            if iou < 0.0001:
                new_iou = get_iou([ft, fb, fl, fr], [tl, tr, tt, tb])
                print('new: ', new_iou)
            '''
            '''
            mmax, mmin = max([rt, rb, rl, rr]), min([rt, rb, rl, rr])
            if mmax > 0.9999 or mmin < 0.00001:
                print('here. ')
            '''
            roi_reg_outs.append(reg[:-4])
            roi_reg_tgts.append([rt, rb, rl, rr])
    if len(roi_reg_tgts) > 0:
        roi_reg_tgts = torch.tensor(roi_reg_tgts).reshape(-1).float().to(device)
        roi_reg_outs = torch.cat(roi_reg_outs)
        roi_reg_loss = F.smooth_l1_loss(roi_reg_outs, roi_reg_tgts)
    else:
        roi_reg_loss = torch.tensor(0)
    if len(roi_reg_infos) > 0:
        roi_pos_neg_outs = [info[-1][-4:-3] for info in roi_reg_infos if info[-5] > 0]  # info[-2] is for_pos_neg
        roi_pos_neg_tgts = [info[2] for info in roi_reg_infos if info[-5] > 0]
        roi_pos_neg_tgts = torch.FloatTensor(roi_pos_neg_tgts).float().to(device)
        roi_pos_neg_outs = torch.cat(roi_pos_neg_outs)
        roi_pos_neg_loss = F.binary_cross_entropy(roi_pos_neg_outs, roi_pos_neg_tgts)
    else:
        roi_pos_neg_loss = torch.tensor(0)

    roi_stand_outs = [info[-1][-3:-2] for info in roi_reg_infos if info[-4]==1]
    roi_stand_tgts = [info[5] for info in roi_reg_infos if info[-4]==1]
    if len(roi_stand_outs) == 0:
        roi_stand_loss = torch.tensor(0)
    else:
        roi_stand_outs = torch.cat(roi_stand_outs)
        roi_stand_tgts = torch.FloatTensor(roi_stand_tgts).float().to(device)
        roi_stand_loss = F.binary_cross_entropy(roi_stand_outs, roi_stand_tgts)\

    roi_lr_outs = [info[-1][-2:-1] for info in roi_reg_infos if info[-3]==1]
    roi_lr_tgts = [info[6] for info in roi_reg_infos if info[-3]==1]
    if len(roi_lr_outs) == 0:
        roi_lr_loss = torch.tensor(0)
    else:
        roi_lr_outs = torch.cat(roi_lr_outs)
        roi_lr_tgts = torch.FloatTensor(roi_lr_tgts).float().to(device)
        roi_lr_loss = F.binary_cross_entropy(roi_lr_outs, roi_lr_tgts)

    roi_angle_outs = [info[-1][-1:] for info in roi_reg_infos if info[-2]==1]
    roi_angle_tgts = [info[7] for info in roi_reg_infos if info[-2]==1]
    if len(roi_angle_outs) == 0:
        roi_angle_loss = torch.tensor(0)
    else:
        roi_angle_outs = torch.cat(roi_angle_outs)
        roi_angle_tgts = torch.FloatTensor(roi_angle_tgts).float().to(device)
        roi_angle_loss = F.smooth_l1_loss(roi_angle_outs, roi_angle_tgts)

    masks_loss, dice_loss = torch.tensor(0).to(device).float(), torch.tensor(0).to(device).float()
    segs_o = outputs[3]
    for seg_o, obj in zip(segs_o, objs):
        img_idx, cls_idx, ct, cb, cl, cr, mask_o = seg_o
        mask_t = obj[-1][ct:cb+1,cl:cr+1]
        mask_t = torch.tensor(mask_t/255).to(device).float()
        masks_loss += F.binary_cross_entropy(mask_o, mask_t)
        dice_loss += dice_loss_function(mask_o, mask_t)

    # This is for a better convergence, not used in test mode.
    mask_qs = outputs[5]
    tgt_mask_qs = torch.tensor(tgt_mask_qs/255).to(device).float()
    mask_qs_loss = F.binary_cross_entropy(mask_qs, tgt_mask_qs)

    return pos_detect_loss, neg_detect_loss, regression_loss, roi_reg_loss, roi_pos_neg_loss, \
           roi_stand_loss, roi_lr_loss, roi_angle_loss, masks_loss, dice_loss, mask_qs_loss
