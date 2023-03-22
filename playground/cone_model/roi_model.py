import random

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.transforms import Resize
from torchvision.ops import roi_align
from torchvision.transforms import Resize
from roi_params import *
from roi_NMS import NMS
import roi_tools

F_thick = (classes_num+4*classes_num)*anchor_base_n

class ResMimic(nn.Module):
    def __init__(self, n_in, n_out):
        super(ResMimic, self).__init__()
        self.conv1 = nn.Conv2d(n_in, n_in, 3, 1, padding=1)
        self.conv2 = nn.Conv2d(n_in, n_in, 3, 1, padding=1)
        self.conv3 = nn.Conv2d(n_in*2, n_out, 3, 1, padding=1)
    def forward(self, x):
        conv1 = F.relu(self.conv1(x))
        conv2 = F.relu(self.conv1(conv1))
        merge = torch.cat((x, conv2), 1)
        conv3 = self.conv3(merge)
        return conv3

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv_in = nn.Conv2d(n_ch[0], n_ch[1], 3, 1, padding=1)
        self.conv1a = ResMimic(n_ch[1], n_ch[1])
        self.conv1b = ResMimic(n_ch[1], n_ch[1])
        self.conv2a = ResMimic(n_ch[1], n_ch[2])
        self.conv2b = ResMimic(n_ch[2], n_ch[2])
        self.conv3a = ResMimic(n_ch[2], n_ch[3])
        self.conv3b = ResMimic(n_ch[3], n_ch[3])
        self.conv4a = ResMimic(n_ch[3], n_ch[4])
        self.conv4b = ResMimic(n_ch[4], n_ch[4])
        self.conv5a = ResMimic(n_ch[4], n_ch[5])
        self.conv5b = ResMimic(n_ch[5], n_ch[5])
        self.upsample6 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up6 = nn.Conv2d(n_ch[5], n_ch[4], 3, 1, padding = 1)
        self.conv6a = ResMimic(n_ch[4]*2, n_ch[4])
        self.conv6b = ResMimic(n_ch[4], n_ch[4])
        self.upsample7 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up7 = nn.Conv2d(n_ch[4], n_ch[3], 3, 1, padding=1)
        self.conv7a = ResMimic(n_ch[3]*2, n_ch[3])
        self.conv7b = ResMimic(n_ch[3], n_ch[3])
        self.conv7_ext = ResMimic(n_ch[3]*2, n_ch[3]) # Starts the roi branch
        self.conv_q = ResMimic(n_ch[3], n_ch[3]) # Starts the mask_q branch
        self.conv_mask_q = nn.Conv2d(n_ch[3], classes_num, 1, 1, padding=0)
        self.upsample8 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up8 = nn.Conv2d(n_ch[3], n_ch[2], 3, 1, padding=1)
        self.conv8 = ResMimic(n_ch[2]*2, n_ch[2])
        self.upsample9 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up9 = nn.Conv2d(n_ch[2], n_ch[1], 3, 1, padding=1)
        self.conv9 = ResMimic(n_ch[1]*2, n_ch[1])
        self.conv10 = nn.Conv2d(n_ch[1], classes_num, 1, 1, padding=0)

        # for detection outputs
        self.convF1a = nn.Conv2d(n_ch[5], n_ch[5], 3, 1, padding=1)
        self.convF1b = nn.Conv2d(n_ch[5], 100, 3, 1, padding=1)
        self.convF1c = nn.Conv2d(100, F_thick, 1, 1, padding=0)
        self.convF2a = nn.Conv2d(n_ch[4], n_ch[4], 3, 1, padding=1)
        self.convF2b = nn.Conv2d(n_ch[4], 100, 3, 1, padding=1)
        self.convF2c = nn.Conv2d(100, F_thick, 1, 1, padding=0)
        self.convF3a = nn.Conv2d(n_ch[3], n_ch[3], 3, 1, padding=1)
        self.convF3b = nn.Conv2d(n_ch[3], 100, 3, 1, padding=1)
        self.convF3c = nn.Conv2d(100, F_thick, 1, 1, padding=0)

        # for roi regression
        self.roi_conv1a = nn.Conv2d(n_ch[3], n_ch[3], 3, 1, padding=1)
        self.roi_conv1b = nn.Conv2d(n_ch[3], n_ch[3], 3, 1, padding=1)
        self.roi_conv2a = nn.Conv2d(n_ch[3], n_ch[2], 3, 1, padding=1)
        self.roi_conv2b = nn.Conv2d(n_ch[2], n_ch[2], 3, 1, padding=1)
        self.roi_conv3a = nn.Conv2d(n_ch[2], n_ch[1], 3, 1, padding=1)
        self.roi_fc1 = nn.Linear(n_ch[1]*(roi_align_h//4)*(roi_align_w//4), 200) # needs activation
        self.roi_fc2 = nn.Linear(200, classes_num+4*classes_num+3)

    def forward(self, imgs, mode='testing', targets = None, objs = None, roi_samples=None):
        def get_iou(box, obj):
            l, r, t, b = box[2:]
            tt, bb, ll, rr = obj[2:6]
            o_l, o_r, o_t, o_b = np.maximum(l, ll), np.minimum(r, rr), np.maximum(t, tt), np.minimum(b,
                                                                                                     bb)  # overlap lrtb
            ow, oh = np.maximum(0, o_r - o_l), np.maximum(0, o_b - o_t)
            area_of_overlap = ow * oh
            area_box, area_target = (r - l) * (b - t), (rr - ll) * (bb - tt)
            area_target = np.maximum(area_target, 1)
            area_of_union = area_box + area_target - area_of_overlap
            iou = area_of_overlap / area_of_union
            return iou
        def sub_segments(obj, mode):
            if mode == 'training':
                over_t, over_b, over_l, over_r = np.random.randint(0, 15, 4)
            else:
                over_t, over_b, over_l, over_r = 3, 3, 3, 3
            img_idx, cls_idx, org_t, org_b, org_l, org_r = obj[:6]
            t, b = min(max(org_t - over_t, 0), IMG_H - 1), min(max(org_b + over_b, 0), IMG_H - 1)
            l, r = min(max(org_l - over_l, 0), IMG_W - 1), min(max(org_r + over_r, 0), IMG_W - 1)
            qt, qb, ql, qr = (t + 2) // 4, (b + 2) // 4, (l + 2) // 4, (r + 2) // 4
            qt, qb = min(max(qt - 1, 0), IMG_H // 4 - 1), min(max(qb + 1, 0), IMG_H // 4 - 1)  # quater
            ql, qr = min(max(ql - 1, 0), IMG_W // 4 - 1), min(max(qr + 1, 0), IMG_W // 4 - 1)
            ht, hb, hl, hr = qt * 2, (qb + 1) * 2 - 1, ql * 2, (qr + 1) * 2 - 1  # half
            ct, cb, cl, cr = qt * 4, (qb + 1) * 4 - 1, ql * 4, (qr + 1) * 4 - 1  # current

            up8 = F.relu(self.up8(self.upsample8(conv7b[img_idx:img_idx + 1, ::, qt:qb + 1, ql:qr + 1])))
            merge8 = torch.cat((conv2b[img_idx:img_idx + 1, ::, ht:hb + 1, hl:hr + 1], up8), 1)
            conv8 = F.relu(self.conv8(merge8))

            up9 = F.relu(self.up9(self.upsample9(conv8)))
            merge9 = torch.cat((conv1b[img_idx:img_idx + 1, ::, ct:cb + 1, cl:cr + 1], up9), 1)
            conv9 = F.relu(self.conv9(merge9))
            conv10 = torch.sigmoid(self.conv10(conv9))
            mask = conv10.permute(0, 2, 3, 1)

            mask = mask[0, ::, ::, cls_idx]
            segment = [img_idx, cls_idx, ct, cb, cl, cr, mask]
            return segment
        conv_in = F.relu(self.conv_in(imgs))
        conv1a = F.relu(self.conv1a(conv_in))
        conv1b = F.relu(self.conv1b(conv1a))
        pool1 =F.max_pool2d(conv1b, 2)
        conv2a = F.relu(self.conv2a(pool1))
        conv2b = F.relu(self.conv2b(conv2a))
        pool2 =F.max_pool2d(conv2b, 2)
        conv3a = F.relu(self.conv3a(pool2))
        conv3b = F.relu(self.conv3b(conv3a))
        pool3 = F.max_pool2d(conv3b, 2)
        conv4a = F.relu(self.conv4a(pool3))
        conv4b = F.relu(self.conv4b(conv4a))
        pool4 = F.max_pool2d(conv4b, 2)
        conv5a = F.relu(self.conv5a(pool4))
        conv5b = F.relu(self.conv5b(conv5a))
        up6 = F.relu(self.up6(self.upsample6(conv5b)))
        merge6 = torch.cat((conv4b, up6), 1)
        conv6a = F.relu(self.conv6a(merge6))
        conv6b = F.relu(self.conv6b(conv6a))
        up7 = F.relu(self.up7(self.upsample7(conv6b)))
        merge7 = torch.cat((conv3b, up7), 1)
        conv7a = F.relu(self.conv7a(merge7))
        conv7b = F.relu(self.conv7b(conv7a))
        conv_q = F.relu(self.conv_q(conv7b))
        mask_qs = torch.sigmoid(self.conv_mask_q(conv_q))
        merge_7_q = torch.cat((conv7b, conv_q), 1)
        conv7_ext = F.relu(self.conv7_ext(merge_7_q))
        big_conv7 = Resize([IMG_H, IMG_W])(conv7_ext) # for roiAlign

        convF1a = F.relu(self.convF1a(conv5b))
        convF1b = F.relu(self.convF1b(convF1a))
        convF1c = torch.sigmoid(self.convF1c(convF1b))
        convF2a = F.relu(self.convF2a(conv6b))
        convF2b = F.relu(self.convF2b(convF2a))
        convF2c = torch.sigmoid(self.convF2c(convF2b))
        convF3a = F.relu(self.convF3a(conv7b))
        convF3b = F.relu(self.convF3b(convF3a))
        convF3c = torch.sigmoid(self.convF3c(convF3b))

        feature1 = convF1c.permute(0, 2, 3, 1)
        feature2 = convF2c.permute(0, 2, 3, 1)
        feature3 = convF3c.permute(0, 2, 3, 1)
        features = [f.cpu().detach().numpy() for f in [feature1, feature2, feature3]]
        box_cred_th_nms = box_cred_th_nms_train if mode == 'training' else box_cred_th_nms_test
        boxes = NMS(features, box_cred_th_nms, maximumn_obj_num=maximumn_obj_num)
        aligns = []
        roi_train_set = []

        roi_score_th = roi_score_th_train if mode == 'training' else roi_score_th_test
        if mode == 'training':
            '''
            for obj_idx, obj in enumerate(objs):
                img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, _ = obj
                to_box = [t, b, l, r]
                for box in boxes:
                    box_img_idx, box_cls_idx, bl, br, bt, bb = box
                    if img_idx == box_img_idx and cls_idx == box_cls_idx:
                        iou = get_iou(box, obj)
                        if iou > 0.5:
                            for_reg, for_pos_neg = 1, iou > 0.75
                            if bt > 0 and bb < IMG_H and bb > bt and bl > 0 and br < IMG_W and br > bl:
                                from_box = [bt, bb, bl, br]
                                pos_neg = 1
                                roi_train_set.append(
                                    [img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle])
            '''
            cls_pos_candidates, cls_neg_candidates = [[] for idx in range(classes_num)], [[] for idx in range(classes_num)]
            for box in boxes:
                box_img_idx, box_cls_idx, bl, br, bt, bb = box
                max_iou, max_idx = -1, -1
                bt, bb, bl, br = max(bt, 0), min(bb, IMG_H - 1), max(bl, 0), min(br, IMG_W - 1)
                if bt >= bb or bl >= br:
                    continue
                from_box = [bt, bb, bl, br]
                for obj_idx, obj in enumerate(objs):
                    img_idx, cls_idx = obj[:2]
                    iou = get_iou(box, obj) if img_idx == box_img_idx and cls_idx == box_cls_idx else 0
                    if max_iou < iou:
                        max_iou = iou
                        max_idx = obj_idx
                if max_iou > roi_iou_reg_thresh:
                    img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, _ = objs[max_idx]
                    to_box = [t, b, l, r]
                    for_reg, for_pos_neg = 1, max_iou > roi_iou_pos_thresh
                    pos_neg = 1
                    cls_pos_candidates[box_cls_idx].append(
                        [box_img_idx, box_cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg,
                         for_pos_neg, for_s, for_l_r, for_angle])
                elif max_iou < roi_iou_neg_thresh:  # the negative samples from proposals are quite important
                    for_reg, for_pos_neg = 0, 1
                    to_box = [0, 0, 0, 0] # in fact it's not used in this case
                    pos_neg = 0
                    stand, l_r, angle, for_s, for_l_r, for_angle = 0, 0, 0, 0, 0, 0
                    cls_neg_candidates[box_cls_idx].append([box_img_idx, box_cls_idx, pos_neg, from_box, to_box, stand, l_r, angle,
                                         for_reg, for_pos_neg, for_s, for_l_r, for_angle])
            # from the candidates select proposals
            cls_obj_nums = np.zeros(classes_num, dtype=np.int32) # the number of objects that fall into this category
            cls_max_num_pos_proposals = np.zeros(classes_num, dtype=np.int32)
            for obj in objs:
                cls_idx = obj[1]
                cls_obj_nums[cls_idx] += 1
            for cls_idx in range(classes_num):
                cls_max_num_pos_proposals[cls_idx] = max(roi_pos_nms_sample_quota_each_class, cls_obj_nums[cls_idx]*3)
            '''
            cls_pos_candidates = [[]*classes_num]
            for cd in pos_candidates:
                cls_idx = cd[1]
                cls_pos_candidates[cls_idx].append(cd)
            cls_neg_candidates = [[]*classes_num]
            for cd in neg_candidates:
                cls_idx = cd[1]
                cls_neg_candidates[cls_idx].append(cd)
            '''
            for cls_idx in range(classes_num):
                cls_pos_proposals = random.sample(cls_pos_candidates[cls_idx], \
                    min(cls_max_num_pos_proposals[cls_idx], len(cls_pos_candidates[cls_idx])))
                cls_neg_proposals = random.sample(cls_neg_candidates[cls_idx], \
                    min(len(cls_neg_candidates[cls_idx]), roi_neg_nms_sample_quota_each_class))
                roi_train_set += (cls_pos_proposals + cls_neg_proposals)
            #input samples
            for obj_idx, obj in enumerate(objs):
                img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, _ = obj
                to_box = [t, b, l, r]
                for sample in roi_samples:
                    smp_img_idx, smp_cls_idx, smp_obj_idx, bt, bb, bl, br, pos_neg, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle = sample
                    if img_idx == smp_img_idx and cls_idx == smp_cls_idx and obj_idx == smp_obj_idx:
                        from_box = [bt, bb, bl, br]
                        roi_train_set.append(
                            [img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg,
                             for_s, for_l_r, for_angle])
            for sample in roi_train_set:
                img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle = sample
                t, b, l, r = from_box
                cropped_2d = big_conv7[img_idx:img_idx + 1, ::, t:b, l:r]
                roiAlign = Resize([roi_align_h, roi_align_w])(cropped_2d)
                aligns.append(roiAlign)
            if len(aligns) == 0:
                aligns = torch.zeros([0, big_conv7.shape[1], roi_align_h, roi_align_w])
            else:
                aligns = torch.cat(aligns)
            cls_idcs = [rts[1] for rts in roi_train_set]
        else:
            cls_idcs = []
            for box in boxes:
                img_idx, cls_idx, l, r, t, b = box
                cropped_2d = big_conv7[img_idx:img_idx + 1, ::, t:b, l:r]
                roiAlign = Resize([roi_align_h, roi_align_w])(cropped_2d)
                aligns.append(roiAlign)
                cls_idcs.append(cls_idx)
            if len(aligns) == 0:
                aligns = torch.zeros([0, big_conv7.shape[1], roi_align_h, roi_align_w])
            else:
                aligns = torch.cat(aligns)

        # for roi regression
        # roi network is useful because it indeed makes regression more accurate
        # and it indeed eliminated the false positive proposals presented by NMS
        if len(cls_idcs) > 0:
            roi_conv1a = F.relu(self.roi_conv1a(aligns))
            roi_conv1b = F.relu(self.roi_conv1b(roi_conv1a))
            roi_pool1b = F.max_pool2d(roi_conv1b, 2)
            roi_conv2a = F.relu(self.roi_conv2a(roi_pool1b))
            roi_conv2b = F.relu(self.roi_conv2b(roi_conv2a))
            roi_pool2b = F.max_pool2d(roi_conv2b, 2)
            roi_conv3a = F.relu(self.roi_conv3a(roi_pool2b))
            roi_flat3a = torch.flatten(roi_conv3a, start_dim=1)
            roi_fc1 = F.relu(self.roi_fc1(roi_flat3a))
            roi_fc2 = torch.sigmoid(self.roi_fc2(roi_fc1))
            roi_fc2_ = roi_fc2[::,:-3].reshape(-1)
            roi_fc2_ = roi_fc2_.reshape(len(aligns), classes_num, -1)
            roi_regs = [reg[idx] for idx, reg in zip(cls_idcs, roi_fc2_)] # digits in roi_reg are [cls, 4 coors, s, l_r, angle]
            roi_regs = [torch.cat((reg,rear)) for reg, rear in zip(roi_regs, roi_fc2[::,-3:])]
        else:
            roi_regs = []

        if mode == 'training':
            segments = []  # segments in training mode and testing mode are different
            for idx in range(len(roi_train_set)):
                roi_train_set[idx].append(roi_regs[idx])
            roi_reg_infos = roi_train_set
            for obj in objs:
                img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, _ = obj
                segment = sub_segments(obj, mode='training')
                segments.append(segment)
        else:
            #segments = [[] for i in range(imgs.shape[0])]
            segments = []
            img_cls_scores = np.zeros((imgs.shape[0], classes_num))
            img_cls_box_idcs = np.zeros((imgs.shape[0], classes_num), dtype=int)
            reged_boxes = []
            for idx, ((tr, br, lr, rr, pos_neg, stand, l_r, angle), box) in enumerate(zip(roi_regs, boxes)):
                img_idx, cls_idx, l, r, t, b = box
                cx, cy = (l + r) / 2, (t + b) / 2
                tr, br, lr, rr = [xx.item() for xx in [tr, br, lr, rr]]
                s, e = (l - cx) * roi_reg_range_d + cx, (l - cx) * roi_reg_range_u + cx
                lll = s + (e - s) * lr
                s, e = (r - cx) * roi_reg_range_d + cx, (r - cx) * roi_reg_range_u + cx
                rrr = s + (e - s) * rr
                s, e = (t - cy) * roi_reg_range_d + cy, (t - cy) * roi_reg_range_u + cy
                ttt = s + (e - s) * tr
                s, e = (b - cy) * roi_reg_range_d + cy, (b - cy) * roi_reg_range_u + cy
                bbb = s + (e - s) * br
                lll, rrr, ttt, bbb = [int(xx + 0.5) for xx in [lll, rrr, ttt, bbb]]
                rrr, bbb = min(rrr, IMG_W-1), min(bbb, IMG_H-1)
                lll = min(max(0, lll), IMG_W - 2)
                rrr = min(max(lll + 1, rrr), IMG_W - 1)
                ttt = min(max(0, ttt), IMG_H - 2)
                bbb = min(max(ttt + 1, bbb), IMG_H - 1)
                if pos_neg > roi_score_th:
                    reged_boxes.append([img_idx, cls_idx, ttt, bbb, lll, rrr, stand, l_r, angle])

                '''
                if img_cls_scores[img_idx, cls_idx] < pos_neg:
                    img_cls_scores[img_idx, cls_idx] = pos_neg
                    img_cls_box_idcs[img_idx, cls_idx] = idx
                '''
            '''
            top_reged_boxes = []
            img_cls_scores = img_cls_scores.reshape(-1)
            img_cls_box_idcs = img_cls_box_idcs.reshape(-1)
            roi_score_th = 0.85
            for score, idx in zip(img_cls_scores, img_cls_box_idcs):
                if score > roi_score_th:
                    top_reged_boxes.append(reged_boxes[idx])
            '''
            for obj in reged_boxes:
                #img_idx, cls_idx, t, b, l, r = obj
                org_t, org_b, org_l, org_r = obj[2:6]
                segment = sub_segments(obj, mode='testing')
                img_idx, cls_idx, ct, cb, cl, cr, mask = segment
                stand, l_r, angle = obj[-3:]
                # for testing mode, the bounding boxes are original box, not aligned by 16,
                # masks are corresponding to bounding boxes
                mask = mask[org_t-ct:org_b+1-ct,org_l-cl:org_r+1-cl]
                segment = [img_idx, cls_idx, org_t, org_b, org_l, org_r, stand, l_r, angle, mask]
                #segments[img_idx].append(segment)
                segments.append(segment)

        # the contents of segments are different in training and testing mode
        output = [feature1, feature2, feature3, segments, roi_reg_infos, mask_qs] if mode == 'training' else [segments, mask_qs]
        return output
