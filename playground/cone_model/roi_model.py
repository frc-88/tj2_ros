
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.transforms import Resize
from torchvision.ops import roi_align
from torchvision.transforms import Resize
from roi_params import *
from roi_NMS import NMS

F_thick = (classes_num+4*classes_num)*anchor_base_n

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv_in = nn.Conv2d(n_ch[0], n_ch[1], 3, 1, padding=1)
        self.conv1 = nn.Conv2d(n_ch[1], n_ch[1], 3, 1, padding=1)
        self.conv2 = nn.Conv2d(n_ch[1], n_ch[2], 3, 1, padding=1)
        self.conv3 = nn.Conv2d(n_ch[2], n_ch[3], 3, 1, padding=1)
        self.conv4 = nn.Conv2d(n_ch[3], n_ch[4], 3, 1, padding=1)
        self.conv5 = nn.Conv2d(n_ch[4], n_ch[5], 3, 1, padding=1)
        self.upsample6 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up6 = nn.Conv2d(n_ch[5], n_ch[4], 3, 1, padding = 1)
        self.conv6 = nn.Conv2d(n_ch[4]*2, n_ch[4], 3, 1, padding=1)
        self.upsample7 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up7 = nn.Conv2d(n_ch[4], n_ch[3], 3, 1, padding=1)
        self.conv7 = nn.Conv2d(n_ch[3]*2, n_ch[3], 3, 1, padding=1)
        self.upsample8 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up8 = nn.Conv2d(n_ch[3], n_ch[2], 3, 1, padding=1)
        self.conv8 = nn.Conv2d(n_ch[2]*2, n_ch[2], 3, 1, padding=1)
        self.upsample9 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.up9 = nn.Conv2d(n_ch[2], n_ch[1], 3, 1, padding=1)
        self.conv9 = nn.Conv2d(n_ch[1]*2, n_ch[1], 3, 1, padding=1)
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
            ll, rr, tt, bb = obj[2:6]
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
            img_idx, cls_idx, t, b, l, r = obj[:6]
            t, b = min(max(t - over_t, 0), IMG_H - 1), min(max(b + over_b, 0), IMG_H - 1)
            l, r = min(max(l - over_l, 0), IMG_W - 1), min(max(r + over_r, 0), IMG_W - 1)
            qt, qb, ql, qr = (t + 2) // 4, (b + 2) // 4, (l + 2) // 4, (r + 2) // 4
            qt, qb = min(max(qt - 1, 0), IMG_H // 4 - 1), min(max(qb + 1, 0), IMG_H // 4 - 1)  # quater
            ql, qr = min(max(ql - 1, 0), IMG_W // 4 - 1), min(max(qr + 1, 0), IMG_W // 4 - 1)
            ht, hb, hl, hr = qt * 2, (qb + 1) * 2 - 1, ql * 2, (qr + 1) * 2 - 1  # half
            ct, cb, cl, cr = qt * 4, (qb + 1) * 4 - 1, ql * 4, (qr + 1) * 4 - 1  # current

            up8 = F.relu(self.up8(self.upsample8(conv7[img_idx:img_idx + 1, ::, qt:qb + 1, ql:qr + 1])))
            merge8 = torch.cat((conv2[img_idx:img_idx + 1, ::, ht:hb + 1, hl:hr + 1], up8), 1)
            conv8 = F.relu(self.conv8(merge8))

            up9 = F.relu(self.up9(self.upsample9(conv8)))
            merge9 = torch.cat((conv1[img_idx:img_idx + 1, ::, ct:cb + 1, cl:cr + 1], up9), 1)
            conv9 = F.relu(self.conv9(merge9))
            conv10 = torch.sigmoid(self.conv10(conv9))
            mask = conv10.permute(0, 2, 3, 1)

            mask = mask[0, ::, ::, cls_idx]
            segment = [img_idx, cls_idx, ct, cb, cl, cr, mask]
            return segment
        conv_in = F.relu(self.conv_in(imgs))
        conv1 = F.relu(self.conv1(conv_in))
        pool1 =F.max_pool2d(conv1, 2)
        conv2 = F.relu(self.conv2(pool1))
        pool2 =F.max_pool2d(conv2, 2)
        conv3 = F.relu(self.conv3(pool2))
        pool3 = F.max_pool2d(conv3, 2)
        conv4 = F.relu(self.conv4(pool3))
        pool4 = F.max_pool2d(conv4, 2)
        conv5 = F.relu(self.conv5(pool4))
        up6 = F.relu(self.up6(self.upsample6(conv5)))
        merge6 = torch.cat((conv4, up6), 1)
        conv6 = F.relu(self.conv6(merge6))
        up7 = F.relu(self.up7(self.upsample7(conv6)))
        merge7 = torch.cat((conv3, up7), 1)
        conv7 = F.relu(self.conv7(merge7))

        convF1a = F.relu(self.convF1a(conv5))
        convF1b = F.relu(self.convF1b(convF1a))
        convF1c = torch.sigmoid(self.convF1c(convF1b))
        convF2a = F.relu(self.convF2a(conv6))
        convF2b = F.relu(self.convF2b(convF2a))
        convF2c = torch.sigmoid(self.convF2c(convF2b))
        convF3a = F.relu(self.convF3a(conv7))
        convF3b = F.relu(self.convF3b(convF3a))
        convF3c = torch.sigmoid(self.convF3c(convF3b))

        feature1 = convF1c.permute(0, 2, 3, 1)
        feature2 = convF2c.permute(0, 2, 3, 1)
        feature3 = convF3c.permute(0, 2, 3, 1)

        features = [f.cpu().detach().numpy() for f in [feature1, feature2, feature3]]
        boxes = NMS(features, box_cred_th_nms, maximumn_obj_num=maximumn_obj_num)
        aligns = []
        roi_train_set = []

        if mode == 'training':
            for obj in objs:
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
                for sample in roi_samples:
                    smp_img_idx, smp_cls_idx, bt, bb, bl, br, pos_neg, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle = sample
                    if img_idx == smp_img_idx and cls_idx == smp_cls_idx:
                        from_box = [bt, bb, bl, br]
                        roi_train_set.append(
                            [img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle])

            for sample in roi_train_set:
                img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle = sample
                t, b, l, r = from_box
                big_conv7 = Resize([IMG_H, IMG_W])(conv7)
                cropped_2d = big_conv7[img_idx:img_idx + 1, ::, t:b, l:r]
                roiAlign = Resize([roi_align_h, roi_align_w])(cropped_2d)
                aligns.append(roiAlign)
            aligns = torch.cat(aligns)
            cls_idcs = [rts[1] for rts in roi_train_set]
        else:
            cls_idcs = []
            for box in boxes:
                img_idx, cls_idx, l, r, t, b = box
                big_conv7 = Resize([IMG_H, IMG_W])(conv7)
                cropped_2d = big_conv7[img_idx:img_idx + 1, ::, t:b, l:r]
                roiAlign = Resize([roi_align_h, roi_align_w])(cropped_2d)
                aligns.append(roiAlign)
                cls_idcs.append(cls_idx)
            aligns = torch.cat(aligns)

        # for roi regression
        # roi network is useful because it indeed makes regression more accurate
        # and it indeed eliminated the false positive proposals presented by NMS
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

        '''
        roi_fc2_ = []
        if mode == 'training':
            for smp_idx in range(len(roi_train_set)):
                #img_idx, cls_idx, pos_neg, from_box, to_box, for_reg, for_pos_neg = roi_train_set[smp_idx]
                roi_fc2_.append(roi_fc2[smp_idx,:-3])
        else:
            for box_idx in range(len(boxes)):
                #img_idx, cls_idx, l, r, t, b = boxes[box_idx]
                roi_fc2_.append(roi_fc2[box_idx,:-3])
        '''
        roi_fc2_ = roi_fc2[::,:-3].reshape(-1)
        #roi_fc2_ = torch.cat(roi_fc2_, axis=-1)
        roi_fc2_ = roi_fc2_.reshape(len(aligns), classes_num, -1)
        roi_regs = [reg[idx] for idx, reg in zip(cls_idcs, roi_fc2_)] # digits in roi_reg are [cls, 4 coors, s, l_r, angle]
        roi_regs = [torch.cat((reg,rear)) for reg, rear in zip(roi_regs, roi_fc2[::,-3:])]

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
                s, e = (l - cx) * 0.5 + cx, (l - cx) * 1.5 + cx
                ll = s + (e - s) * lr
                s, e = (r - cx) * 0.5 + cx, (r - cx) * 1.5 + cx
                rr = s + (e - s) * rr
                s, e = (t - cy) * 0.5 + cy, (t - cy) * 1.5 + cy
                tt = s + (e - s) * tr
                s, e = (b - cy) * 0.5 + cy, (b - cy) * 1.5 + cy
                bb = s + (e - s) * br
                over_d = 10
                ll, rr, tt, bb = [int(xx + 0.5) for xx in [ll - over_d, rr + over_d, tt - over_d, bb + over_d]]
                ll = min(max(0, ll), IMG_W - 2)
                rr = min(max(ll + 1, rr), IMG_W - 1)
                tt = min(max(0, tt), IMG_H - 2)
                bb = min(max(tt + 1, bb), IMG_H - 1)
                if pos_neg > roi_score_th:
                    reged_boxes.append([img_idx, cls_idx, tt, bb, ll, rr, stand, l_r, angle])

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
                segment = sub_segments(obj, mode='testing')
                img_idx, cls_idx, ct, cb, cl, cr, mask = segment
                stand, l_r, angle = obj[-3:]
                segment = [img_idx, cls_idx, ct, cb, cl, cr, stand, l_r, angle, mask]
                #segments[img_idx].append(segment)
                segments.append(segment)

        # the contents of segments are different in training and testing mode
        output = [feature1, feature2, feature3, segments, roi_reg_infos] if mode == 'training' else segments
        return output
