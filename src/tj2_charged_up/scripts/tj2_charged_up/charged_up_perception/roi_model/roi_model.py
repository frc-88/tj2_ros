import random
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
from torchvision.transforms import Resize
from .roi_params import *
from .roi_NMS import NMS
import time

F_thick = (
    1 + classes_num + 4 * classes_num
) * anchor_base_n  # 1 for objectiveness, classes_num for classification the rest for coordinates


class ResMimic(nn.Module):
    def __init__(self, n_in, n_out):
        super(ResMimic, self).__init__()
        self.conv1 = nn.Conv2d(n_in, n_in, 3, 1, padding=1)
        self.conv2 = nn.Conv2d(n_in, n_in, 3, 1, padding=1)
        self.conv3 = nn.Conv2d(n_in * 2, n_out, 3, 1, padding=1)

    def forward(self, x):
        conv1 = F.relu(self.conv1(x))
        conv2 = F.relu(self.conv1(conv1))
        merge = torch.cat((x, conv2), 1)
        conv3 = self.conv3(merge)
        return conv3


class RegularConv2d(nn.Module):
    def __init__(self, n_in, n_out):
        super(RegularConv2d, self).__init__()
        self.conv1 = nn.Conv2d(n_in, n_out, 3, 1, padding=1)

    def forward(self, x):
        conv1 = self.conv1(x)
        return conv1


class DepthWise(nn.Module):
    def __init__(self, n_in, n_out):
        super(DepthWise, self).__init__()
        self.depthWise = nn.Conv2d(n_in, n_in, 3, 1, padding=1, groups=n_in)
        self.pointWise = nn.Conv2d(n_in, n_out, 1, 1, padding=0)

    def forward(self, x):
        depthWise = self.depthWise(x)
        pointWise = self.pointWise(depthWise)
        return pointWise


class ParallelConv2d(nn.Module):
    def __init__(self, n_in, n_out, kernel_size, stride, padding, activation):
        super(ParallelConv2d, self).__init__()
        self.convs = nn.ModuleList(
            [
                nn.Conv2d(
                    n_in, n_out, kernel_size=kernel_size, stride=stride, padding=padding
                )
                for i in range(classes_num)
            ]
        )
        self.activation = activation

    def forward(self, xs):
        outs = []
        if type(xs) is torch.Tensor:
            for conv in self.convs:
                out = self.activation(conv(xs))
                outs.append(out)
        else:
            for x, conv in zip(xs, self.convs):
                out = self.activation(conv(x))
                outs.append(out)
        return outs


class ParallelFC(nn.Module):
    def __init__(self, n_in, n_out, activation):
        super(ParallelFC, self).__init__()
        self.fcs = nn.ModuleList([nn.Linear(n_in, n_out) for i in range(classes_num)])
        self.activation = activation

    def forward(self, xs):
        outs = []
        if type(xs) is torch.Tensor:
            for fc in self.fcs:
                out = self.activation(fc(xs))
                outs.append(out)
        else:
            for x, fc in zip(xs, self.fcs):
                out = self.activation(fc(x))
                outs.append(out)
        return outs


BackboneConv = RegularConv2d


class Net(nn.Module):
    def __init__(self, test_time=False):
        super(Net, self).__init__()
        self.conv_in = nn.Conv2d(n_ch[0], n_ch[1], 3, 1, padding=1)
        self.conv1a = BackboneConv(n_ch[1], n_ch[1])
        self.conv1b = BackboneConv(n_ch[1], n_ch[1])
        self.conv2a = BackboneConv(n_ch[1], n_ch[2])
        self.conv2b = BackboneConv(n_ch[2], n_ch[2])
        self.conv3a = BackboneConv(n_ch[2], n_ch[3])
        self.conv3b = BackboneConv(n_ch[3], n_ch[3])
        self.conv4a = BackboneConv(n_ch[3], n_ch[4])
        self.conv4b = BackboneConv(n_ch[4], n_ch[4])
        self.conv5a = BackboneConv(n_ch[4], n_ch[5])
        self.conv5b = BackboneConv(n_ch[5], n_ch[5])
        self.upsample6 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up6 = nn.Conv2d(n_ch[5], n_ch[4], 3, 1, padding=1)
        self.conv6a = BackboneConv(n_ch[4] * 2, n_ch[4])
        self.upsample7 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up7 = nn.Conv2d(n_ch[4], n_ch[3], 3, 1, padding=1)
        self.conv7a = BackboneConv(n_ch[3] * 2, n_ch[3])
        self.upsample8 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up8 = nn.Conv2d(n_ch[3], n_ch[2], 3, 1, padding=1)
        self.conv8 = BackboneConv(n_ch[2] * 2, n_ch[2])
        self.upsample9 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up9 = nn.Conv2d(n_ch[2], n_ch[1], 3, 1, padding=1)
        self.conv9 = BackboneConv(n_ch[1] * 2, n_ch[1])
        self.conv10 = nn.Conv2d(n_ch[1], classes_num, 1, 1, padding=0)

        # for detection outputs
        self.convF1a = nn.Conv2d(n_ch[5], n_ch[5], 3, 1, padding=1)
        self.convF1b_objness = nn.Conv2d(n_ch[5], 100, 3, 1, padding=1)
        self.convF1c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF1b_paral_clses = ParallelConv2d(
            n_ch[5], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF1c_paral_clses = ParallelConv2d(
            100, 1 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )
        self.convF1b_paral_coors = ParallelConv2d(
            n_ch[5], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF1c_paral_coors = ParallelConv2d(
            100, 4 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )

        self.convF2a = nn.Conv2d(n_ch[4], n_ch[4], 3, 1, padding=1)
        self.convF2b_objness = nn.Conv2d(n_ch[4], 100, 3, 1, padding=1)
        self.convF2c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF2b_paral_clses = ParallelConv2d(
            n_ch[4], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF2c_paral_clses = ParallelConv2d(
            100, 1 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )
        self.convF2b_paral_coors = ParallelConv2d(
            n_ch[4], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF2c_paral_coors = ParallelConv2d(
            100, 4 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )

        self.convF3a = nn.Conv2d(n_ch[3], n_ch[3], 3, 1, padding=1)
        self.convF3b_objness = nn.Conv2d(n_ch[3], 100, 3, 1, padding=1)
        self.convF3c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF3b_paral_clses = ParallelConv2d(
            n_ch[3], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF3c_paral_clses = ParallelConv2d(
            100, 1 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )
        self.convF3b_paral_coors = ParallelConv2d(
            n_ch[3], 100, 3, 1, padding=1, activation=F.relu
        )
        self.convF3c_paral_coors = ParallelConv2d(
            100, 4 * anchor_base_n, 1, 1, padding=0, activation=torch.sigmoid
        )

        # for roi regression
        self.roi_conv_in = nn.Conv2d(
            n_roi_ch[0], n_roi_ch[1], 3, 1, padding=1
        )  # Starts the roi branch
        self.roi_conv1 = BackboneConv(n_roi_ch[1], n_roi_ch[2])
        self.roi_conv2 = BackboneConv(n_roi_ch[2], n_roi_ch[3])
        self.roi_branch_Y1 = BackboneConv(n_roi_ch[3], n_roi_ch[3])
        self.roi_branch_Y2 = BackboneConv(n_roi_ch[3], n_roi_ch[3])
        self.roi_fc1_Y1s = ParallelFC(
            n_roi_ch[3] * roi_align_h * roi_align_w // (4 * 4), 1000, activation=F.relu
        )
        self.roi_fc2_Y1s = ParallelFC(1000, 1 + 4, activation=torch.sigmoid)
        self.roi_fc1_Y2 = nn.Linear(
            n_roi_ch[3] * roi_align_h * roi_align_w // (4 * 4), 1000
        )
        self.roi_fc2_Y2 = nn.Linear(1000, 2 + 18)

        self.test_time = test_time

    def forward(self, imgs, mode="testing", objs=None, roi_samples=None):
        assert mode in [
            "training",
            "training_stage1",
            "training_stage2",
            "testing",
            "testing_stage1",
        ]

        def get_iou(box, obj):
            l, r, t, b = box[2:]
            tt, bb, ll, rr = obj["t"], obj["b"], obj["l"], obj["r"]
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
            iou = area_of_overlap / area_of_union
            return iou

        def sub_segments(obj, mode):
            if mode == "training":
                over_t, over_b, over_l, over_r = np.random.randint(0, 15, 4)
            else:
                over_t, over_b, over_l, over_r = 3, 3, 3, 3
            img_idx, cls_idx = obj["img_idx"], obj["cls_idx"]
            org_t, org_b, org_l, org_r = obj["t"], obj["b"], obj["l"], obj["r"]
            t, b = min(max(org_t - over_t, 0), IMG_H - 1), min(
                max(org_b + over_b, 0), IMG_H - 1
            )
            l, r = min(max(org_l - over_l, 0), IMG_W - 1), min(
                max(org_r + over_r, 0), IMG_W - 1
            )
            qt, qb, ql, qr = (t + 2) // 4, (b + 2) // 4, (l + 2) // 4, (r + 2) // 4
            qt, qb = min(max(qt - 1, 0), IMG_H // 4 - 1), min(
                max(qb + 1, 0), IMG_H // 4 - 1
            )  # quater
            ql, qr = min(max(ql - 1, 0), IMG_W // 4 - 1), min(
                max(qr + 1, 0), IMG_W // 4 - 1
            )
            ht, hb, hl, hr = qt * 2, (qb + 1) * 2 - 1, ql * 2, (qr + 1) * 2 - 1  # half
            ct, cb, cl, cr = (
                qt * 4,
                (qb + 1) * 4 - 1,
                ql * 4,
                (qr + 1) * 4 - 1,
            )  # current

            up8 = F.relu(
                self.up8(
                    self.upsample8(
                        conv7a[img_idx : img_idx + 1, ::, qt : qb + 1, ql : qr + 1]
                    )
                )
            )
            merge8 = torch.cat(
                (conv2b[img_idx : img_idx + 1, ::, ht : hb + 1, hl : hr + 1], up8), 1
            )
            conv8 = F.relu(self.conv8(merge8))

            up9 = F.relu(self.up9(self.upsample9(conv8)))
            merge9 = torch.cat(
                (conv1b[img_idx : img_idx + 1, ::, ct : cb + 1, cl : cr + 1], up9), 1
            )
            conv9 = F.relu(self.conv9(merge9))
            conv10 = torch.sigmoid(self.conv10(conv9))
            mask = conv10.permute(0, 2, 3, 1)

            mask = mask[0, ::, ::, cls_idx]
            segment = [img_idx, cls_idx, ct, cb, cl, cr, mask]
            return segment

        checkpoint_a = time.time()
        conv_in = F.relu(self.conv_in(imgs))
        conv1a = F.relu(self.conv1a(conv_in))
        conv1b = F.relu(self.conv1b(conv1a))
        pool1 = F.max_pool2d(conv1b, 2)
        conv2a = F.relu(self.conv2a(pool1))
        conv2b = F.relu(self.conv2b(conv2a))
        pool2 = F.max_pool2d(conv2b, 2)
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
        up7 = F.relu(self.up7(self.upsample7(conv6a)))
        merge7 = torch.cat((conv3b, up7), 1)
        conv7a = F.relu(self.conv7a(merge7))
        big_conv7 = Resize([IMG_H, IMG_W])(conv7a)  # for roiAlign

        convF1a = F.relu(self.convF1a(conv5b))
        convF1b_objness = F.relu(self.convF1b_objness(convF1a))
        convF1c_objness = torch.sigmoid(self.convF1c_objness(convF1b_objness))
        convF1b_paral_clses = self.convF1b_paral_clses(convF1a)
        convF1c_paral_clses = self.convF1c_paral_clses(convF1b_paral_clses)
        convF1b_paral_coors = self.convF1b_paral_coors(convF1a)
        convF1c_paral_coors = self.convF1c_paral_coors(convF1b_paral_coors)
        convF1c_clses = torch.cat(convF1c_paral_clses, axis=1)  # type: ignore
        convF1c_coors = torch.cat(convF1c_paral_coors, axis=1)  # type: ignore
        convF1c_objness = convF1c_objness.reshape(
            (convF1c_objness.shape[0], anchor_base_n, -1) + convF1c_objness.shape[-2:]
        )
        convF1c_clses = convF1c_clses.reshape(
            (convF1c_clses.shape[0], anchor_base_n, -1) + convF1c_clses.shape[-2:]
        )
        convF1c_coors = convF1c_coors.reshape(
            (convF1c_coors.shape[0], anchor_base_n, -1) + convF1c_coors.shape[-2:]
        )
        convF1c = torch.cat((convF1c_objness, convF1c_clses, convF1c_coors), axis=2)  # type: ignore
        convF1c = convF1c.reshape((convF1c.shape[0], -1) + convF1c.shape[-2:])

        convF2a = F.relu(self.convF2a(conv6a))
        convF2b_objness = F.relu(self.convF2b_objness(convF2a))
        convF2c_objness = torch.sigmoid(self.convF2c_objness(convF2b_objness))
        convF2b_paral_clses = self.convF2b_paral_clses(convF2a)
        convF2c_paral_clses = self.convF2c_paral_clses(convF2b_paral_clses)
        convF2b_paral_coors = self.convF2b_paral_coors(convF2a)
        convF2c_paral_coors = self.convF2c_paral_coors(convF2b_paral_coors)
        convF2c_clses = torch.cat(convF2c_paral_clses, axis=1)  # type: ignore
        convF2c_coors = torch.cat(convF2c_paral_coors, axis=1)  # type: ignore
        convF2c_objness = convF2c_objness.reshape(
            (convF2c_objness.shape[0], anchor_base_n, -1) + convF2c_objness.shape[-2:]
        )
        convF2c_clses = convF2c_clses.reshape(
            (convF2c_clses.shape[0], anchor_base_n, -1) + convF2c_clses.shape[-2:]
        )
        convF2c_coors = convF2c_coors.reshape(
            (convF2c_coors.shape[0], anchor_base_n, -1) + convF2c_coors.shape[-2:]
        )
        convF2c = torch.cat((convF2c_objness, convF2c_clses, convF2c_coors), axis=2)  # type: ignore
        convF2c = convF2c.reshape((convF2c.shape[0], -1) + convF2c.shape[-2:])

        convF3a = F.relu(self.convF3a(conv7a))
        convF3b_objness = F.relu(self.convF3b_objness(convF3a))
        convF3c_objness = torch.sigmoid(self.convF3c_objness(convF3b_objness))
        convF3b_paral_clses = self.convF3b_paral_clses(convF3a)
        convF3c_paral_clses = self.convF3c_paral_clses(convF3b_paral_clses)
        convF3b_paral_coors = self.convF3b_paral_coors(convF3a)
        convF3c_paral_coors = self.convF3c_paral_coors(convF3b_paral_coors)
        convF3c_clses = torch.cat(convF3c_paral_clses, axis=1)  # type: ignore
        convF3c_coors = torch.cat(convF3c_paral_coors, axis=1)  # type: ignore
        convF3c_objness = convF3c_objness.reshape(
            (convF3c_objness.shape[0], anchor_base_n, -1) + convF3c_objness.shape[-2:]
        )
        convF3c_clses = convF3c_clses.reshape(
            (convF3c_clses.shape[0], anchor_base_n, -1) + convF3c_clses.shape[-2:]
        )
        convF3c_coors = convF3c_coors.reshape(
            (convF3c_coors.shape[0], anchor_base_n, -1) + convF3c_coors.shape[-2:]
        )
        convF3c = torch.cat((convF3c_objness, convF3c_clses, convF3c_coors), axis=2)  # type: ignore
        convF3c = convF3c.reshape((convF3c.shape[0], -1) + convF3c.shape[-2:])
        feature1 = convF1c.permute(0, 2, 3, 1)
        feature2 = convF2c.permute(0, 2, 3, 1)
        feature3 = convF3c.permute(0, 2, 3, 1)
        segments = []
        if "training" in mode:
            assert objs is not None
            for obj in objs:
                segment = sub_segments(obj, mode="training")
                segments.append(segment)

        roi_reg_infos = None
        if mode == "training_stage1":
            return [feature1, feature2, feature3, segments, roi_reg_infos]
        if "testing" in mode and self.test_time:
            checkpoint_b = time.time()
            print("checkpoint A time: %.5fs" % (checkpoint_b - checkpoint_a))
            checkpoint_a = checkpoint_b
        features = [f.cpu().detach().numpy() for f in [feature1, feature2, feature3]]
        box_cred_th_nms = (
            box_cred_th_nms_train if "training" in mode else box_cred_th_nms_test
        )
        cls_cred_th_nms = (
            box_cred_cls_confi_th_nms_train
            if "training" in mode
            else box_cred_cls_confi_th_nms_test
        )
        boxes = NMS(
            features,
            box_cred_th_nms,
            cls_cred_th_nms,
            maximumn_obj_num=maximumn_obj_num,
        )
        roi_train_set = []
        aligns = []
        if mode == "testing_stage1":
            segments = []
            for box in boxes:
                box_img_idx, box_cls_idx, bl, br, bt, bb = box
                intact, stand, angle = (
                    torch.tensor(0).to(device),
                    torch.tensor(1).to(device),
                    torch.tensor(0).to(device),
                )
                mask = np.zeros((bb + 1 - bt, br - bl + 1))
                segments.append(
                    [
                        box_img_idx,
                        box_cls_idx,
                        bt,
                        bb,
                        bl,
                        br,
                        intact,
                        stand,
                        angle,
                        mask,
                    ]
                )
            return segments
        if "testing" in mode and self.test_time:
            checkpoint_b = time.time()
            print("checkpoint B time: %.5fs" % (checkpoint_b - checkpoint_a))
            checkpoint_a = checkpoint_b
        if len(boxes) > 0:
            aspect_ratios = []
            for box in boxes:
                l, r, t, b = box[2:]
                w, h = max(1, r - l + 1), max(1, b - t + 1)
                mmax, mmin = max(w, h), min(w, h)
                aspect_ratios.append(mmax / mmin)
            aspect_ratios = np.array(aspect_ratios)
            boxes = boxes[np.where(aspect_ratios < 4.5)]

        # if function is not returned now, it is doing the whole training or whole testing
        if "training" in mode:
            assert objs is not None
            cls_pos_candidates, cls_neg_candidates = [
                [] for idx in range(classes_num)
            ], [[] for idx in range(classes_num)]
            for box in boxes:
                box_img_idx, box_cls_idx, bl, br, bt, bb = box
                max_iou, max_idx = -1, -1
                bt, bb, bl, br = (
                    max(bt, 0),
                    min(bb, IMG_H - 1),
                    max(bl, 0),
                    min(br, IMG_W - 1),
                )
                if bt >= bb or bl >= br:
                    continue
                from_box = {"t": bt, "b": bb, "l": bl, "r": br}
                for obj_idx, obj in enumerate(objs):
                    iou = (
                        get_iou(box, obj)
                        if obj["img_idx"] == box_img_idx
                        and obj["cls_idx"] == box_cls_idx
                        else 0
                    )
                    if max_iou < iou:
                        max_iou = iou
                        max_idx = obj_idx
                if max_iou > roi_iou_reg_thresh:
                    # img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, _ = objs[max_idx]
                    obj = objs[max_idx]
                    # for_seg = 0
                    to_box = {
                        "t": obj["t"],
                        "b": obj["b"],
                        "l": obj["l"],
                        "r": obj["r"],
                    }
                    for_reg, for_pos_neg = 1, max_iou > roi_iou_pos_thresh
                    pos_neg = 1
                    cls_pos_candidates[box_cls_idx].append(
                        {
                            "img_idx": box_img_idx,
                            "cls_idx": box_cls_idx,
                            "pos_neg": pos_neg,
                            "from_box": from_box,
                            "to_box": to_box,
                            "intact": obj["intact"],
                            "stand": obj["stand"],
                            "l_r": obj["l_r"],
                            "angle": obj["angle"],
                            "for_reg": for_reg,
                            "for_pos_neg": for_pos_neg,
                            "for_i": obj["for_i"],
                            "for_s": obj["for_s"],
                            "for_lr": obj["for_lr"],
                            "for_angle": obj["for_angle"],
                        }
                    )
                elif (
                    max_iou < roi_iou_neg_thresh
                ):  # the negative samples from proposals are quite important
                    for_reg, for_pos_neg = 0, 1
                    to_box = {
                        "t": 0,
                        "b": 0,
                        "l": 0,
                        "r": 0,
                    }  # in fact it's not used in this case
                    pos_neg = 0
                    intact, stand, l_r, angle, for_s, for_lr, for_angle, for_i = (
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    cls_neg_candidates[box_cls_idx].append(
                        {
                            "img_idx": box_img_idx,
                            "cls_idx": box_cls_idx,
                            "pos_neg": pos_neg,
                            "from_box": from_box,
                            "to_box": to_box,
                            "intact": intact,
                            "stand": stand,
                            "l_r": l_r,
                            "angle": angle,
                            "for_reg": for_reg,
                            "for_pos_neg": for_pos_neg,
                            "for_i": for_i,
                            "for_s": for_s,
                            "for_lr": for_lr,
                            "for_angle": for_angle,
                        }
                    )
            # from the candidates select proposals
            cls_obj_nums = np.zeros(
                classes_num, dtype=np.int32
            )  # the number of objects that fall into this category
            cls_max_num_pos_proposals = np.zeros(classes_num, dtype=np.int32)
            for obj in objs:
                cls_idx = obj["cls_idx"]
                cls_obj_nums[cls_idx] += 1
            for cls_idx in range(classes_num):
                cls_max_num_pos_proposals[cls_idx] = max(
                    roi_pos_nms_sample_quota_each_class, cls_obj_nums[cls_idx] * 3
                )
            for cls_idx in range(classes_num):
                cls_pos_proposals = random.sample(
                    cls_pos_candidates[cls_idx],
                    min(
                        cls_max_num_pos_proposals[cls_idx],
                        len(cls_pos_candidates[cls_idx]),
                    ),
                )
                cls_neg_proposals = random.sample(
                    cls_neg_candidates[cls_idx],
                    min(
                        len(cls_neg_candidates[cls_idx]),
                        roi_neg_nms_sample_quota_each_class,
                    ),
                )
                roi_train_set += cls_pos_proposals + cls_neg_proposals
            # input samples
            assert objs is not None
            assert roi_samples is not None
            for obj_idx, obj in enumerate(objs):
                # img_idx, cls_idx, t, b, l, r, stand, l_r, angle, for_s, for_l_r, for_angle, obj_mask = obj
                to_box = {"t": obj["t"], "b": obj["b"], "l": obj["l"], "r": obj["r"]}
                for sample in roi_samples:
                    # smp_img_idx, smp_cls_idx, smp_obj_idx, bt, bb, bl, br, pos_neg, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle, for_seg, mask = sample
                    if (
                        obj["img_idx"] == sample["img_idx"]
                        and obj["cls_idx"] == sample["cls_idx"]
                        and obj_idx == sample["obj_idx"]
                    ):
                        from_box = {
                            "t": sample["ft"],
                            "b": sample["fb"],
                            "l": sample["fl"],
                            "r": sample["fr"],
                        }
                        roi_train_set.append(
                            {
                                "img_idx": sample["img_idx"],
                                "cls_idx": sample["cls_idx"],
                                "pos_neg": sample["pos_neg"],
                                "from_box": from_box,
                                "to_box": to_box,
                                "intact": sample["intact"],
                                "stand": sample["stand"],
                                "l_r": sample["l_r"],
                                "angle": sample["angle"],
                                "for_reg": sample["for_reg"],
                                "for_pos_neg": sample["for_pos_neg"],
                                "for_i": sample["for_i"],
                                "for_s": sample["for_s"],
                                "for_lr": sample["for_lr"],
                                "for_angle": sample["for_angle"],
                            }
                        )
            for sample in roi_train_set:
                # img_idx, cls_idx, pos_neg, from_box, to_box, stand, l_r, angle, for_reg, for_pos_neg, for_s, for_l_r, for_angle, for_seg, mask = sample
                t, b, l, r = (
                    sample["from_box"]["t"],
                    sample["from_box"]["b"],
                    sample["from_box"]["l"],
                    sample["from_box"]["r"],
                )
                img_idx = sample["img_idx"]
                cropped_2d = big_conv7[img_idx : img_idx + 1, ::, t : b + 1, l : r + 1]
                roiAlign = Resize([roi_align_h, roi_align_w])(cropped_2d)
                aligns.append(roiAlign)
            cls_idcs = [rts["cls_idx"] for rts in roi_train_set]
        else:
            cls_idcs = []
            for box in boxes:
                img_idx, cls_idx, l, r, t, b = box
                cropped_2d = big_conv7[img_idx : img_idx + 1, ::, t : b + 1, l : r + 1]
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
            roi_conv_in = F.relu(self.roi_conv_in(aligns))
            roi_conv1 = F.relu(self.roi_conv1(roi_conv_in))
            roi_pool1 = F.max_pool2d(roi_conv1, 2)
            roi_conv2 = F.relu(self.roi_conv2(roi_pool1))
            roi_pool2 = F.max_pool2d(roi_conv2, 2)
            roi_branch_Y1 = F.relu(self.roi_branch_Y1(roi_pool2))
            roi_branch_Y2 = F.relu(self.roi_branch_Y2(roi_pool2))
            roi_flat_Y1 = torch.flatten(roi_branch_Y1, start_dim=1)
            roi_flat_Y2 = torch.flatten(roi_branch_Y2, start_dim=1)
            roi_fc1_Y1s = self.roi_fc1_Y1s(roi_flat_Y1)
            roi_fc2_Y1s = self.roi_fc2_Y1s(roi_fc1_Y1s)
            roi_fc2_clses = torch.cat([fc[::, :1] for fc in roi_fc2_Y1s], axis=1)  # type: ignore
            roi_fc2_coors = torch.cat([fc[::, 1:] for fc in roi_fc2_Y1s], axis=1)  # type: ignore
            roi_fc2_Y1 = torch.cat((roi_fc2_clses, roi_fc2_coors), axis=1)  # type: ignore
            roi_fc1_Y2 = F.relu(self.roi_fc1_Y2(roi_flat_Y2))
            roi_fc2_Y2 = torch.sigmoid(self.roi_fc2_Y2(roi_fc1_Y2))
            roi_detect = roi_fc2_Y1.reshape(len(aligns), classes_num, -1)
            roi_regs = [
                reg[idx] for idx, reg in zip(cls_idcs, roi_detect)
            ]  # digits in crop_reg are [cls, 4 coors, intact, s]
            roi_regs = [
                torch.cat((reg, rear)) for reg, rear in zip(roi_regs, roi_fc2_Y2)
            ]
        else:
            roi_regs = []

        segments = []
        if "training" in mode:
            for idx in range(len(roi_train_set)):
                roi_train_set[idx]["crop_reg"] = roi_regs[idx]
            roi_reg_infos = roi_train_set
        else:
            reged_boxes = []
            for idx, (reg, box) in enumerate(zip(roi_regs, boxes)):
                tr, br, lr, rr, pos_neg, intact, stand = reg[:7]
                angle_code = reg[7:]
                angle_l_code = torch.cat((angle_code[1:], angle_code[:1]))
                angle_r_code = torch.cat((angle_code[-1:], angle_code[:-1]))
                angle_code = angle_code + 0.5 * angle_l_code + 0.5 * angle_r_code
                angle = torch.argmax(angle_code)  # filtering of the output
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
                rrr, bbb = min(rrr, IMG_W - 1), min(bbb, IMG_H - 1)
                lll = min(max(0, lll), IMG_W - 2)
                rrr = min(max(lll + 1, rrr), IMG_W - 1)
                ttt = min(max(0, ttt), IMG_H - 2)
                bbb = min(max(ttt + 1, bbb), IMG_H - 1)
                if pos_neg > roi_score_th:
                    reged_boxes.append(
                        [
                            img_idx,
                            cls_idx,
                            ttt,
                            bbb,
                            lll,
                            rrr,
                            intact,
                            stand,
                            angle * 20,
                        ]
                    )
            for reged_box in reged_boxes:
                img_idx, cls_idx, org_t, org_b, org_l, org_r = reged_box[:6]
                obj = {
                    "img_idx": img_idx,
                    "cls_idx": cls_idx,
                    "t": org_t,
                    "b": org_b,
                    "l": org_l,
                    "r": org_r,
                }
                segment = sub_segments(obj, mode="testing")
                img_idx, cls_idx, ct, cb, cl, cr, mask = segment
                intact, stand, angle = reged_box[-3:]
                # for testing mode, the bounding boxes are original box, not aligned by 16,
                # masks are corresponding to bounding boxes
                mask = (
                    mask[org_t - ct : org_b + 1 - ct, org_l - cl : org_r + 1 - cl]
                    .cpu()
                    .detach()
                    .numpy()
                )
                segment = [
                    img_idx,
                    cls_idx,
                    org_t,
                    org_b,
                    org_l,
                    org_r,
                    intact,
                    stand,
                    angle,
                    mask,
                ]
                segments.append(segment)
        if "testing" in mode and self.test_time:
            checkpoint_b = time.time()
            print("checkpoint C time: %.5fs" % (checkpoint_b - checkpoint_a))
            checkpoint_a = checkpoint_b
        # the contents of segments are different in training and testing mode
        output = (
            [feature1, feature2, feature3, segments, roi_reg_infos]
            if "training" in mode
            else segments
        )
        return output
