import torch
import torch.nn as nn
from torch.nn.functional import relu
from .components import RegularConv2d, ParallelConv2d, ParallelFC
from .parameters import ChargedUpPerceptionParameters


class Net(nn.Module):
    def __init__(self, parameters: ChargedUpPerceptionParameters, test_time=False):
        super(Net, self).__init__()
        self.model_parameters = parameters
        n_ch = parameters.n_ch
        classes_num = parameters.classes_num
        anchor_base_n = parameters.classes_num
        n_roi_ch = parameters.n_roi_ch
        roi_align_h = parameters.roi_align_h
        roi_align_w = parameters.roi_align_w

        self.conv_in = nn.Conv2d(n_ch[0], n_ch[1], 3, 1, padding=1)
        self.conv1a = RegularConv2d(n_ch[1], n_ch[1])
        self.conv1b = RegularConv2d(n_ch[1], n_ch[1])
        self.conv2a = RegularConv2d(n_ch[1], n_ch[2])
        self.conv2b = RegularConv2d(n_ch[2], n_ch[2])
        self.conv3a = RegularConv2d(n_ch[2], n_ch[3])
        self.conv3b = RegularConv2d(n_ch[3], n_ch[3])
        self.conv4a = RegularConv2d(n_ch[3], n_ch[4])
        self.conv4b = RegularConv2d(n_ch[4], n_ch[4])
        self.conv5a = RegularConv2d(n_ch[4], n_ch[5])
        self.conv5b = RegularConv2d(n_ch[5], n_ch[5])
        self.upsample6 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up6 = nn.Conv2d(n_ch[5], n_ch[4], 3, 1, padding=1)
        self.conv6a = RegularConv2d(n_ch[4] * 2, n_ch[4])
        self.upsample7 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up7 = nn.Conv2d(n_ch[4], n_ch[3], 3, 1, padding=1)
        self.conv7a = RegularConv2d(n_ch[3] * 2, n_ch[3])
        self.upsample8 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up8 = nn.Conv2d(n_ch[3], n_ch[2], 3, 1, padding=1)
        self.conv8 = RegularConv2d(n_ch[2] * 2, n_ch[2])
        self.upsample9 = nn.Upsample(
            scale_factor=2, mode="bilinear", align_corners=True
        )
        self.up9 = nn.Conv2d(n_ch[2], n_ch[1], 3, 1, padding=1)
        self.conv9 = RegularConv2d(n_ch[1] * 2, n_ch[1])
        self.conv10 = nn.Conv2d(n_ch[1], classes_num, 1, 1, padding=0)

        # for detection outputs
        self.convF1a = nn.Conv2d(n_ch[5], n_ch[5], 3, 1, padding=1)
        self.convF1b_objness = nn.Conv2d(n_ch[5], 100, 3, 1, padding=1)
        self.convF1c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF1b_paral_clses = ParallelConv2d(
            n_ch[5], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF1c_paral_clses = ParallelConv2d(
            100,
            1 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )
        self.convF1b_paral_coors = ParallelConv2d(
            n_ch[5], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF1c_paral_coors = ParallelConv2d(
            100,
            4 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )

        self.convF2a = nn.Conv2d(n_ch[4], n_ch[4], 3, 1, padding=1)
        self.convF2b_objness = nn.Conv2d(n_ch[4], 100, 3, 1, padding=1)
        self.convF2c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF2b_paral_clses = ParallelConv2d(
            n_ch[4], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF2c_paral_clses = ParallelConv2d(
            100,
            1 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )
        self.convF2b_paral_coors = ParallelConv2d(
            n_ch[4], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF2c_paral_coors = ParallelConv2d(
            100,
            4 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )

        self.convF3a = nn.Conv2d(n_ch[3], n_ch[3], 3, 1, padding=1)
        self.convF3b_objness = nn.Conv2d(n_ch[3], 100, 3, 1, padding=1)
        self.convF3c_objness = nn.Conv2d(100, 1 * anchor_base_n, 1, 1, padding=0)
        self.convF3b_paral_clses = ParallelConv2d(
            n_ch[3], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF3c_paral_clses = ParallelConv2d(
            100,
            1 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )
        self.convF3b_paral_coors = ParallelConv2d(
            n_ch[3], 100, 3, 1, padding=1, activation=relu, classes_num=classes_num
        )
        self.convF3c_paral_coors = ParallelConv2d(
            100,
            4 * anchor_base_n,
            1,
            1,
            padding=0,
            activation=torch.sigmoid,
            classes_num=classes_num,
        )

        # for roi regression
        self.roi_conv_in = nn.Conv2d(
            n_roi_ch[0], n_roi_ch[1], 3, 1, padding=1
        )  # Starts the roi branch
        self.roi_conv1 = RegularConv2d(n_roi_ch[1], n_roi_ch[2])
        self.roi_conv2 = RegularConv2d(n_roi_ch[2], n_roi_ch[3])
        self.roi_branch_Y1 = RegularConv2d(n_roi_ch[3], n_roi_ch[3])
        self.roi_branch_Y2 = RegularConv2d(n_roi_ch[3], n_roi_ch[3])
        self.roi_fc1_Y1s = ParallelFC(
            n_roi_ch[3] * roi_align_h * roi_align_w // (4 * 4), 1000, activation=relu
        )
        self.roi_fc2_Y1s = ParallelFC(1000, 1 + 4, activation=torch.sigmoid)
        self.roi_fc1_Y2 = nn.Linear(
            n_roi_ch[3] * roi_align_h * roi_align_w // (4 * 4), 1000
        )
        self.roi_fc2_Y2 = nn.Linear(1000, 2 + 18)

        self.test_time = test_time
