import torch
from dataclasses import dataclass


@dataclass
class ChargedUpPerceptionParameters:
    batch_size = 4
    batch_nums = 200000 + 1

    # the dimension of the input image width and height
    # have to be integer multiples of 16
    # IMG_W, IMG_H = 1024, 512
    # anchor_base_sizes = [200, 100, 30]
    IMG_W, IMG_H = 512, 256
    CROP_W, CROP_H = 32, 32
    # IMG_W, IMG_H = 1024, 512
    # anchor_base_sizes = [46, 26, 16]
    # the base size is "half" of the height or width!!!
    # anchor_base_sizes = [53, 28, 16]

    # anchor_base_sizes = [53*2, 28*2, 16*2]
    anchor_base_sizes = [40, 23, 10]
    anchor_skips = [16, 8, 4]  # constant for this model, cannot take any other values
    targets_sizes = [
        (IMG_H // 16, IMG_W // 16),
        (IMG_H // 8, IMG_W // 8),
        (IMG_H // 4, IMG_W // 4),
    ]

    # aspect ratios for anchors of different output layers
    """
    anchor_aspect_ratios = \
    [[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]],
    [[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]],
    [[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]]]
    """
    anchor_aspect_ratios = [
        [
            [1, 1],
            [1, 0.7],
            [0.7, 1],
            [1, 0.5],
            [0.5, 1],
            [1, 0.37],
            [0.37, 1],
            [0.67, 0.46],
            [0.46, 0.67],
            [0.67, 0.67],
        ],
        [
            [1, 1],
            [1, 0.7],
            [0.7, 1],
            [1, 0.5],
            [0.5, 1],
            [1, 0.37],
            [0.37, 1],
            [0.67, 0.46],
            [0.46, 0.67],
            [0.67, 0.67],
        ],
        [
            [1, 1],
            [1, 0.7],
            [0.7, 1],
            [1, 0.5],
            [0.5, 1],
            [1, 0.37],
            [0.37, 1],
            [0.67, 0.46],
            [0.46, 0.67],
            [0.67, 0.67],
        ],
    ]
    # the numbers of anchors of each point in each output layer have to be the same
    assert (
        len(anchor_aspect_ratios[0])
        == len(anchor_aspect_ratios[1])
        == len(anchor_aspect_ratios[2])
    )

    # the number of anchors of each point in each output layer
    anchor_base_n = len(anchor_aspect_ratios[0])

    classes = ["cone", "cube"]
    classes_num = len(classes)

    # for the first stage selecting samples
    iou_pos_sample_thresh = 0.8
    iou_reg_sample_thresh = 0.7
    iou_neg_sample_thresh = 0.35
    # iou_neg_within_sample_thresh = 0.45
    # iou_within_sample_thresh = 0.8

    # press_sample_diff_th = 0.0
    press_neg_sample_each_img = 200
    top_press_neg_sample_each_img = 20
    assert press_neg_sample_each_img > top_press_neg_sample_each_img

    # for the second stage selecting samples from the region proposals
    # after the first stage and NMS, most false positives have lower IOUs, so lower the ths here.
    roi_iou_pos_thresh = 0.83
    roi_iou_reg_thresh = 0.75
    roi_iou_neg_thresh = 0.35
    assert (
        iou_pos_sample_thresh > iou_reg_sample_thresh
        and iou_reg_sample_thresh > iou_neg_sample_thresh
    )
    assert (
        roi_iou_pos_thresh > roi_iou_reg_thresh
        and roi_iou_reg_thresh > roi_iou_neg_thresh
    )
    """
    assert(roi_iou_pos_thresh > iou_pos_sample_thresh and \
        roi_iou_reg_thresh > iou_reg_sample_thresh and \
        roi_iou_neg_thresh > iou_neg_sample_thresh)
    """
    reg_sample_quota_each_obj = 5
    # for the first stage, more neg samples doesn't affect training speed.
    # overlapped neg samples
    neg_ol_sample_quota_each_obj = 5  # two many overlaped neg samples will lead to the the case that positives are not detected
    # neg_within_sample_quota_each_ojb = 25
    # background neg samples
    neg_bg_sample_quota = 80
    # for the second stage, the number of neg samples affects training speed.
    roi_neg_ol_sample_quota_each_obj = 0  # two many overlaped neg samples will lead to the the case that positives are not detected
    # roi_neg_bg_sample_quota = 12
    # roi samples from the nms results
    roi_pos_nms_sample_quota_each_class = 30
    roi_neg_nms_sample_quota_each_class = 40

    # 8_32
    n_ch = [3, 8, 16, 32, 64, 128]
    n_roi_ch = [n_ch[3], 32, 64, 128]
    # 8_64
    # n_ch = [3, 8, 16, 32, 64, 128]
    # n_roi_ch = [n_ch[3], 64, 128, 256]
    # 16_32
    # n_ch = [3, 16, 32, 64, 128, 256]
    # n_roi_ch = [n_ch[3], 32, 64, 128]
    # 16_64
    # n_ch = [3, 16, 32, 64, 128, 256]
    # n_roi_ch = [n_ch[3], 64, 128, 256]

    device = torch.device("cuda")

    # give the train mode a smaller value so that more false positives
    # are generated for te second stage training.
    box_cred_th_nms_train = 0.8
    box_cred_th_nms_test = 0.85  # 0.85
    box_cred_cls_confi_th_nms_train = 0.85
    box_cred_cls_confi_th_nms_test = 0.85
    if not (box_cred_th_nms_train < box_cred_th_nms_test):
        print("It is highly recommended that box_cred_th_nms_test has a larger value. ")

    maximumn_obj_num = 400

    roi_score_th = (
        0.5  # # after the roi, know whether it's an object. Only used in test mode.
    )

    roi_align_h, roi_align_w = 20, 20

    neg_overlap_ratio = (
        0.2  # ensure a certain amount of overlapped neg samples for detection
    )

    nms_iou_thresh = (
        0.2  # whose iou with the current box is larger than this one will be deleted
    )

    pos_detect_weight, neg_detect_weight = 2, 2
    all_detect_weight = 1
    classification_weight = 1
    regression_weight, roi_reg_weight = 1, 1
    roi_pos_neg_weight = 2
    masks_weight, dice_weight = 1, 1
    roi_intact_weight = 1
    roi_stand_weight, roi_angle_weight = 1, 1
    mask_qs_weight = 1
    press_weight = 1

    reg_range_d, reg_range_u = 0, 2  # -4, 6 #0, 2 # regression range down and up
    # for the second regression, with a better resolution
    roi_reg_range_d, roi_reg_range_u = 1 - 0.5, 1 + 0.5

    TARGET_LABEL_EFF, TARGET_LABEL_REG = 1, 2
