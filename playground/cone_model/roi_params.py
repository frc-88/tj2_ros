
import torch

batch_size = 4
batch_nums = 100000+1

# the dimension of the input image width and height
# have to be integer multiples of 16
#IMG_W, IMG_H = 1024, 512
#anchor_base_sizes = [200, 100, 30]
IMG_W, IMG_H = 512, 256
anchor_base_sizes = [200//2, 100//2, 30//2]

anchor_skips = [16, 8, 4]
targets_sizes = [(IMG_H//16, IMG_W//16), (IMG_H//8, IMG_W//8), (IMG_H//4, IMG_W//4)]

# aspect ratios for anchors of different output layers
anchor_aspect_ratios = \
[[[1, 1], [1, 0.75], [0.75, 1], [0.75, 0.75], [1, 0.5], [0.5, 1], [0.75, 0.5], [0.5, 0.75]],
[[1, 1], [1, 0.6], [0.6, 1], [0.75, 0.75], [1, 0.3], [0.3, 1], [0.75, 0.5], [0.5, 0.75]],
[[1, 1], [1, 0.6], [0.6, 1], [0.6, 0.6], [0.4, 0.4], [0.8, 0.8], [0.6, 0.4], [0.4, 0.6]]]

# the numbers of anchors of each point in each output layer have to be the same
assert(len(anchor_aspect_ratios[0])==len(anchor_aspect_ratios[1])==len(anchor_aspect_ratios[2]))

#the number of anchors of each point in each output layer
anchor_base_n = len(anchor_aspect_ratios)

classes = ['cone', 'cube']
classes_num = len(classes)

iou_pos_thresh = 0.75
iou_reg_thresh = 0.55
iou_neg_thresh = 0.45

reg_sample_quota_each_obj = 5
neg_sample_quota_each_obj = 10

n_ch = [3, 32, 64, 128, 256, 512]

device = torch.device('cuda')

box_cred_th_nms = 0.7
maximumn_obj_num = 20

roi_score_th = 0.65  # after the roi, know whether it's an object.

roi_align_h, roi_align_w = 20, 20

neg_overlap_ratio = 0.7 # given more overlaped neg samples for detection

pos_detect_weight, neg_detect_weight, regression_weight, roi_reg_weight, roi_pos_neg_weight, masks_weight, dice_weight = 1, 1, 1, 1, 1, 1, 1
roi_stand_weight, roi_lr_weight, roi_angle_weight = 1, 1, 1
