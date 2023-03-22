
import torch

batch_size = 4
batch_nums = 100000+1

# the dimension of the input image width and height
# have to be integer multiples of 16
#IMG_W, IMG_H = 1024, 512
#anchor_base_sizes = [200, 100, 30]
IMG_W, IMG_H = 512, 256
#IMG_W, IMG_H = 1024, 512
#anchor_base_sizes = [46, 26, 16]
# the base size is "half" of the height or width!!!
anchor_base_sizes = [53, 28, 16]
#anchor_base_sizes = [53*2, 28*2, 16*2]

anchor_skips = [16, 8, 4] # constant for this model, cannot take any other values
targets_sizes = [(IMG_H//16, IMG_W//16), (IMG_H//8, IMG_W//8), (IMG_H//4, IMG_W//4)]

# aspect ratios for anchors of different output layers
'''
anchor_aspect_ratios = \
[[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]],
[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]],
[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.6, 0.4], [0.4, 0.6], [0.6, 0.6]]]
'''
anchor_aspect_ratios = \
[[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.67, 0.46], [0.46, 0.67], [0.67, 0.67]],
[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.67, 0.46], [0.46, 0.67], [0.67, 0.67]],
[[1, 1], [1, 0.7], [0.7, 1], [1, 0.5], [0.5, 1], [1, 0.37], [0.37, 1], [0.67, 0.46], [0.46, 0.67], [0.67, 0.67]]]
# the numbers of anchors of each point in each output layer have to be the same
assert(len(anchor_aspect_ratios[0])==len(anchor_aspect_ratios[1])==len(anchor_aspect_ratios[2]))

#the number of anchors of each point in each output layer
anchor_base_n = len(anchor_aspect_ratios[0])

classes = ['cone', 'cube']
classes_num = len(classes)

# for the first stage selecting samples
iou_pos_sample_thresh = 0.7
iou_reg_sample_thresh = 0.55
iou_neg_sample_thresh = 0.3

# for the second stage selecting samples from the region proposals
roi_iou_pos_thresh = 0.75
roi_iou_reg_thresh = 0.7
roi_iou_neg_thresh = 0.3

reg_sample_quota_each_obj = 5
# for the first stage, more neg samples doesn't affect training speed.
# overlapped neg samples
neg_ol_sample_quota_each_obj = 4
# background neg samples
neg_bg_sample_quota = 5000
# for the second stage, the number of neg samples affects training speed.
roi_neg_ol_sample_quota_each_obj = 4
roi_neg_bg_sample_quota = 12
# roi samples from the nms results
roi_pos_nms_sample_quota_each_class = 30
roi_neg_nms_sample_quota_each_class = 30

n_ch = [3, 32, 64, 128, 256, 512]

device = torch.device('cuda')

box_cred_th_nms_train = 0.7
box_cred_th_nms_test = 0.6
maximumn_obj_num = 400

roi_score_th_train = 0.6  # after the roi, know whether it's an object.
roi_score_th_test = 0.6

roi_align_h, roi_align_w = 20, 20

neg_overlap_ratio = 0.2 # ensure a certain amount of overlaped neg samples for detection

nms_iou_thresh = 0.2  # whose iou with the current box is larger than this one will be deleted

pos_detect_weight, neg_detect_weight, regression_weight, roi_reg_weight, roi_pos_neg_weight, masks_weight, dice_weight = 1, 1, 1, 1, 1, 1, 1
roi_stand_weight, roi_lr_weight, roi_angle_weight = 1, 1, 1
mask_qs_weight = 1


reg_range_d, reg_range_u = 0, 2#-4, 6 #0, 2 # regression range down and up
# for the second regression, with a better resolution
roi_reg_range_d, roi_reg_range_u = 1-0.5, 1+0.5
