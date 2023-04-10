
# This model is based on crop_branch_q_all
# crop means the second stage is cropped, branch means means two stages can be trained seperately, q means mask_qs to accelerate convergence, all means overall detection loss.
# shallow means this model has less layers

from pick_generator import *
import random
from roi_long_model import Net
from pick_loss import *
from roi_params import *
import time
import torch.optim as optim

dirs = [
    '/home/jinling/data/Tj2_2023/rmv_small/train_refine',
'/home/jinling/data/Tj2_2023/rmv_small/FIRST_bgs',
'/home/jinling/data/Tj2_2023/rmv_small/game_pieces_2023_03_12',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_1',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_2',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_3',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_4',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_5',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_6',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_7',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_8',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_9',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_10',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_11',
'/home/jinling/data/Tj2_2023/rmv_small/Reading_12',
'/home/jinling/data/Tj2_2023/rmv_small/scorpion_field_recording',
        ]

img_json_pairs = []
for dir in dirs:
    files = os.listdir(dir)
    names = [file.strip('.json') for file in files if '.json' in file]
    names = [name for name in names if os.path.exists(os.path.join(dir, name+'.jpg')) or os.path.exists(os.path.join(dir, name+'.png'))]
    img_names = [name+'.jpg' if os.path.exists(os.path.join(dir, name+'.jpg')) else name+'.png' for name in names]
    json_names = [name+'.json' for name in names]
    pairs = [[os.path.join(dir, img), os.path.join(dir, json)] for img, json in zip(img_names, json_names)]
    img_json_pairs += pairs

def load_stage1(frm, to_model): # e.g. model = load_stage1('models/angle_034000.pkl', model)
    model1 = Net().to(device)
    model1.load_state_dict(torch.load(frm))
    model_dict = to_model.state_dict().copy()
    param_names = list(model.state_dict().keys())
    for name in param_names:
        if 'crop_' not in name:
            model_dict[name] = model1.state_dict()[name]
    to_model.load_state_dict(model_dict)
    return to_model

model = Net().to(device)
model.train()
last_batch_time = time.time()
training_mode = 'training'
print('When loading a trained model, correctly set the values of optimizer and training_mode. ')

'''
import roi_model
model1 = roi_model.Net().to(device)
model1.load_state_dict(torch.load('models/roi_8_32_080000.pkl'))
model_dict = model.state_dict().copy()
param_names = list(model.state_dict().keys())
for name in param_names:
    if 'roi_' not in name:
        model_dict[name] = model1.state_dict()[name]
model.load_state_dict(model_dict)
'''

for batch_idx in range(batch_nums):
    if batch_idx == 0:
        training_mode = 'training_stage1'
    if batch_idx == 29000:
        training_mode = 'training_stage2'
    if batch_idx == 30000:
        training_mode = 'training'
    if batch_idx == 0:
        optimizer = optim.Adadelta(model.parameters(), lr=0.1)
    if batch_idx == 5000:
        optimizer = optim.Adadelta(model.parameters(), lr=0.01)
    if training_mode == 'training_stage1':
        for name, param in model.named_parameters():
            param.requires_grad = True if 'roi_' not in name else False
    if training_mode == 'training_stage2':
        for name, param in model.named_parameters():
            param.requires_grad = True if 'roi_' in name else False
    if training_mode == 'training':
        for name, param in model.named_parameters():
            param.requires_grad = True
    batch_pairs = random.sample(img_json_pairs, batch_size)
    imgs, targets, objs, roi_samples, tgt_mask_qs = generator(batch_pairs)
    data = torch.tensor(imgs.transpose(0, 3, 1, 2)).to(device).float()/255
    output = model(data, mode=training_mode, objs = objs, roi_samples=roi_samples)
    pos_detect_loss, neg_detect_loss, all_detect_loss, press_loss, classification_loss, regression_loss,\
    roi_reg_loss, roi_pos_neg_loss, roi_intact_loss, roi_stand_loss, roi_angle_loss, masks_loss = \
    get_loss(imgs, training_mode, output, targets, objs)

    optimizer.zero_grad()
    loss = pos_detect_loss*pos_detect_weight + neg_detect_loss*neg_detect_weight + regression_loss*regression_weight +\
           all_detect_loss*all_detect_weight + press_loss*press_weight + classification_loss*classification_weight +\
        roi_reg_loss*roi_reg_weight + roi_pos_neg_loss*roi_pos_neg_loss + \
           +roi_intact_loss*roi_intact_weight + roi_stand_loss*roi_stand_weight + roi_angle_loss*roi_angle_weight  + \
           masks_loss*masks_weight
    # whay the number of neg samples affects the backward so much
    loss.backward()
    optimizer.step()
    optimizer.zero_grad()  # clear the gradient, for accumulated gradient approach, don't do this in every iteration

    print(batch_idx,
          '  %.6f:   %.6f, %.6f, %.6f, %.6f --- %.6f, %.6f, %.6f --|-- %.6f, %.6f --- %.6f, %.6f, %.6f, time:%f' % (
              loss.item(), \
              pos_detect_loss.item(), neg_detect_loss.item(), all_detect_loss.item(), press_loss.item(),
              classification_loss.item(), regression_loss.item(), masks_loss.item(),
              roi_pos_neg_loss.item(), roi_reg_loss.item(),
              roi_intact_loss.item(), roi_stand_loss.item(), roi_angle_loss.item(),

            time.time() - last_batch_time))
    last_batch_time = time.time()
    if batch_idx % 1000 == 0 and batch_idx > 0:
        torch.save(model.state_dict(), 'models/roi' + '_%06d.pkl' % (batch_idx+100000))
