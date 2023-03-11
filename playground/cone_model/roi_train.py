
from roi_generator import *
import random
from roi_model import *
from roi_loss import *
import time
import torch.optim as optim


dirs = ['/home/jinling/data/Tj2_2023/scorpion_samples/',
        '/home/jinling/data/Tj2_2023/train_refine/']
img_json_pairs = []
for dir in dirs:
    files = os.listdir(dir)
    names = [file.strip('.json') for file in files if file.find('.json')!=-1]
    img_names = [name+'.jpg' if os.path.exists(os.path.join(dir, name+'.jpg')) else name+'.png' for name in names]
    json_names = [name.strip('.jpg').strip('.png')+'.json' for name in img_names]
    pairs = [[os.path.join(dir, img), os.path.join(dir, json)] for img, json in zip(img_names, json_names)]
    img_json_pairs += pairs

model = Net().to(device)
model.train()
last_batch_time = time.time()
for batch_idx in range(batch_nums):
    if batch_idx == 0:
        optimizer = optim.Adadelta(model.parameters(), lr=0.1)
    if batch_idx == 5000:
        optimizer = optim.Adadelta(model.parameters(), lr=0.01)
    batch_pairs = random.sample(img_json_pairs, batch_size)
    imgs, targets, objs, roi_samples = generator(batch_pairs)
    data = torch.tensor(imgs.transpose(0, 3, 1, 2)).to(device).float()/255
    output = model(data, mode='training', targets = targets, objs = objs, roi_samples=roi_samples)
    pos_detect_loss, neg_detect_loss, regression_loss, \
    roi_reg_loss, roi_pos_neg_loss, roi_stand_loss, roi_lr_loss, roi_angle_loss, masks_loss, dice_loss = \
    get_loss(output, targets, objs)

    optimizer.zero_grad()
    loss = pos_detect_loss*pos_detect_weight + neg_detect_loss*neg_detect_weight + regression_loss*regression_weight +\
        roi_reg_loss*roi_reg_weight + roi_pos_neg_loss*roi_pos_neg_loss + \
           roi_stand_loss*roi_stand_weight + roi_lr_loss*roi_lr_weight + roi_angle_loss*roi_angle_weight + \
           masks_loss*masks_weight + dice_loss*dice_weight

    loss.backward()
    optimizer.step()
    optimizer.zero_grad()  # clear the gradient, for accumulated gradient approach, don't do this in every iteration

    print(batch_idx,
          '  %.8f:   %.8f, %.8f, %.8f --- %.8f, %.8f, %.8f, %.8f, %.8f --- %.8f, %.8f, time:%f' % (
              loss.item(), pos_detect_loss.item(), neg_detect_loss.item(), regression_loss.item(), \
              roi_reg_loss.item(), roi_pos_neg_loss.item(),
              roi_stand_loss.item(), roi_lr_loss.item(), roi_angle_loss.item(),
              masks_loss.item(), dice_loss.item(), time.time() - last_batch_time))
    last_batch_time = time.time()
    if batch_idx % 1000 == 0 and batch_idx > 0:
        torch.save(model.state_dict(), 'models/roi' + '_%06d.pkl' % (batch_idx))
