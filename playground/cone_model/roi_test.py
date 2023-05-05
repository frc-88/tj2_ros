
import os

import numpy as np

from roi_params import *
from roi_generator import *
import random
from roi_long_model import *
import time
import torch.optim as optim


dir = '/home/jinling/data/Tj2_2023/rmv_small/train_refine'
dir = '/home/jinling/data/Tj2_2023/rmv_small/FIRST_bgs'
dir = '/home/jinling/data/Tj2_2023/rmv_small/game_pieces_2023_03_12'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_1'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_2'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_3'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_4'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_5'
dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_6'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_7'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_8'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_9'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_10'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_11'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/Reading_12'
#dir = '/home/jinling/data/Tj2_2023/rmv_small/scorpion_field_recording'

files = sorted(os.listdir(dir))

img_names = [file for file in files if '.png' in file or '.jpg' in file]

model = Net().to(device)
model.load_state_dict(torch.load('models/bak/long_281000.pkl'))
model.eval()

colors = np.array([[255,0,0],[0,255,0]], dtype=np.uint8)

paints = {
    'purple':np.array([180,120,255], dtype=np.uint8),
    'yellow':np.array([0,255,255], dtype=np.uint8)
}

RADIUS = 14
cir0_xs, cir0_ys = [], []
for y in range(-RADIUS, RADIUS+1):
    for x in range(-RADIUS, RADIUS+1):
        if int(np.sqrt(x**2+y**2)+0.5) == RADIUS:
            cir0_xs.append(x)
            cir0_ys.append(y)

def disp_angle(img, cx, cy, angle, color):
    angle_ = angle * np.pi / 180
    ey = int(cy - RADIUS*np.sin(angle_))
    ex = int(cx + RADIUS*np.cos(angle_))
    if abs(ey-cy) > abs(ex-cx):
        k = (ex-cx)/(ey-cy)
        b = cx - k*cy
        ty, by = min(cy, ey), max(cy, ey)
        yys = [yy for yy in range(ty, by+1)]
        xxs = [int(k*yy+b+0.5) for yy in yys]
    else:
        k = (ey-cy)/(ex-cx)
        b = cy - k*cx
        lx, rx = min(cx, ex), max(cx, ex)
        xxs = [xx for xx in range(lx, rx+1)]
        yys = [int(k*xx+b+0.5) for xx in xxs]
    cir_xs, cir_ys = [cir_x + cx for cir_x in cir0_xs], [cir_y + cy for cir_y in cir0_ys]
    xxs, yys = xxs + cir_xs, yys + cir_ys
    x_ys = [[xx, yy] for xx, yy in zip(xxs, yys) if xx >= 0 and xx < IMG_W and yy >= 0 and yy < IMG_H]
    if len(x_ys) > 0:
        xxs, yys = zip(*x_ys)
        img[yys, xxs] = color
    return img

test_mode = 'testing_stage1'
test_mode = 'testing'
# segmentation is not good because the best samples are given for training.
for id, img_name in enumerate(img_names):
    path = os.path.join(dir, img_name)
    img = cv2.imread(path)
    img = cv2.resize(img, (IMG_W, IMG_H))
    data = torch.tensor(img.transpose(2,0,1)).to(device).float() / 255
    data = data.reshape((-1,)+data.shape)
    segments = model(data, mode=test_mode)
    mask = np.zeros(img.shape, dtype=np.uint8)
    for segment in segments:
        img_idx, cls_idx, ct, cb, cl, cr, intact, stand, angle, sub_mask = segment
        #sub_mask = sub_mask.cpu().detach().numpy()
        #sub_mask = ((sub_mask>0.5)*255).astype(np.uint8)
        angle = angle.item()
        #paint = paints['purple'] if cls_idx == 1 else paints['yellow']
        if classes[cls_idx] == 'cube':
            paint = np.array([0, 255, 0], dtype=np.uint8)
        elif not intact:
            paint = np.array([128, 128, 128], dtype=np.uint8)
        elif stand > 0.5:
            paint = np.array([255, 255, 255], dtype=np.uint8)
        else:
            paint = np.array([255, 0, 0], dtype=np.uint8)

        #mask[ct:cb+1,cl:cr+1][np.where(sub_mask>0.5)] = paint
        mask[ct:cb + 1, cl:cr + 1][np.where(sub_mask>0.5)] = paint

        mask[ct,cl:cr+1] = colors[cls_idx]
        mask[cb,cl:cr+1] = colors[cls_idx]
        mask[ct:cb+1,cl] = colors[cls_idx]
        mask[ct:cb+1,cr] = colors[cls_idx]

        img[ct,cl:cr+1] = colors[cls_idx]
        img[cb,cl:cr+1] = colors[cls_idx]
        img[ct:cb+1,cl] = colors[cls_idx]
        img[ct:cb+1,cr] = colors[cls_idx]

        angle_color = np.array([0, 0, 255], dtype=np.uint8)
        if classes[cls_idx] == 'cone' and intact > 0.5 and stand < 0.5:
            img = disp_angle(img, (cl+cr)//2, (ct+cb)//2, angle, angle_color)

    cv2.imshow('img', img)
    cv2.imshow('mask', mask)
    cv2.waitKey(1)
    save_name = img_name.split('.')[0]+'.png'
    #cv2.imwrite(os.path.join(out_dir, save_name), img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(id, ': ', path)
