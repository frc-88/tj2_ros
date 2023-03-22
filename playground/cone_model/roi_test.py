
import os
from roi_params import *
from roi_generator import *
import random
from roi_deep_model import *
from roi_loss import *
import time
import torch.optim as optim

dir = '/home/jinling/data/Tj2_2023/scorpion_samples'
dir = '/home/jinling/data/Tj2_2023/game_pieces_2023_03_12/'
#dir = '/home/jinling/data/Tj2_2023/FIRST_bgs/'
dir = '/home/jinling/data/Tj2_2023/2023.3.18Collection/2/'
dir = '/home/jinling/data/Tj2_2023/2023.3.18Collection/4/'
dir = '/home/jinling/data/Tj2_2023/2023.3.18Collection/5/'
#dir = '/home/jinling/data/Tj2_2023/2023.3.18Collection/6/'
out_dir = '/home/jinling/data/Tj2_2023/output'
#dir = '/home/jinling/data/Tj2_2023/train_refine/'
files = sorted(os.listdir(dir))

img_names = [file for file in files if '.png' in file or '.jpg' in file]

model = Net().to(device)
model.load_state_dict(torch.load('models/roi_deep_038000.pkl'))
model.eval()

colors = np.array([[255,0,0],[0,255,0]], dtype=np.uint8)

paints = {
    'purple':np.array([180,120,255], dtype=np.uint8),
    'yellow':np.array([0,255,255], dtype=np.uint8)
}

for id, img_name in enumerate(img_names):
    path = os.path.join(dir, img_name)
    img = cv2.imread(path)
    img = cv2.resize(img, (IMG_W, IMG_H))
    data = torch.tensor(img.transpose(2,0,1)).to(device).float() / 255
    data = data.reshape((-1,)+data.shape)
    segments, mask_qs = model(data, mode='testing')
    mask = np.zeros(img.shape, dtype=np.uint8)
    for segment in segments:
        img_idx, cls_idx, ct, cb, cl, cr, stand, l_r, angle, sub_mask = segment
        sub_mask = sub_mask.cpu().detach().numpy()
        sub_mask = ((sub_mask>0.5)*255).astype(np.uint8)
        angle = angle.item()
        #paint = paints['purple'] if cls_idx == 1 else paints['yellow']
        if cls_idx == 1:
            paint = np.array([0, 255, 0], dtype=np.uint8)
        elif stand > 0.5:
            paint = np.array([255, 255, 255], dtype=np.uint8)
        elif l_r < 0.5:
            paint = np.array([255*angle, 0, 0], dtype=np.uint8)
        else:
            paint = np.array([0, 0, 255*angle], dtype=np.uint8)
        mask[ct:cb+1,cl:cr+1][np.where(sub_mask>0.5)] = paint

        mask[ct,cl:cr+1] = colors[cls_idx]
        mask[cb,cl:cr+1] = colors[cls_idx]
        mask[ct:cb+1,cl] = colors[cls_idx]
        mask[ct:cb+1,cr] = colors[cls_idx]

        img[ct,cl:cr+1] = colors[cls_idx]
        img[cb,cl:cr+1] = colors[cls_idx]
        img[ct:cb+1,cl] = colors[cls_idx]
        img[ct:cb+1,cr] = colors[cls_idx]






    cv2.imshow('img', img)
    cv2.imshow('mask', mask)
    cv2.waitKey(200)
    save_name = img_name.split('.')[0]+'.png'
    #cv2.imwrite(os.path.join(out_dir, save_name), img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    '''
    ws = [s[3]-s[2] for s in segments] if len(segments) > 0 else [0]
    hs = [s[5]-s[4] for s in segments] if len(segments) > 0 else [0]
    maxx = max((max(ws), max(hs)))
    print(id, '%3d: '%maxx, path)
    '''

    '''
    for img_idx in range(mask_qs.shape[0]):
        for cls_idx in range(mask_qs.shape[1]):
            mask_q = (mask_qs[img_idx, cls_idx].cpu().detach().numpy()*255).astype(np.uint8)
            cv2.imshow('mask_q', mask_q)
            cv2.waitKey(1000)
            print('here.')
    '''
    print(id, ': ', path)
