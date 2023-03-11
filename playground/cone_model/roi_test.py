
import os
from roi_params import *
from roi_generator import *
import random
from roi_model import *
from roi_loss import *
import time
import torch.optim as optim

dir = '/home/jinling/data/Tj2_2023/scorpion_samples'
out_dir = '/home/jinling/data/Tj2_2023/output'
#dir = '/home/jinling/data/Tj2_2023/train_refine/'
files = os.listdir(dir)

names = [file.strip('.json') for file in files if file.find('.json')!=-1]
img_names = [name+'.jpg' if os.path.exists(os.path.join(dir, name+'.jpg')) else name+'.png' for name in names]

model = Net().to(device)
model.load_state_dict(torch.load('models/roi_001683.pkl'))
model.eval()

colors = np.array([[255,0,0],[0,255,0]], dtype=np.uint8)

for img_name in img_names:
    path = os.path.join(dir, img_name)
    img = cv2.imread(path)
    img = cv2.resize(img, (IMG_W, IMG_H))
    data = torch.tensor(img.transpose(2,0,1)).to(device).float() / 255
    data = data.reshape((-1,)+data.shape)
    segments = model(data, mode='testing')
    mask = np.zeros(img.shape)
    for segment in segments:
        img_idx, cls_idx, ct, cb, cl, cr, stand, l_r, angle, sub_mask = segment
        sub_mask = sub_mask.cpu().detach().numpy()
        sub_mask = ((sub_mask>0.5)*255).astype(np.uint8)
        mask[ct:cb+1,cl:cr+1][np.where(sub_mask>0.5)] = colors[cls_idx]

        mask[ct,cl:cr+1] = colors[cls_idx]
        mask[cb,cl:cr+1] = colors[cls_idx]
        mask[ct:cb+1,cl] = colors[cls_idx]
        mask[ct:cb+1,cr] = colors[cls_idx]

        img[ct,cl:cr+1] = colors[cls_idx]
        img[cb,cl:cr+1] = colors[cls_idx]
        img[ct:cb+1,cl] = colors[cls_idx]
        img[ct:cb+1,cr] = colors[cls_idx]
        angle = str(int(angle.item()*180))

        ss = 'S' if stand > 0.5 else 'D'
        l_r = 'L' if l_r < 0.5 else 'R'
        if ss == 'S' or classes[cls_idx] != 'cone':
            angle = '0'
        label = ss+' '+l_r+' '+angle
        cxx, cyy = cl, ct + 5
        #img[cyy:cyy + 20, cxx:cxx + 40] = np.array([255, 255, 255], dtype=np.uint8)
        #cv2.putText(img, label, (cxx, cyy + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
        '''

        img = cv2.resize(img, (IMG_W*3, IMG_H*3))
        mask = cv2.resize(mask, (IMG_W * 3, IMG_H * 3))
        ct, cb, cl, cr = [x*3 for x in [ct, cb, cl, cr]]
        mask[ct, cl:cr + 1] = colors[cls_idx]
        mask[cb, cl:cr + 1] = colors[cls_idx]
        mask[ct:cb + 1, cl] = colors[cls_idx]
        mask[ct:cb + 1, cr] = colors[cls_idx]

        img[ct, cl:cr + 1] = colors[cls_idx]
        img[cb, cl:cr + 1] = colors[cls_idx]
        img[ct:cb + 1, cl] = colors[cls_idx]
        img[ct:cb + 1, cr] = colors[cls_idx]
        angle = str(int(angle.item() * 180))

        ss = 'S' if stand > 0.5 else 'D'
        l_r = 'L' if l_r < 0.5 else 'R'
        if ss == 'S' or classes[cls_idx] != 'cone':
            angle = '0'
        label = ss + ' ' + l_r + ' ' + angle
        cxx, cyy = cl, ct + 5

        img[cyy:cyy + 20, cxx:cxx + 65] = np.array([255, 255, 255], dtype=np.uint8)
        cv2.putText(img, label, (cxx, cyy + 20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
        '''
    cv2.imshow('img', img)
    cv2.imshow('mask', mask)
    cv2.waitKey(200)
    save_name = img_name.split('.')[0]+'.png'
    #cv2.imwrite(os.path.join(out_dir, save_name), img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print('here. ')

# when collecting new images, give some fake cones and cubes.
# some half seen