
import os
import json
import cv2
from roi_params import *
import numpy as np

dirs = ['C:/FILES/TJ square/2023/data/scorpion_field_recording/scorpion_samples',
'C:/FILES/TJ square/2023/data/game_pieces_2023_03_12',
'C:/FILES/TJ square/2023/data/game_pieces_2023_03_12',
'C:/FILES/TJ square/2023/data/train_'
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

dots = np.zeros((IMG_H, IMG_W), dtype=np.uint8)

for img_idx, (img_path, json_path) in enumerate(img_json_pairs):
    img = cv2.imread(img_path)
    org_h, org_w = img.shape[:2]
    img = cv2.resize(img, (IMG_W, IMG_H))
    json_data = json.load(open(json_path, encoding='gbk'))
    for i in range(len(json_data['shapes'])):
        label = json_data['shapes'][i]['label']
        points = np.array(json_data['shapes'][i]['points'])
        points = [[int(p[0] * IMG_W / org_w + 0.5), int(p[1] * IMG_H / org_h + 0.5)] for p in points]
        if label == 'hide':
            cv2.fillPoly(img, [np.array(points, dtype=int)], (0, 0, 0))
            continue
        obj_t, obj_b = min([p[1] for p in points]), max([p[1] for p in points])
        obj_l, obj_r = min([p[0] for p in points]), max([p[0] for p in points])
        obj_w, obj_h = obj_r - obj_l + 1, obj_b - obj_t + 1
        dots[obj_h, obj_w] = 255

cv2.imshow('dots', dots)
cv2.imwrite('dots.png', dots)
cv2.waitKey(0)



