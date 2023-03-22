
import cv2
import numpy as np
import os

dir = 'C:/FILES/TJ square/2023/2023.3.18Collection/'
video_name = '6.mp4'
path = os.path.join(dir, video_name)
folder_name = video_name.strip('.mp4')
save_folder_path = os.path.join(dir, folder_name)
if not os.path.exists(save_folder_path):
    os.makedirs(save_folder_path)
cap = cv2.VideoCapture(path)

idx = 0
while True:
    ret, frame = cap.read()
    file_name = '%05d.png'%idx
    save_path = os.path.join(save_folder_path, file_name)
    cv2.imwrite(save_path, frame)
    cv2.imshow('frame', frame)
    cv2.waitKey(1)
    print(idx)
    idx += 1

print('Finished. ')
