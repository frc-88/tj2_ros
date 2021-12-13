import cv2
import numpy as np

max_value = 255
column_width = 4
height = 50
image = np.zeros((height, column_width * (max_value + 1), 3), np.uint8)
for h in range(0, max_value + 1):
    pixel = np.array([[[h, 255, 255]]]).astype(np.uint8)
    pixel = cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)
    column = np.zeros((height, column_width, 3), np.uint8)
    # if h < 90 or h > 120:
    if h < 35 or h > 90:
        column[..., 0:3] = np.array([0, 0, 0])
    else:
        column[..., 0:3] = pixel[0:3]

    image[0:height, h * column_width:(h + 1) * column_width, 0:3] = column[..., 0:3]

pixel = np.array([[[255, 0, 0]]]).astype(np.uint8)
pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
print(pixel)


cv2.imshow("image", image)
cv2.waitKey(-1)
