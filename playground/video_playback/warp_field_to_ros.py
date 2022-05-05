import cv2
import time
import tqdm
import numpy as np

# shrink output video with:
# ffmpeg -i input.mp4 -vcodec libx265 -crf 28 output.mp4
# see: https://unix.stackexchange.com/questions/28803/how-can-i-reduce-a-videos-size-with-ffmpeg

path1 = "/home/benjamin/Videos/Qualification 35 - 2022 FIRST Championship - Galileo Division.mp4"
path2 = "/home/benjamin/Videos/real-match-overlay-2022-04-24_21.44.55.mp4"
out_path = "/home/benjamin/Videos/TJ2 ROS Demo Qualification 35 - 2022 FIRST Championship - Galileo Division.mp4"

video1_start_time = 5.5
video2_start_time = 0.0

show = True

video1 = cv2.VideoCapture(path1)
video2 = cv2.VideoCapture(path2)

paused = False

width1 = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH))
height1 = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT))
length = int(max(video1.get(cv2.CAP_PROP_FRAME_COUNT), video2.get(cv2.CAP_PROP_FRAME_COUNT)))
width2 = int(video2.get(cv2.CAP_PROP_FRAME_WIDTH))
height2 = int(video2.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps1 = video1.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter(out_path, fourcc, fps1, (width2, height2))

empty_frame = None
image1 = None
image2 = None

video1_start_frame = int(video1_start_time * fps1)
video2_start_frame = int(video2_start_time * fps1)

count = 0
while count < video1_start_frame:
    success, _ = video1.read()
    if not success:
        raise RuntimeError("Ran out of frames while skipping to the start!")
    count += 1

count = 0
while count < video2_start_frame:
    success, _ = video1.read()
    if not success:
        raise RuntimeError("Ran out of frames while skipping to the start!")
    count += 1



def mouse_callback(event, x, y, flags, param):
    if (event == cv2.EVENT_LBUTTONDOWN):
        print(x, y)
        if image1 is not None:
            print(image1[y, x])
        if image2 is not None:
            print(image2[y, x])

window_name = "video"
if show:
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

camera_matrix = np.array([
    [1000.0, 0.0, 155.0],
    [0.0, 1000.0, 126.0],
    [0.0, 0.0, 1.0]
])
distort_coeffs = np.array([
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
])


video1_src = np.array([[171, 119], [1025, 113], [-10, 494], [1194, 490]]).astype(np.float32)
video1_coords = np.array([[0, 0], [width2, 0], [0, height2], [width2, height2]]).astype(np.float32)
# video2_coords = np.array([[0, 0], [0, width2], [height2, 0]]).astype(np.float32)
warp_mat = cv2.getPerspectiveTransform(video1_src, video1_coords)

try:
    with tqdm.tqdm(total=length) as pbar:
        while True:
            key = cv2.waitKey(1)
            key &= 0xff
            key = chr(key)
            if key == 'q':
                break
            elif key == ' ':
                paused = not paused
                print("paused:", paused)
            if paused:
                time.sleep(0.05)
                continue

            success1, image1 = video1.read()
            if empty_frame is None:
                empty_frame = np.zeros_like(image1)
            if not success1:
                image1 = empty_frame
            success2, image2 = video2.read()
            if not success2:
                image2 = empty_frame
            if not success1 and not success2:
                break
            pbar.update(1)
            # image2 = cv2.resize(image2, (width2, height2))
            mask = cv2.inRange(image2, (100, 100, 100), (200, 200, 200))
            image2[mask > 0] = (0, 0, 0)

            # image1 = cv2.undistort(image1, camera_matrix, distort_coeffs)
            image1 = cv2.resize(image1, (width2, height2))
            image1 = cv2.warpPerspective(image1, warp_mat, (width2, height2))

            concat_image = cv2.addWeighted(image1, 1.0, image2, 1.0, 0.5)
            writer.write(concat_image)

            if show:
                cv2.imshow(window_name, concat_image)
    
finally:
    print("Writing to %s" % out_path)
    writer.release()
    video1.release()
    video2.release()
    cv2.destroyAllWindows()
