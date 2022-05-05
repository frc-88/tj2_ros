import cv2
import time
import tqdm
import numpy as np

# shrink output video with:
# ffmpeg -i input.mp4 -vcodec libx265 -crf 28 output.mp4
# see: https://unix.stackexchange.com/questions/28803/how-can-i-reduce-a-videos-size-with-ffmpeg

# convert to gif:
# ffmpeg -ss 30 -t 30 -i input.mp4 -vf "fps=10,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 output.gif
# see: https://superuser.com/questions/556029/how-do-i-convert-a-video-to-gif-using-ffmpeg-with-reasonable-quality

path1 = "/home/benjamin/Videos/Qualification 35 - 2022 FIRST Championship - Galileo Division.mp4"
path2 = "/home/benjamin/Videos/real-match-overlay-2022-04-24_21.44.55.mp4"
out_path = "/home/benjamin/Videos/TJ2 ROS Demo Qualification 35 - 2022 FIRST Championship - Galileo Division.mp4"

video1_start_time = 5.5
video2_start_time = 0.0

show = False

video1 = cv2.VideoCapture(path1)
video2 = cv2.VideoCapture(path2)

paused = False

width1 = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
height1 = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2
length = int(max(video1.get(cv2.CAP_PROP_FRAME_COUNT), video2.get(cv2.CAP_PROP_FRAME_COUNT)))
width2 = int(video2.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
height2 = int(video2.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2
fps1 = video1.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter(out_path, fourcc, fps1, (width1, height1 * 2))

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

video2_src = np.array([[0, 0], [width1, 0], [0, height1], [width1, height1]]).astype(np.float32)
# video2_coords = np.array([[0, 0], [0, width2], [height2, 0]]).astype(np.float32)
video2_coords = np.array([[113, 80], [727, 64], [-20, 393], [836, 381]]).astype(np.float32)
warp_mat = cv2.getPerspectiveTransform(video2_src, video2_coords)


# camera_matrix = np.array([
#     [10.0, 0.0, 10.0],
#     [0.0, 10.0, 10.0],
#     [0.0, 0.0, 1.0]
# ])
# distort_coeffs = np.array([
#     0.0,
#     0.0,
#     0.0,
#     0.0,
#     0.0,
# ])

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
            image1 = cv2.resize(image1, (width1, height1))
            image2 = cv2.resize(image2, (width1, height1))
            image2_overlay = np.copy(image2)

            mask = cv2.inRange(image2_overlay, (100, 100, 100), (200, 200, 200))
            image2_overlay[mask > 0] = (0, 0, 0)

            # image2 = cv2.undistort(image2, camera_matrix, distort_coeffs)
            image2_overlay = cv2.warpPerspective(image2_overlay, warp_mat, (width1, height1))

            concat_image = cv2.addWeighted(image1, 1.0, image2_overlay, 1.0, 0.5)
            concat_image = np.concatenate((concat_image, image2))
            writer.write(concat_image)

            if show:
                cv2.imshow(window_name, concat_image)
    
finally:
    print("Writing to %s" % out_path)
    writer.release()
    video1.release()
    video2.release()
    cv2.destroyAllWindows()
