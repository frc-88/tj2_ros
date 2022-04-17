import cv2
import time
import tqdm
import numpy as np

# shrink output video with:
# ffmpeg -i input.mp4 -vcodec libx265 -crf 28 output.mp4
# see: https://unix.stackexchange.com/questions/28803/how-can-i-reduce-a-videos-size-with-ffmpeg

path1 = "/home/benjamin/Videos/new-amcl-on-week3-2022-04-16_22.05.39.mp4"
path2 = "/home/benjamin/Videos/week3-2022-04-16_22.08.54.mp4"
out_path = "/home/benjamin/Videos/2022-04-16_22.05.39-new-vs-old.mp4"

video1 = cv2.VideoCapture(path1)
video2 = cv2.VideoCapture(path2)

paused = False

width = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2
height = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2
length = max(video1.get(cv2.CAP_PROP_FRAME_COUNT), video2.get(cv2.CAP_PROP_FRAME_COUNT))
fps = video1.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height * 2))

empty_frame = None

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
            image1 = cv2.resize(image1, (width, height))
            image2 = cv2.resize(image2, (width, height))

            concat_image = np.concatenate((image1, image2))
            # concat_image = cv2.addWeighted(image1, 0.5, image2, 0.5, 0.0)
            writer.write(concat_image)

            # cv2.imshow("video", concat_image)
    
finally:
    print("Writing to %s" % out_path)
    writer.release()
    video1.release()
    video2.release()
    cv2.destroyAllWindows()
