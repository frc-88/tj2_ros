import cv2
import numpy as np
from stereo_calicam import StereoCaliCam



def main():
    window_name = "video"

    cv2.namedWindow(window_name)

    cam = StereoCaliCam(4, "21-181220-0172.yml")
    image = None

    def next_frame():
        nonlocal image
        left, right, success = cam.get_images()
        if not success:
            return False
        disparity = cam.disparity_image(left, right)
        # print(np.min(disparity), np.max(disparity))
        disparity_show = cam.normalize_disparity_int(disparity)
        disparity_show = cv2.cvtColor(disparity_show, cv2.COLOR_GRAY2BGR)
        image = np.concatenate((left, disparity_show), axis=1)
        # image = disparity_show
        # print(cam.get_distance(disparity, disparity.shape[0] // 2, disparity.shape[1] // 2))
        return True

    while True:
        if not next_frame():
            break

        cv2.imshow(window_name, image)
        value = cv2.waitKey(1)
        key = chr(value & 0xff)

        if key == 'q':
            break


if __name__ == '__main__':
    main()
