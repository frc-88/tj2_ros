import cv2


def main():
    window_name = "video"

    cv2.namedWindow(window_name)

    capture = cv2.VideoCapture(4)  # , cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    print(capture.get(cv2.CAP_PROP_FRAME_WIDTH), capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    while True:
        success, image = capture.read()
        if not success:
            print("No more frames. Exiting")
            break

        cv2.imshow(window_name, image)
        value = cv2.waitKey(1)
        key = chr(value & 0xff)

        if key == 'q':
            break


if __name__ == '__main__':
    main()
