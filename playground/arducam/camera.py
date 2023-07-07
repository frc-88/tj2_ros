import cv2

path = "/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.3:1.0-video-index0"
video = cv2.VideoCapture(path)

while True:
    success, frame = video.read()
    if not success:
        print("Failed to read")
        break
    print("video", frame.shape)
