import cv2

path = "/dev/v4l/by-path/platform-3610000.xhci-usb-0:2.3:1.0-video-index0"
auto_exposure = 0
exposure = 157
gain = 0
width = 1600
height = 1200
video = cv2.VideoCapture(
    f"v4l2src device={path} "
    f'extra_controls="c,exposure_auto={auto_exposure},'
    f"exposure_absolute={exposure},"
    f'gain={gain},sharpness=0,brightness=0" ! '
    f"image/jpeg,format=MJPG,width={width},height={height} ! "
    f"jpegdec ! video/x-raw ! appsink drop=1",
    cv2.CAP_GSTREAMER,
)

while True:
    success, frame = video.read()
    if not success:
        print("Failed to read")
        break
    print("video0", frame.shape)
